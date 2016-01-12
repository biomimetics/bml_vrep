#!/usr/bin/python

import rospy
import os, fnmatch

from vrep_common.srv import *
from vrep_common.msg import *
from keyboard.msg import *
from geometry_msgs.msg import *

VREP_ROOT_DIR = os.environ['VREP_ROOT_DIR']

# vrep Constants, see v_repConst.h
SIMROS_STRMCMD_SET_JOINT_STATE = 2051
SIMROS_STRMCMD_GET_OBJECT_POSE = 8193
SIMROS_STRMCMD_GET_VISION_SENSOR_IMAGE = 4108
SIMROS_STRMCMD_READ_FORCE_SENSOR = 4111
SIMROS_STRMCMD_GET_VISION_SENSOR_INFO = 4114

JOINT_MODE_CONTROL_VELOCITY = 2
JOINT_MODE_CONTROL_POSITION = 0

JOINT_OBJECT_TYPE = 1
VISION_SENSOR_OBJECT_TYPE = 9
FORCE_SENSOR_OBJECT_TYPE = 12

OBJECT_DATA_NAME = 0
OBJECT_DATA_TYPE = 1
OBJECT_DATA_PARENT = 2

SIM_HANDLE_ALL = -2
SIM_APPOBJ_OBJECT_TYPE = 109

# Default robot parameters (Approximately Zumy dimensions)
DEFAULT_ROBOT_D = 0.1
DEFAULT_ROBOT_R = 0.02

def directory_search(filename, directory=VREP_ROOT_DIR):
  for root, dirs, files in os.walk(directory):
    for basename in files:
      if fnmatch.fnmatch(basename, filename):
        found_filename = os.path.join(root, basename)
        return found_filename

class VrepMonitor():
  def __init__(self):
    rospy.init_node('vrep_monitor')
    
    rospy.Subscriber('/vrep/keyboard/keydown', Key, self.keyboard_callback, queue_size=1)

    rospy.wait_for_service('/vrep/simRosStopSimulation')
    self.stop_simulation = rospy.ServiceProxy('/vrep/simRosStopSimulation', simRosStopSimulation)

    rospy.wait_for_service('/vrep/simRosStartSimulation')
    self.start_simulation = rospy.ServiceProxy('/vrep/simRosStartSimulation', simRosStartSimulation)

    rospy.wait_for_service('/vrep/simRosPauseSimulation')
    self.pause_simulation = rospy.ServiceProxy('/vrep/simRosPauseSimulation', simRosPauseSimulation)

    rospy.wait_for_service('/vrep/simRosEnablePublisher')
    enable_publisher = rospy.ServiceProxy('/vrep/simRosEnablePublisher', simRosEnablePublisher)

    rospy.wait_for_service('/vrep/simRosEnableSubscriber')
    enable_subscriber = rospy.ServiceProxy('/vrep/simRosEnableSubscriber', simRosEnableSubscriber)

    rospy.wait_for_service('/vrep/simRosLoadScene')
    load_scene = rospy.ServiceProxy('/vrep/simRosLoadScene', simRosLoadScene)

    rospy.wait_for_service('/vrep/simRosLoadModel')
    load_model = rospy.ServiceProxy('/vrep/simRosLoadModel', simRosLoadModel)
    
    rospy.wait_for_service('/vrep/simRosSetObjectPose')
    set_object_pose = rospy.ServiceProxy('/vrep/simRosSetObjectPose', simRosSetObjectPose)

    rospy.wait_for_service('/vrep/simRosGetObjectChild')
    self.get_object_child = rospy.ServiceProxy('/vrep/simRosGetObjectChild', simRosGetObjectChild)

    rospy.wait_for_service('/vrep/simRosGetObjectGroupData')
    self.get_object_group_data = rospy.ServiceProxy('/vrep/simRosGetObjectGroupData', simRosGetObjectGroupData)

    rospy.wait_for_service('/vrep/simRosGetObjects')
    self.get_objects = rospy.ServiceProxy('/vrep/simRosGetObjects', simRosGetObjects)

    if rospy.has_param('stop_condition'):
      self.stop_condition = rospy.get_param('stop_condition')
    else:
      self.stop_condition = None

    if rospy.has_param('scene_file'):
      scene_file = directory_search(rospy.get_param('scene_file'))
      load_scene(scene_file)
      print 'Loaded scene file %s' % scene_file

    robot_count = 0
    self.robots = []
    
    # Add robots specified in parameter file
    if rospy.has_param('robots'):
      for robot in rospy.get_param('robots'):
        robot_namespace = '/robot_%d/' % robot_count
        model_file = directory_search(robot['model_file'])
        load_response = load_model(model_file)
        robot_handle = load_response.baseHandle
        
        print 'Loaded model %s for %s, handle %s' % (model_file, robot_namespace, robot_handle)
        
        robot_record = {
          'handle': robot_handle,
          'namespace': robot_namespace,
          'count': robot_count,
          'diff_drive_params': [DEFAULT_ROBOT_D, DEFAULT_ROBOT_R],
          'wheel_handles': {'left':None, 'right':None},
          'scanner_handles': []
        }

        if 'initial_pose' in robot.keys():
          pose = robot['initial_pose']
          set_object_pose(
            robot_handle, -1, Pose(Point(**pose['position']),Quaternion(**pose['orientation']))
          )
            
        if 'diff_drive_params' in robot.keys():
          robot_record['diff_drive_params'] = robot['diff_drive_params']

        self.robots.append(robot_record)
        
        robot_count += 1
    
    # Gather all object data after robots have been added
    self.get_all_object_data()
    print self.object_data

    # Start simulation before enabling publishers and subscribers
    self.start_simulation()
    self.stopped = False
    
    print 'vrep simulation started'

    # Only one publisher for all controlled joint states
    enable_subscriber('/vrep/joints', 1, SIMROS_STRMCMD_SET_JOINT_STATE, -1, -1, '')
    self.joint_pub = rospy.Publisher('/vrep/joints', JointSetStateData, queue_size=1)

    # Set up publishers and subscribers
    for robot in self.robots:
      enable_publisher(
        robot['namespace'] + 'pose', 1, SIMROS_STRMCMD_GET_OBJECT_POSE, robot['handle'], -1, ''
      )
      
      rospy.Subscriber(
        robot['namespace'] + 'cmd_vel', Twist, self.curried_twist_callback(robot['count'])
      )
      
      robot_data = self.get_object_child_data(robot['handle'])
      for child_data in robot_data:
        if child_data['type'] is JOINT_OBJECT_TYPE:
          if child_data['name'][:4] == 'left':
            robot['wheel_handles']['left'] = child_data['handle']
          if child_data['name'][:5] == 'right':
            robot['wheel_handles']['right'] = child_data['handle']
        
        elif child_data['type'] is FORCE_SENSOR_OBJECT_TYPE and child_data['name'][:3] == 'imu':
          topic_name = robot['namespace'] + 'imu'
          print 'Enabling IMU publisher %s %d on %s'% (
            child_data['name'], child_data['handle'], topic_name
          )
          enable_publisher(
            topic_name, 1, SIMROS_STRMCMD_READ_FORCE_SENSOR, child_data['handle'], -1, ''  
          )

        elif child_data['type'] is VISION_SENSOR_OBJECT_TYPE:
          base_name = child_data['name'].split('#')[0]
          image_topic = robot['namespace'] + base_name + '/image'
          info_topic = robot['namespace'] + base_name + '/camera_info'
          print 'Enabling camera %s %d on %s %s' % (
            base_name, child_data['handle'], image_topic, info_topic
          )
          enable_publisher(
            image_topic, 1, SIMROS_STRMCMD_GET_VISION_SENSOR_IMAGE, child_data['handle'], -1, ''
          )
          enable_publisher(
            info_topic, 1, SIMROS_STRMCMD_GET_VISION_SENSOR_INFO, child_data['handle'], -1, ''
          )
      
    print 'vrep simulation initialized'
    
  def get_all_object_data(self):
    name_data = self.get_object_group_data(SIM_APPOBJ_OBJECT_TYPE, OBJECT_DATA_NAME)
    handles = name_data.handles
    names = name_data.strings
    self.object_data = {
      handle:{'handle':handle, 'name':name} for handle, name in zip(handles,names)
    }
    
    type_data = self.get_object_group_data(SIM_APPOBJ_OBJECT_TYPE, OBJECT_DATA_TYPE)
    for handle, type_int in zip(type_data.handles, type_data.intData):
      self.object_data[handle]['type'] = type_int 

    parent_data = self.get_object_group_data(SIM_APPOBJ_OBJECT_TYPE, OBJECT_DATA_PARENT)
    for handle, parent in zip(parent_data.handles, parent_data.intData):
      self.object_data[handle]['parent'] = parent
  
  def get_object_child_data(self, object_handle):
    
    child_data = []
    child_index = 0
    child_handle = self.get_object_child(object_handle, child_index).childHandle
    while child_handle != -1:
      if child_handle in self.object_data.keys():
        child_data.append(self.object_data[child_handle])
        child_data.extend(self.get_object_child_data(child_handle))
      child_index += 1
      child_handle = self.get_object_child(object_handle, child_index).childHandle

    return child_data

  def curried_twist_callback(self, robot_idx):
    return lambda msg: self.twist_callback(msg, robot_idx)

  def twist_callback(self, msg, robot_idx):
    this_robot = self.robots[robot_idx]
    if this_robot['wheel_handles']['left'] is not None:
      cmd_linear = msg.linear.x
      cmd_angular = msg.angular.z
      robot_d, robot_r = this_robot['diff_drive_params']
      cmd_left = (cmd_linear - (robot_d * cmd_angular)/2.0)/robot_r
      cmd_right = (cmd_linear + (robot_d * cmd_angular)/2.0)/robot_r
      jssd = JointSetStateData()
      jssd.handles.data = [
        this_robot['wheel_handles']['left'],
        this_robot['wheel_handles']['right']
      ]
      jssd.setModes.data = [JOINT_MODE_CONTROL_VELOCITY, JOINT_MODE_CONTROL_VELOCITY]
      jssd.values.data = [cmd_left, cmd_right]
      self.joint_pub.publish(jssd)

  def keyboard_callback(self, msg):
    if msg.code == ord('q'):
      print 'Stopping vrep simulation'
      self.stop_simulation()
      self.stopped = True
    elif msg.code == ord('p'):
      print 'Pausing vrep simulation'
      self.pause_simulation()
    elif msg.code == ord('s'):
      print 'Starting vrep simulation'
      self.start_simulation()

  def run(self):
    rate = rospy.Rate(1)
    while not (self.stop_condition == 'manual' and self.stopped) and not rospy.is_shutdown():
      rate.sleep()

if __name__ == '__main__':
  vm = VrepMonitor()
  vm.run()

