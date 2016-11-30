#!/usr/bin/python

import os
import sys

VREP_ROOT = os.environ['VREP_ROOT']
sys.path.append(VREP_ROOT + '/programming/remoteApiBindings/python/python')
os.chdir(VREP_ROOT + '/programming/remoteApiBindings/lib/lib/64Bit')

import vrep
import rospy
from tf.transformations import *

import fnmatch
from collections import OrderedDict

GROUP_DATA_NAMES = 0


def directory_search(filename, directory=VREP_ROOT):
  for root, dirs, files in os.walk(directory):
    for basename in files:
      if fnmatch.fnmatch(basename, filename):
        found_filename = os.path.join(root, basename)
        return found_filename

class VrepInterface():
  def __init__(self):
    # just in case, close all opened connections
    vrep.simxFinish(-1)

    # Connect to V-REP
    self.client_id = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 1)

    self.stop_sim()

  def load_scene(self):
    if self.client_id != -1:
      # Close last open scene
      vrep.simxCloseScene(self.client_id, vrep.simx_opmode_blocking)

      # Load scene specified in parameter file
      if rospy.has_param('scene_file'):
        scene_file = directory_search(rospy.get_param('scene_file'))
        if scene_file is not None:
          vrep.simxLoadScene(self.client_id, scene_file, 0,
            vrep.simx_opmode_blocking)

  def load_models(self):
    if self.client_id == -1:
      return
    
    configs = OrderedDict()
    
    # Add scene objects
    if rospy.has_param('objects'):
      for obj in rospy.get_param('objects'):
        if 'stl_filename' in obj.keys():
          object_handle, config = self.add_model_get_config(ros_topic=obj['stl_filename'], **obj)
        else:
          object_handle, config = self.add_model_get_config(**obj)
        configs[object_handle] = config
        print configs
  
    # Add robots and child objects specified in parameter file creating appropriate
    # namespaces
    if rospy.has_param('robots'):
      robot_count = 0
      

      for robot in rospy.get_param('robots'):
        robot_namespace = '/robot_%02d' % robot_count
        
        robot_handle, config = self.add_model_get_config(ros_topic=robot_namespace, **robot)
        configs[robot_handle] = config

        if 'children' in robot.keys():
          child_counts = {}
          
          for child in robot['children']:
            topic = child['topic']
            if topic not in child_counts.keys():
              child_counts[topic] = 0
            else:
              child_counts[topic] += 1
            full_topic = '%s/%s_%d' % (robot_namespace, topic, child_counts[topic])
            
            child_handle, config = self.add_model_get_config(
              ros_topic=full_topic, parent_handle=robot_handle, **child)

            configs[child_handle] = config

        robot_count += 1

    _, handles, _, _, string_data = vrep.simxGetObjectGroupData(
      self.client_id, vrep.sim_appobj_object_type, GROUP_DATA_NAMES,
      vrep.simx_opmode_blocking)
    
    names = dict(zip(handles, string_data))
    
    print names

    for handle,config in configs.items()[-1::-1]:
      print vrep.simxCallScriptFunction(self.client_id, names[handle], 
        vrep.sim_scripttype_customizationscript, 'config', *config, 
          operationMode=vrep.simx_opmode_blocking)

  def add_model_get_config(self, model_filename, ros_topic,
    position=3*[0.0], orientation=3*[0.0]+[1.0], parent_handle=-1, **kwargs):

    model_handle = None
    model_file = directory_search(model_filename)
    
    if model_file is not None:
      _, model_handle = vrep.simxLoadModel(
        self.client_id, model_file, 0, vrep.simx_opmode_blocking)

    config = [
      [parent_handle],
      position + orientation,
      [ros_topic],
      ''
    ]

    if 'marker_positions' in kwargs.keys():
      mp = kwargs['marker_positions']
      config[1] += [mp[i][k] for i in range(len(mp)) for k in ['h','x','y','z']]

    return model_handle, config
  
  def add_model(self, model_filename, ros_topic, 
    position=3*[0.0], orientation=3*[0.0]+[1.0], parent_handle=-1, **kwargs):

    vrep.simxSetStringSignal(self.client_id, 'ros_topic', ros_topic, 
      vrep.simx_opmode_oneshot)

    model_handle = None
    model_file = directory_search(model_filename)
    
    if model_file is not None:
      _, model_handle = vrep.simxLoadModel(
        self.client_id, model_file, 0, vrep.simx_opmode_blocking)
    
    if parent_handle != -1:
      vrep.simxSetObjectParent(self.client_id, model_handle, parent_handle, False, 
        vrep.simx_opmode_oneshot)
    
    # Set the center of this model to have given pose relative to parent
    center_handle = self.get_child_by_name(model_handle, 'center', vrep.sim_object_dummy_type)
    model_to_center_tf = self.get_object_transform(center_handle, model_handle)
    center_tf = translation_matrix(position).dot(quaternion_matrix(orientation))
    model_tf = center_tf.dot(numpy.linalg.inv(model_to_center_tf))
    self.set_object_transform(model_handle, model_tf, parent_handle)

    return model_handle, center_handle
  
  def get_object_transform(self, object_handle, parent_handle=-1):
    if self.client_id != -1:
      _,position = vrep.simxGetObjectPosition(
        self.client_id, object_handle, parent_handle, vrep.simx_opmode_blocking)
      _,orientation = vrep.simxGetObjectOrientation(
        self.client_id, object_handle, parent_handle, vrep.simx_opmode_blocking)
      transform = translation_matrix(position)
      transform = transform.dot(euler_matrix(*orientation))
      return transform

  def set_object_transform(self, object_handle, transform, parent_handle=-1):
    if self.client_id != -1:
      position = list(transform[:3,3])
      orientation = euler_from_matrix(transform)
      
      vrep.simxSetObjectPosition(self.client_id, object_handle, parent_handle,
        position, vrep.simx_opmode_oneshot)
    
      vrep.simxSetObjectOrientation(self.client_id, object_handle, parent_handle,
        orientation, vrep.simx_opmode_oneshot)

  def get_object_name(self, handle, object_type = vrep.sim_appobj_object_type):
    name = ''
    if self.client_id != -1:
      _, handles, _, _, string_data = vrep.simxGetObjectGroupData(
        self.client_id, object_type, GROUP_DATA_NAMES, vrep.simx_opmode_blocking)
      if handle in handles:
        name = string_data[handles.index(handle)]
    return name

  def get_child_by_name(self, parent, child_name, object_type = vrep.sim_appobj_object_type):
    child_handle = -2
    child_index = 0
    if self.client_id != -1:
      while child_handle != -1:
        _, child_handle = vrep.simxGetObjectChild(
          self.client_id, parent, child_index, vrep.simx_opmode_blocking)
        if self.get_object_name(child_handle, object_type).startswith(child_name):
          return child_handle
        child_index += 1
    return child_handle

  def start_sim(self):
    if self.client_id != -1:
      vrep.simxStartSimulation(self.client_id, vrep.simx_opmode_oneshot)
  
  def stop_sim(self):
    if self.client_id != -1:
      vrep.simxStopSimulation(self.client_id, vrep.simx_opmode_oneshot)
  
  def pause_sim(self):
    if self.client_id != -1:
      vrep.simxPauseSimulation(self.client_id, vrep.simx_opmode_oneshot)
  
  def __del__(self):
    # Now close the connection to V-REP:
    if self.client_id != -1:
      vrep.simxFinish(self.client_id)

if __name__ == '__main__':
  vi = VrepInterface()
  vi.load_scene()
  vi.load_models()
