# bml\_vrep

Code for running vrep simulations of Biomimetic Millisystems Lab robots with ROS

Setup for v-rep 3.3.2
=====================

 * Install v-rep 3.3.2 rev3
 * Install [catkin tools](http://catkin-tools.readthedocs.io/en/latest/installing.html)
 * export environment variable VREP\_ROOT with the full path to the vrep dir, and `export PATH=$PATH:VREP\ROOT` to your bashrc
 * replace VREP\_ROOT\programming/ros\_packages/v\_repExtRosInterface with repo at github.com/abuchan/v\_repExtRosInterface.git
 * make a catkin workspace
 * have this repo and a symbolic link to the vrep/programming/ros\_packages folder in the src directory ofthe catkin workspace
 * run `catkin build` (not catkin\_make) from workspace root
 * symbolic link devel/lib/libv\_repExtRosInterface.so in v-rep root
 * In vrep\_root/programming/v\_repExtRemoteApi add `-DNON_MATLAB_PARSING -DMAX_EXT_API_CONNECTIONS=255` to makefile `CFLAGS` variable, run `make`, link libv\_repExtRemoteApi.so in bml\_vrep/src as remoteApi.so
 * symbolic link remoteApi.so in bml\_vrep/src
 * roslaunch bml\_vrep 

Setup for v-rep 3.2.2
=====================

 * Install v-rep 3.2.2
 * Install packages ros-indigo-ar-track-alvar, ros-indigo-keyboard, ros-indigo-teleop-twist-keyboard
 * Add environment variable VREP\_ROOT\_DIR to bashrc that points to v-rep isntall directory
 * Add VREP\_ROOT\_DIR to path
 * Put ar\_tag.ttt in VREP\_ROOT\_DIR/scenes and bubble\_rob\_laser.ttm in VREP\_ROOT\_DIR/models/robots/mobile
 * Create ROS workspace and add this repo, as well as a symbolic link to the full path to VREP\_ROOT\_DIR/programming/ros\_packages to the worspace src directory
 * Create a symbolic link to VREP\_ROOT\_DIR/programming/ros\_packages/ros\_bubble\_rob/include/v\_repConst.h in this repo's include directory
 * Create a symbolic link to VREP\_ROOT\_DIR/vrep.sh in this repo's src directory

Running Demo
============

 * roslaunch bml\_vrep demo.launch
 * (in a separate terminal for each robot, replace n with robot number) roslaunch bml\_vrep keyboard\_input.launch robot=robot\_n. You'll probably want to lower the speed to prevent the robots from falling over.
 * To quit, find the keyboard input window (should have a keyboard icon in the taskbar) and enter q in it before ctrl-c killing everything else
