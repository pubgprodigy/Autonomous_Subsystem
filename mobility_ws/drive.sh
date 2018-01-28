#!/bin/bash
source devel/setup.bash
export ROS_IP=10.42.0.88
export ROS_MASTER_URI=http://10.42.0.1:11311
roslaunch rover_mobility steer_run.launch
