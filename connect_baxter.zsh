#!/usr/bin/env zsh
CATKIN_SHELL=zsh

export ROS_MASTER_URI=http://10.42.0.2:11311
export ROS_IP=10.42.0.1
unset ROS_HOSTNAME

printenv | grep ROS_MASTER_URI
printenv | grep ROS_PACKAGE_PATH

echo "Done!"