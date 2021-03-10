#!/bin/zsh
export ROS_MASTER_URI=http://10.42.0.2:11311
export ROS_IP=10.42.0.1
unset ROS_HOSTNAME
#source directory first
rosrun tools enable_robot.py -e
rosrun baxter_interface joint_trajectory_action_server.py
echo "Done!"