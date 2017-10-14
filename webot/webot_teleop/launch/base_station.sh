#!/bin/bash

echo robot_ip=$1 pc_ip=$2
export ROS_HOSTNAME=$2
export ROS_MASTER_URI=http://$1:11311
export ROSLAUNCH_SSH_UNKNOWN=1
roslaunch ric_base_station remote_ric_robot.launch robot_ip:=$1 pc_ip:=$2 user:=$3 pass:=$4 launch_file:=$5
echo 'Done'
