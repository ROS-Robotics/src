#!/bin/sh
export ROS_HOSTNAME="192.168.0.101"
export ROS_IP="192.168.0.101"
. ~/catkin_ws/devel/setup.sh
exec "$@"
