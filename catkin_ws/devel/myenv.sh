#!/bin/sh
export ROS_MASTER_URI=http://arm:11311 
export ROS_HOSTNAME=arm
. /opt/ros/kinetic/setup.sh
. /home/ubuntu/catkin_ws/devel/setup.sh
exec "$@"
