#!/bin/bash
echo "=================================="
echo "Starting Gazebo Reflex simulation."
echo "=================================="
. /opt/ros/noetic/setup.sh
. ~/overlay/work/catkin_ws/devel/setup.sh
roslaunch description reflex.launch gui:=false