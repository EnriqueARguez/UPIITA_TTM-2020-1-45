#!/usr/bin/env bash

cd ~/catkin_ws/src
catkin_create_pkg ... std_msgs rospy roscpp

cd ~/catkin_ws
catkin_make

. ~/catkin_ws/devel/setup.bash

roscd ...

mkdir scripts
cd scripts
