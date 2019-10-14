#!/bin/bash
source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash
roslaunch cyborg_controller controller.launch

exec $SHELL