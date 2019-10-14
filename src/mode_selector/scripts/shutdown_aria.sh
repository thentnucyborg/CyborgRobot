#!/bin/bash
source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash

rosnode kill -a
killall -SIGINT demo
#killall -SIGINT rosarnl_node
#killall -SIGINT cyborg_controller
#killall -SIGINT roscore
#pkill xfce4-terminal

exit

exec $SHELL
