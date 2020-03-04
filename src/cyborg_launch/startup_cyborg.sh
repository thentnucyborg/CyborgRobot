#! /bin/bash

source ~/.bashrc
export ROS_IP=localhost
export ROS_HOSTNAME=localhost
export ROS_MASTER_URI=http://localhost:11311
source /opt/ros/kinetic/setup.bash

cd ~/catkin_ws/
catkin_make
source ~/catkin_ws/devel/setup.bash 

cd ~/catkin_ws/src/cyborg_launch/

## run this command for testing on computers not on the cyborg
gnome-terminal -e 'roslaunch cyborg.launch'
	
## run this command and comment out the other for running on the cyborg
# xfce4-terminal -e 'roslaunch cyborg.launch' --hold