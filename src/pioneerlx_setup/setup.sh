#!/bin/bash
echo "Setup script running..."

# Exit if failure
set -e

sudo apt-get update
sudo apt-get upgrade
sudo apt-get install git

# ROS
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt-get update
sudo apt-get install ros-kinetic-desktop-full
sudo rosdep init
rosdep update
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
echo 'source ~/catkin_ws/devel/setup.bash' >> ~/.bashrc 
source ~/.bashrc

# Setup ROS
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
sudo apt install catkin
catkin_init_workspace
cd ~/catkin_ws/
catkin_make
source ~/catkin_ws/devel/setup.bash
echo $ROS_PACKAGE_PATH

# Install SMACH
sudo apt-get install ros-kinetic-smach

# Other
sudo apt-get install python3-pip
pip3 install npyscreen
sudo apt-get install python3-pygraphviz
sudo apt-get install python-pygraphviz
sudo apt-get install sqlitebrowser
pip install --upgrade pip

# Speech Output requirements:
sudo apt-get install python-requests
sudo apt install python-pip # This is pip2 for Python 2.7
sudo apt-get install sox #might be redundant without speech recognition
pip2 install PyTTSX3

# LED Dome Module requirements:
pip2 install pyopengl 
pip2 install pyopengl-accelerate
pip2 install numpy
pip2 install pandas
pip2 install colour
pip2 install pyserial


# Music Module requirements
sudo apt-get install vlc
pip2 install python-vlc

# Command tool requirements
pip2 install npyscreen

# Put ROSARNL in ~/catkin_ws/src
sudo apt-get install ros-kinetic-move-base
cd ~/catkin_ws/src
git clone https://github.com/MobileRobots/ros-arnl
cd ~/catkin_ws
catkin_make
source ~/catkin_ws/devel/setup.bash

# Put Cyborg modules in ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/thentnucyborg/cyborg_ros_controller
git clone https://github.com/thentnucyborg/LED-dome
git clone https://github.com/thentnucyborg/cyborg_ros_navigation
git clone https://github.com/thentnucyborg/executive_smach_visualization
git clone https://github.com/thentnucyborg/cyborg_ros_command
# audio
# behavior
# event scheduler
# primary states




# Base requirements
sudo usermod -a -G dialout $USER
# Relogin is required for last cmd to take effect
echo "You must logout and back in for userpivilages to take effect..."
#gnome-session-quit
# Alternativly: rebbot

echo "Setup script ended..."

