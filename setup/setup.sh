#!/bin/bash

echo "Setup script running..."
echo "Needs to be run with sudo"

# This is not complete, and may be missing some libraries or install commands 
# All commands should also be updated to include versions. 


## Exit if failure
if [[ $EUID -ne 0 ]]; then
   echo "This script must be run as root"
   exit 1
fi

sudo apt-get update
sudo apt-get install git
sudo apt install python-pip
sudo apt install pyhton2.7 python-pip
#sudo apt install python3-pip 	#may be unnecessary 


## ROS
# Setup your computer to accept software from packages.ros.org:
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
# Set up your keys
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

sudo apt-get update
# Full ROS installation 
sudo apt install ros-kinetic-desktop-full
# Find avaliable packages
apt-cache search ros-kinetic
read -p "Avaliable packages (above). Take a note if some are needed. Press ENTER to continue."
# Initialise rosdep
sudo rosdep init
rosdep update
# Setup the environment
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
# Dependencies for building packages
sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential
# Create a workspace
mkdir -p ~/catkin_ws/


## Clone Cyborg Repo from git, and place it in the right directory
cd ~/catkin_ws
git clone https://github.com/thentnucyborg/CyborgRobot.git 	#clones the master branch
mv ~/catkin_ws/CyborgRobot/* ~/catkin_ws/	#move all files and folders to the workspace
mv ~/catkin_ws/CyborgRobot/.* ~/catkin_ws/	#move all hidden files and folders to the workspace. Ingore the message saying . and .. cannot be moved
rm -rf CyborgRobot 	#delete the now empty folder
# Finish setting up the workspace
catkin_make
source devel/setup.bash


## Installs for Navigation stack (may be more)
sudo apt-get install ros-kinetic-navigation
sudo apt-get install ros-kinetic-tf2-sensor-msgs
sudo apt-get install libsdl-dev
sudo apt-get install libsdl-image1.2-dev
sudo apt-get install libbullet-dev
git clone https://github.com/ros-visualization/rviz.git -b kinetic-devel
git clone https://github.com/ros-planning/navigation.git -b kinetic-devel
# Install move-base 
sudo apt install ros-kinetic-move-base


## Install SMACH
sudo apt-get install ros-kinetic-executive-smach
sudo apt-get install ros-kinetic-executive-smach-visualization

sudo apt-get install python-pyqt5
sudo apt-get install python-qt-binding



## Install for Audio node
pip2 install -Iv pyttsx3==2.7	#-I ignores installed packages, -v prints/verbose
sudo apt-get install vlc
pip2 install python-vlc==3.0.7110


## Install for Command node
pip2 instll npyscreen


## Install for Controller node
pip2 install pygraphviz


## Install for Led Dome node
pip2 install colour==0.1.5
pip2 install pandas==0.24.2
pip2 install pyserial==3.0.1
pip2 install numpy==1.16.5
pip2 install pyopengl
pip2 install pyopengl-accelerate
pip2 install pytz


## Other
sudo apt-get install sqlitebrowser	#tool for editing databases

# Make  python  and  bash  scripts  executable
find ~/catkin_ws/src/ -name '*.py' -exec  chmod +x {} \;
find ~/catkin_ws/src/ -name '*.sh' -exec  chmod +x {} \;


## Base requirements
sudo usermod -a -G dialout $USER 	#add user to dialout group

#sudo apt autoremove


# Relogin is required for last cmd to take effect
echo "You must logout and back in for userpivilages to take effect..."
echo " "

echo "There are still some things to install:"
echo " - If changes to the code on the led-controller are needed (the NodeMCU ESP32), follow the install instructions in cyborg_ros_led_dome/README.md"
echo " - Some udev rules are needed to interface with the led-dome, mode-selector-box and zed-camera. Follow the instructions on the github wiki under 'Cyborg udev Rules'"
echo " - Download 'mobilesim_0.9.8+ubuntu16_amd64.deb' from Box (in Robotics/MobileRobots legacy software/ and install using Ubuntu Software"
echo " - Repeat the step above for 'libaria_2.9.4+ubuntu16_amd64.deb'"

echo "Setup script ended..."



# ????
# # Guide:
# 1. Download Ubuntu 16.04 disk image and create a bootable disk using the image and a USB flash drive.
# 2. Startup the system, if desktop environment is installed skip the next step.
# 3. Get a desktop environment, we have been using Unity. To use unity, open command prompt for Ubuntu and do:
# 	3.1.  sudo apt -get  update
# 	3.2.  sudo apt -get  install  ubuntu -desktop
# # (If you’re using  virtual  machine , follow  these  steps to be
# # able to  remote  desktop  or  follow  this  guide:
# # https :// gist.github.com/spacecat/b71918d49fbd520d18647b0ec3513525):
# # In  Linux  Terminal:
# A. cd
# B. mkdir  Downloads
# C. cd  Downloads
# D. wget "http ://www.c-nergy.be/downloads/install -xrdp -1.9.1. zip"
# E. sudo apt -get  install  unzip
# F unzip  install -xrdp -1.9.1. zip
# G. nano  install -xrdp -1.9.1. sh
# # (** REMOVE ** Step1  Ubuntu  version  check , *from if to fi*)
# H. chmod +x ~/ Downloads/install -xrdp -1.9.1/ install -xrdp -1.9.1. sh
# I. sudo ./install -xrdp -1.9.1. sh25J. # Restart  VM  instance.
# K. # Use  program "remote  desktop  connection" on  windows  and  put in
# # your VM’s public  ip.
