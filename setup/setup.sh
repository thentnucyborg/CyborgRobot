#!/bin/bash

echo "Setup script running..."

# This is not complete, and may be missing some libraries or install commands 
# All commands should also be updated to include versions. 


## Exit if failure
if [[ $EUID -ne 0 ]]; then
   echo "This script must be run as root"
   exit 1
fi

echo "---------- General installs and setup ----------"
echo "to continue press enter"
read 

sudo apt-get update
sudo apt-get install git
sudo apt install python-pip
sudo apt install python2.7 python-pip
#sudo apt install python3-pip 	#may be unnecessary 


echo "---------- Install ROS ----------"
echo "to continue press enter"
read 

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
read -p "Available packages (above). Take a note if some are needed. Press ENTER to continue."
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


echo "---------- Clone Cyborg repo from Git ----------"
echo "to continue press enter"
read 

## Clone Cyborg Repo from git, and place it in the right directory
cd ~/catkin_ws
git clone https://github.com/thentnucyborg/CyborgRobot.git 	#clones the master branch

echo "Use the master branch? Please type the branch name or 'yes' if you wish to use the master branch."
read branchname
if [ "$branchname" == "yes" ]
then
	echo "Using the master branch"
else 
	git checkout $branchname
fi

mv ~/catkin_ws/CyborgRobot/* ~/catkin_ws/	#move all files and folders to the workspace
mv ~/catkin_ws/CyborgRobot/.* ~/catkin_ws/	#move all hidden files and folders to the workspace. Ingore the message saying . and .. cannot be moved
echo "Please ignore the message saying . and .. cannot be moved"
rm -rf CyborgRobot 	#delete the now empty folder



echo "---------- Install Arnl and ARIA .debs ----------"
echo "to continue press enter"
read 

## Open a different folder and download Arnl.tar.gz, libaria 2.9.4, libarnl 1.9.2a, arnl-base_1.9.2 and mobilesim_0.9.8 from BOX.	
# Unzip Arnl.tar.gz	
sudo dpkg -i arnl-base_1.9.2+ubuntu16_amd64.deb	
sudo dpkg -i libarnl_1.9.2a+ubuntu16_amd64.deb	
sudo dpkg -i libaria_2.9.4+ubuntu16_amd64.deb	
sudo dpkg -i mobilesim_0.9.8+ubuntu16_amd64.deb	
sudo cp -r ./Arnl /usr/local/


echo "---------- Installs for Navigation ----------"
echo "to continue press enter"
read 

## Installs for Navigation stack (may be more)
cd  ~/catkin_ws/src
sudo apt-get install ros-kinetic-navigation
sudo apt-get install ros-kinetic-tf2-sensor-msgs
sudo apt-get install libsdl-dev
sudo apt-get install libsdl-image1.2-dev
sudo apt-get install libbullet-dev
git clone https://github.com/ros-visualization/rviz.git -b kinetic-devel
git clone https://github.com/ros-planning/navigation.git -b kinetic-devel
# Install move-base 
sudo apt install ros-kinetic-move-base


echo "---------- Install SMACH ----------"
echo "to continue press enter"
read 

## Install SMACH
sudo apt-get install ros-kinetic-executive-smach
sudo apt-get install ros-kinetic-executive-smach-visualization

sudo apt-get install python-pyqt5
sudo apt-get install python-qt-binding


echo "---------- Install Aduio ----------"
echo "to continue press enter"
read 

## Install for Audio node
pip2 install -Iv pyttsx3==2.7	#-I ignores installed packages, -v prints/verbose
sudo apt-get install vlc
pip2 install python-vlc==3.0.7110


echo "---------- Install Command node ----------"
echo "to continue press enter"
read 

## Install for Command node
pip2 install npyscreen


echo "---------- Install Controller ----------"
echo "to continue press enter"
read 

## Install for Controller node
sudo apt install graphviz-dev	
pip2 install pygraphviz 	
#Alternatively, run: sudo apt-get install python-pygraphviz	


echo "---------- Install LED Dome node ----------"
echo "to continue press enter"
read 

## Install for Led Dome node
pip2 install colour==0.1.5
pip2 install pandas==0.24.2
pip2 install pyserial==3.0.1
pip2 install numpy==1.16.5
pip2 install pyopengl
pip2 install pyopengl-accelerate
pip2 install pytz


echo "---------- Install Behaviour Trees ----------"
echo "to continue press enter"
read 

## Behavior Trees
pip2 install networkx==2.2


echo "---------- Setup UDEV Rules ----------"
echo "to continue press enter"
read 

## Set up UDEV rules
sudo cp ~/catkin_ws/setup/90_cyborg_usb_rules.rules /etc/udev/rules.d/
sudo udevadm controll --reload
sudo udevadm trigger


echo "---------- other ----------"
echo "to continue press enter"
read 

## Other
sudo apt-get install sqlitebrowser	#tool for editing databases

# Make  python  and  bash  scripts  executable
find ~/catkin_ws/src/ -name '*.py' -exec  chmod +x {} \;
find ~/catkin_ws/src/ -name '*.sh' -exec  chmod +x {} \;


echo "---------- Finish setting up catkin_ws ----------"
echo "to continue press enter"
read 

# Finish setting up the workspace	
catkin_make	
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc # Opening new terminal runs source command, so we dont have to source workspace each time.	
source devel/setup.bash


## Base requirements
sudo usermod -a -G dialout $USER 	#add user to dialout group
sudo apt autoremove



# Relogin is required for last cmd to take effect
echo "You must logout and back in for userpivilages to take effect..."
echo " "

echo "There are still some things to install:"
echo " - If changes to the code on the led-controller are needed (the NodeMCU ESP32), follow the install instructions in cyborg_ros_led_dome/README.md"
echo " - Some udev rules are needed to interface with the led-dome, mode-selector-box and zed-camera. Follow the instructions on the github wiki under 'Cyborg udev Rules'"
echo " - Download 'mobilesim_0.9.8+ubuntu16_amd64.deb' from Box (in Robotics/MobileRobots legacy software/ and install using Ubuntu Software"
echo " - Repeat the step above for 'libaria_2.9.4+ubuntu16_amd64.deb'"

echo "Setup script ended..."
