#!/bin/bash

echo "Setup script running..."
echo "WARN - Specific versions of packages are needed in this project and when installing warnings about deprecated or old software may show up. This is expected."
echo "This script was tested and worked in may 2020, with new updates to libraries or changes to any of the nodes this may not be the case any longer."
read 

# All commands should also be updated to include versions. 

echo "---------- General installs and setup ----------"
echo "to continue press enter"
read 

sudo apt-get update
sudo apt-get install git
sudo apt install python2.7 
sudo apt install python-pip
pip install --upgrade pip==20.0.2


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

cd ./CyborgRobot
echo "Use the master branch? Please type the branch name or 'yes' if you wish to use the master branch."
read branchname
if [ "$branchname" == "yes" ]
then
	echo "Using the master branch"
else 
	echo "Using the "$branchname" branch"
	git checkout $branchname
fi

mv ~/catkin_ws/CyborgRobot/* ~/catkin_ws/	#move all files and folders to the workspace
mv ~/catkin_ws/CyborgRobot/.* ~/catkin_ws/	#move all hidden files and folders to the workspace. Ingore the message saying . and .. cannot be moved
echo "Please ignore the message saying . and .. cannot be moved"
rm -rf ~/catkin_ws/CyborgRobot 	#delete the now empty folder



echo "---------- Install MobileSim & ARIA .debs ----------"
echo "to continue press enter"
read 

cd ~/catkin_ws/setup/installs
sudo dpkg -i libaria_2.9.4+ubuntu16_amd64.deb	
sudo dpkg -i mobilesim_0.9.8+ubuntu16_amd64.deb	

	
echo "---------- Install GUI ----------"
echo "to continue press enter"
read

pip2 install pymongo==3.10.1
pip2 install pymongo[srv]
sudo apt-get install ros-kinetic-rosauth
sudo apt-get install ros-kinetic-rosbridge-server
sudo apt-get install ros-kinetic-rospy-message-converter
sudo apt-get install rosbash


echo "---------- Installs for New Navigation ----------"
echo "to continue press enter"
read 

## Installs for Navigation stack
cd  ~/catkin_ws/src
sudo apt-get install ros-kinetic-navigation
sudo apt-get install ros-kinetic-tf2-sensor-msgs
sudo apt-get install ros-kinetic-tf2-geometry-msgs
sudo apt-get install ros-kinetic-cmake-modules
sudo apt-get install ros-kinetic-tf2-kdl
sudo apt-get install ros-kinetic-kdl-parser
sudo apt-get install ros-kinetic-move-base
sudo apt-get install ros-kinetic-image-transport
sudo apt-get install ros-kinetic-interactive-markers
sudo apt-get install ros-kinetic-python-qt-binding
sudo apt-get install ros-kinetic-resource-retriever
sudo apt-get install libogre-1.9-dev
sudo apt-get install libsdl-dev
sudo apt-get install libsdl-image1.2-dev
sudo apt-get install libbullet-dev
sudo apt-get install libassimp-dev assimp-utils

sudo apt-get install openni2-doc && openni2-utils && openni-doc && openni-utils
sudo apt-get install libopenni0 libopenni-sensor-pointclouds0
sudo apt-get install libopenni2-0 
sudo apt-get install libopenni-sensor-pointclouds-dev
sudo apt-get install libopenni2-dev
sudo apt-get install libopenni-dev
sudo ln -s /usr/lib/python2.7/dist-packages/vtk/libvtkRenderingPythonTkWidgets.x86_64-linux-gnu.so /usr/lib/x86_64-linux-gnu/libvtkRenderingPythonTkWidgets.so
sudo update-alternatives --install /usr/bin/vtk vtk /usr/bin/vtk6 10

git clone https://github.com/ros-visualization/rviz.git -b kinetic-devel
git clone https://github.com/ros-planning/navigation.git -b kinetic-devel


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
pip2 install numpy==1.16.6
pip2 install pandas==0.24.2
pip2 install pyserial==3.0.1
pip2 install pyopengl
pip2 install pyopengl-accelerate
pip2 install pytz


echo "---------- Setup UDEV Rules ----------"
echo "to continue press enter"
read 

## Set up UDEV rules
sudo cp ~/catkin_ws/setup/90_cyborg_usb_rules.rules /etc/udev/rules.d/
sudo udevadm control --reload
sudo udevadm trigger


echo "---------- other ----------"
echo "to continue press enter"
read 

## Other
sudo apt-get install sqlitebrowser	#tool for editing databases
pip install -r requirements.txt

# Make  python  and  bash  scripts  executable
find ~/catkin_ws/src/ -name '*.py' -exec  chmod +x {} \;
find ~/catkin_ws/src/ -name '*.sh' -exec  chmod +x {} \;


echo "---------- Finish setting up catkin_ws ----------"
echo "to continue press enter"
read 

# Finish setting up the workspace	
source ~/.bashrc
source /opt/ros/kinetic/setup.bash
cd ~/catkin_ws
catkin_make	
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc # Opening new terminal runs source command, so we dont have to source workspace each time.	
source devel/setup.bash


## Base requirements
sudo usermod -a -G dialout $USER 	#add user to dialout group
sudo apt autoremove



echo "Setup script ended..."
echo "---------------------"

# Relogin is required for last cmd to take effect
echo "You must logout and back in for userpivileges to take effect..."
echo "You might want to change a value in /usr/local/Aria/params/pioneer-lx.p to flip the sensor output on the Cyborg the right way up. See cyborg_navigation/README.md for how to."
echo "If changes to the code on the led-controller are needed (the NodeMCU ESP32), follow the install instructions in cyborg_ros_led_dome/README.md"
