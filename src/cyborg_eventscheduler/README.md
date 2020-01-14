# README
This repository contains the source code for the NTNU Cyborg's Eventscheduler Module (ROS Node). 

Node name: cyborg_eventscheduler
Language: Python


## Requirements:
* Path for databasehandler in navigaiton module, sys.path.append('/home/cyborg/catkin_ws/src/cyborg_ros_navigation/src/')
* Map Name

## Features: 
* Publishes scheduled and "power_low" to the Cyborg state machine.


## Usage:
$ rosrun cyborg_eventscheduler eventscheduler.py