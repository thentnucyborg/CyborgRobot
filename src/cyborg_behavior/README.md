# README
This repository contains the source code for the NTNU Cyborg's Behavior Module (ROS Node). 

Node name: cyborg_behavior
Language: Python


## Requirements:
* ROS
* cyborg_controller.msg
* cyborg_navigation.msg

## Features: 
* Behavioral presets are executed by interfacing the modules actionlib server on topic /cyborg_behavior.
* Behavioral presets are added by appending new configurations in the behavior.launch-file.


## Usage:
$ rosrun cyborg_behavior behavior.py