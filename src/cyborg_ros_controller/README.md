#README
This repository contains the source code for the NTNU Cyborg`s controller (ROS node).

Node name: cyborg_controller  
Language: Python  

## Requirements:
* ROS  
* SMACH 

SMACH can be installed:
$ sudo apt-get install ros-kinetic-smach

## Features:
* State machine: The Cyborg organizes all actions (ROS actionlib servers) into a state machine.
  * Use executive_smach_visualization: (https://github.com/thentnucyborg/executive_smach_visualization) to visualize the state machine
* Emotion System: The Cyborg`s emotional state can be influenced by other modules.
* Motivator: If the Cyborg is idle, it selects the event (action) that gives the largest reward. 

Database location is at ~/controller.db  

## Usage:
$ rosrun cyborg_controller controller.py
