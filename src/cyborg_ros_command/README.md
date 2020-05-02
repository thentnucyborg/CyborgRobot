# README  
This repository contains the source code for the command tool (ROS node) for the NTNU Cyborg controller node. 

Node name: cyborg_command  
Language: Python

## Features  
  
It displays:
* Previous state.
* Event that lead to current state.
* Current state.
* Current emotion.
* Current PAD values.

Allows:
* Publishing events to the controller.  
* Publishing text on the speech_to_text topic.
* Changing of mood.  
* Tuning mood off.  

## Requirements:
* ROS
* npyscreen


## Usage:
Runs using:
$ rosrun cyborg_command command.py