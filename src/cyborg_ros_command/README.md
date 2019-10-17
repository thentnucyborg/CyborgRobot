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
* Changeing of mood.  
* Tuning mood of.  

## Requirements:
* ROS
* npyscreen

Npyscreen can be installed:  
//This is pip2 for Python 2.7 (used by ROS)  
$ sudo apt-get install python-pip  
$ pip2 install npyscreen  

## Usage:  
It can be run using:  
$ rosrun cyborg_command command.py  