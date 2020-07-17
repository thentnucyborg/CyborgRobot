# README
This repository contains the source code for the NTNU Cyborg's Commander Module (ROS Node). It is the top-level package, that handles modes of operation.

Node name: cyborg_commander
Language: Python

## Features: 
* Topic Transmitter transmits BSON data to MongoDB Atlas to be used in GUI.
* Topic Receiver listens to the mode of operation ROS topic which ROSBridge Server handles by requests from GUI.
* Package contains SSL certificate for ROSBridge Server, do not remove.


## Usage:
$ rosrun cyborg_commander commander.py
