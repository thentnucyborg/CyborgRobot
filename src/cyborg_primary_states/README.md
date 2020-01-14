# README
This repository contains the source code for the NTNU Cyborg's Primary States Module (ROS node).

Node name: cyborg_primary_states
Language: Python
Number of actionlib server(s): 1

## Requirements:
* Databasehandler from navigation module
	*"sys.path.append('/home/cyborg/catkin_ws/src/cyborg_ros_navigation/src/')"
* Map name
## Features:
* The planning state: Selects next navigation state based on mood. Finds next location and publishes to the navigation module.
Finds the next location. Available at actionlib server topic cyborg_navigation/planning.
* The wandering emotional state: Activates wandering behavior, preempts state when Cyborg is on the wrong mood.

## Usage:
$ rosrun cyborg_primary_states primary_states.py
