# README
This repository contains the source code for the NTNU Cyborg's Navigation Module (ROS node).

Node name: cyborg_navigation   
Language: Python  
Numbers of actionlib server(s): 1   


## Requirements:  
* ROS
* rosAria  
* ROS Move Base Message Type:

ROS move base message type can be installed:
$ sudo apt-get install ros-kinetic-move-base
  
## Features:   
* The go_to state: The Cyborg is moving to the next location. Available at actionlib server topic cyborg_navigation/navigation.  
* The wandering state: The Cyborg is wandering until preempted. Available at actionlib server topic cyborg_navigation/navigation.
* The docking state: Not yet finished.
* Publishes current location on topic cyborg_navigation/current_location.


Database location is at ~/catkin_ws/src/cyborg_navigation/src/navigation.db  

## Usage:
$ rosrun cyborg_navigation navigation.py


## Tips and Tricks
* If it seems that the sensor output from the cyborg is mirriored/flipped in rviz (meaning it doesn't match with the map after setting a pose estimate), go to /usr/local/Aria/params/pinoeer-lx.p and change the variable 'LaserFlipped' to false (from true). This should flipp the sensor output back the right way.