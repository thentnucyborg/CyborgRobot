#README
This repository contains the code for the Cyborg behaviour tree node. The whole fodirectory is a ROS node (cyborg_bt), but the behaviour tree nodes are stored in cyborg_bt_nodes. 


## Usefil info
* Morten Mjelva's master thesis from 2018 
* Johanne D. Kalland master thesis from 2020
* sources used in the master theses


## Usage
`roslaunch cyborg_bt.launch`

The navigation part of this node has its own launch file and is run using: `roslaunch cyborg_nav.launch`


To add your own nodes or expand the tree it's easiest to use the behavior3editor (`./start_behavior3editor.sh` in catkin_ws/src/cyborg_launch). Import the project json file into the editor and add any nodes or other functionality you want. The node you created either it is an action or something else needs an implementation file. Check out moveto.py for inspiration.