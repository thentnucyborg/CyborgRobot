#!/bin/bash

echo "Starting behavior3 editor - open http://127.0.0.1:8000 in chrome to edit"
cd ~/catkin_ws/src/behavior3editor
gnome-terminal -e 'gulp serve'
