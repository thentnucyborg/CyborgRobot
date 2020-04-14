#!/bin/bash

# This file is stored here so that it can be run using the launch file, and because begavior3editor isn't a ros node or package.

echo "Starting behavior3 editor - open http://127.0.0.1:8000 in chrome to edit"

cd ~/catkin_ws/src/behavior3editor
gnome-terminal -e 'gulp serve'
