# README
This repository contains the source code for the NTNU Cyborg's LED-dome and ESP32 interface.

Node name: cyborg_led_dome

## Requirements:
* ROS
* SMACH
* pyopengl
* pyopengl-accelerate
* numpy
* pandas
* colour
* pyserial

## Features:
* Executes visualization commands published on topic cyborg_visual/domecontrol.
* Current list of interpreters: Eyes, Siren, MovingAverage, IndividualMovingAverage, Snake.
* Text visualizations are executed by publishing on topic cyborg_visual/domecontrol.  The message must start with the word "text", for vertical text visualizations, the message must start with "text vertical".

## Usage:
$ rosrun cyborg_led_dome led_dome.py


