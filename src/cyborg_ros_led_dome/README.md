# README
This repository contains the source code for the NTNU Cyborg's LED-Dome and ESP32 interface.

Node name: cyborg_led_dome

There are 791 leds and the bytearray sent to the led-controller contains three values for each led (RGB). To access led 365: led_array[365*3+x] where you use x to set the red (+0), green (+1) or blue (+2) led. 

## Requirements:
* ROS
* SMACH
* pyopengl
* pyopengl-accelerate
* numpy
* pandas
* colour
* pyserial

Libraries needed when programming using the Arduino IDE:
* FastLED - can be added by searching for FastLED in Library Manager
* LEDMatrix.h - download from: https://github.com/AaronLiddiment/LEDMatrix
* LEDText.h + FontMatrise.h - download from: https://github.com/AaronLiddiment/LEDText

## Features:
* Executes visualization commands published on topic cyborg_visual/domecontrol.
* Current list of interpreters: Eyes, Siren, MovingAverage, IndividualMovingAverage, Snake.
* Text visualizations are executed by publishing on topic cyborg_visual/domecontrol.  The message must start with the word "text", for vertical text visualizations, the message must start with "text vertical".

## Usage:
$ rosrun cyborg_led_dome led_dome.py