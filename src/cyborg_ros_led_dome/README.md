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
	// FastLED needs to be included before the two others.
* LEDMatrix.h - download from: https://github.com/AaronLiddiment/LEDMatrix
* LEDText.h + FontMatrise.h - download from: https://github.com/AaronLiddiment/LEDText
	// For LEDMatrix and LEdText don't use the repo RGBLEDS as it is a never version where some things have been changed.

## Features:
* Executes visualization commands published on topic cyborg_visual/domecontrol.
* Current list of interpreters: Eyes, Siren, MovingAverage, IndividualMovingAverage, Snake.
* Text visualizations are executed by publishing on topic cyborg_visual/domecontrol.  The message must start with the word "text", for vertical text visualizations, the message must start with "text vertical".

## Usage:
$ rosrun cyborg_led_dome led_dome.py


## Useful info
* To program the ESP-32 using the Arduino IDE go to https://randomnerdtutorials.com/installing-the-esp32-board-in-arduino-ide-mac-and-linux-instructions/ to install the add-on in the IDE and choose NodeMCU-32.
* Some help for troubleshooting the ESP-32 can be found here: https://randomnerdtutorials.com/esp32-troubleshooting-guide/