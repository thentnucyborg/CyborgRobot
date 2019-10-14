# LED_box
The [LED box](https://ntnu.box.com/s/54gfmdsg4xefp7i1zyaw3o8i6tqbxnvq) is used as a showpiece for visualizing live MEA/neural data. The current box is a cube of opal acrylic plates containing 240 individually controlled ws2812b RGB-LEDs controlled by an Arduino Mega.

## Getting started
See the [LED box wiki](https://github.com/thentnucyborg/LED_box/wiki)

## Prerequisites
Using [FastLED](https://github.com/FastLED/FastLED) library to control ws2812b LEDs.

## Schematic
Connecting the Arduino Mega to the ws2812b LED strip:

![Arduino schematic](schematics/arduino_ws2812b_schematic.png "Arduino Mega to ws2812b LED strip")

Powering the Arduino using a 5V power supply:

![Arduino 5V](schematics/arduino_powered_by_5V.jpg "Arduino Mega powered by 5V")