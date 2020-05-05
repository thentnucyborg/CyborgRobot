#!/usr/bin/env python
"""Created by Areg Babayan on 20/9/2018.
Copyright (C) 2018 Areg Babayan. ALl rights reserved."""
__author__      = "Areg Babayan, edited by Johanne Doevle Kalland"
__copyright__   = "Copyright (C) 2018 Areg Babayan"
__license__     = "BSD"
__version__     = "0.0.3"
__all__         = []  

import rospy
import domecontrol
from neural_presenters.serial.serial_communication import SerialInterface
import system.settings as settings
import time


def shutdown_test():
	rospy.loginfo("LED - shutdown test funksjon")
	leds = bytearray([0] * (3 * settings.LEDS_TOTAL))
	leds[788*3] = 100

	led_colors = SerialInterface()
	led_colors.refresh(leds)

def startup_test():
    print("LED - startup test funksjon")
    leds = bytearray([0] * (3 * settings.LEDS_TOTAL))
    leds[789*3+2] = 100

    led_colors = SerialInterface()
    led_colors.refresh(leds)

def main():
    rospy.init_node("cyborg_led_dome")
    rospy.logwarn("Charge animation not avaliable with new navigation stack. Update charge.py to subscribe to the correct topic!")

    #print("for startup_test")
    #startup_test()
    #print("etter startup_test")
    time.sleep(5)
    print("etter sleep")


    domecontrol.domecontrol()
    rospy.on_shutdown(shutdown_test)
    rospy.spin()

if __name__ == '__main__':
    rospy.loginfo("Cyborg LED Dome: Starting Program...")
    main()
    rospy.loginfo("Cyborg LED Dome: End of Program...")
