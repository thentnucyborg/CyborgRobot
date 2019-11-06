#!/usr/bin/env python
"""Created by Areg Babayan on 20/9/2018.
Copyright (C) 2018 Areg Babayan. ALl rights reserved."""
__author__      = "Areg Babayan"
__copyright__   = "Copyright (C) 2018 Areg Babayan"
__license__     = "BSD"
__version__     = "0.0.3"
__all__         = []  

import rospy
import domecontrol
from neural_presenters.serial.serial_communication import SerialInterface
import system.settings as settings


def shutdown_test():
	print("LED - shutdown test funksjon")
	leds = bytearray([0] * (3 * settings.LEDS_TOTAL))
	leds[788*3+1] = 100

	led_colors = SerialInterface()
	led_colors.refresh(leds)

def main():
    rospy.init_node("cyborg_led_dome")
    domecontrol.domecontrol()
    rospy.on_shutdown(shutdown_test)
    rospy.spin()

if __name__ == '__main__':
    print("Cyborg LED Dome: Starting Program...")
    main()
    print("Cyborg LED Dome: End of Program...")
