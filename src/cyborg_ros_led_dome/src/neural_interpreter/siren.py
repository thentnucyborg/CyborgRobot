#!/usr/bin/env python
import rospy
import system.settings as settings



class Siren():
    def __init__(self):
        self.previous_color = "blue"
        #refresh rate
        self.rate = rospy.Rate(3)
        self.isStatic = False

    def render(self, input_data, output_data):
        if not settings.CHANGE_REQUESTED:
            if self.previous_color == "blue":
                for i in range((settings.LEDS_TOTAL/2)-1):
                    output_data[i*3 ] = 100
                    output_data[i*3+ 2] = 0
                for i in range(settings.LEDS_TOTAL/2,settings.LEDS_TOTAL):
                    output_data[i*3 ] = 0
                    output_data[i*3 + 2] = 100
                self.previous_color = "red"
            else:
                for i in range((settings.LEDS_TOTAL/2)-1):
                    output_data[i*3 ] = 0
                    output_data[i*3+ 2] = 100
                for i in range(settings.LEDS_TOTAL/2,settings.LEDS_TOTAL):
                    output_data[i*3 ] = 100
                    output_data[i*3 + 2] = 0
                self.previous_color = "blue"
            self.rate.sleep()
