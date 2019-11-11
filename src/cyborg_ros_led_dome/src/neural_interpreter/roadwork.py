#!/usr/bin/env python


import system.settings as settings
import rospy


class Roadwork:
    def __init__(self):
        self.rate = rospy.Rate(3)
        self.isStatic = False
        self.previous_light = False

# oransje er rgb(255,140,0)

    def render(self, input_data, output_data):
        if not settings.CHANGE_REQUESTED:
            #blinking orange lights on the sides of the dome
            if self.previous_light: 
                for i in range((settings.LEDS_TOTAL/2)+5):
                    output_data[i*3 ]   = 0
                    output_data[i*3+1]  = 0
                for i in range(settings.LEDS_TOTAL/2+5,settings.LEDS_TOTAL):
                    output_data[i*3 ]   = 150
                    output_data[i*3+1]  = 40

                self.previous_light = False

            else:
                for i in range((settings.LEDS_TOTAL/2)+5):
                    output_data[i*3 ]   = 150
                    output_data[i*3+1]  = 40
                for i in range(settings.LEDS_TOTAL/2+5,settings.LEDS_TOTAL):
                    output_data[i*3 ]   = 0
                    output_data[i*3+1]  = 0

                self.previous_light = True

            self.rate.sleep()
