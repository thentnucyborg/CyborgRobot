#!/usr/bin/env python

import system.settings as settings
import rospy
import math
from std_msgs.msg import Float32

class Charge:
    def __init__(self):
        self.rate = rospy.Rate(3)
        self.isStatic = False
        self.battery_charge = 0.0
        self.leds_charged = 0
        self.red_battery = [245,246,247,281,280,279,278,277,313,317,350,346,381,385,418,414,449,450,451,452,453]

        self.subscriber_battery_charge = rospy.Subscriber("/RosAria/battery_state_of_charge", Float32, self.battery_status_callback)


    def battery_status_callback(self, message):
        rospy.loginfo("callback function charge")
        self.battery_charge = message.data


    def render(self, input_data, output_data):
        if not settings.CHANGE_REQUESTED:
            #flush led_colors array
            for i in range((settings.LEDS_TOTAL*3)-1):
                output_data[i] = 0

            #calculate number of leds to turn on
            self.leds_charged = int((self.battery_charge * (settings.LEDS_TOTAL/100.0))//1)
            for leds in range(self.leds_charged):
                output_data[leds*3+1] = 10

            for x in self.red_battery:
                #turn off green leds
                output_data[x*3+1] = 0
                #turn on red leds
                output_data[x*3] = 10

            self.rate.sleep()