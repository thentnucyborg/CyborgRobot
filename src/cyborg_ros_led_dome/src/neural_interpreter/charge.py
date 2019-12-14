#!/usr/bin/env python

import system.settings as settings
import rospy
import math
from rosarnl.msg import BatteryStatus

class Charge:
    def __init__(self):
        self.rate = rospy.Rate(3)
        self.isStatic = False
        self.battery_charge = 0.0
        self.leds_charged = 0
        self.red_battery = [245,246,247,281,280,279,278,277,313,317,350,346,381,385,418,414,449,450,451,452,453]


        subscriber_battery_status = rospy.Subscriber("/rosarnl_node/battery_status", BatteryStatus, self.battery_status_callback)

    def battery_status_callback(self, message):
        # might want to change this to the whole msg later
        print("callback function charge")
        self.battery_charge = message.charge_percent


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

'''
Example battery state message printed in terminal:
    charging_state: 0
    charge_percent: 51.6100006104
    ---
    charging_state: 0
    charge_percent: 51.5099983215
    ---

Example msg published to topic rosarnl/battery_status:
    rostopic pub /rosarnl_node/battery_status rosarnl/BatteryStatus 0 60.0

'''