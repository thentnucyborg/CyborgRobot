#!/usr/bin/env python

import system.settings as settings
import rospy
import math
from rosarnl.msg import BatteryStatus



class Charge:
    def __init__(self, charge_msg):
        self.rate = rospy.Rate(3)
        self.isStatic = False
        self.battery_charge = 0.0
        self.leds_charged = 0

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
            self.leds_charged = int((self.battery_charge * (settings.LEDS_TOTAL/100))//1)
            for leds in range(self.leds_charged-1):
                output_data[leds*3+1] = 10

            #battery image
            output_data[245*3] = 10
            output_data[246*3] = 10
            output_data[247*3] = 10
            output_data[281*3] = 10
            output_data[280*3] = 10
            output_data[279*3] = 10
            output_data[278*3] = 10
            output_data[277*3] = 10
            output_data[313*3] = 10
            output_data[317*3] = 10
            output_data[350*3] = 10
            output_data[346*3] = 10
            output_data[381*3] = 10
            output_data[385*3] = 10
            output_data[418*3] = 10
            output_data[414*3] = 10
            output_data[449*3] = 10
            output_data[450*3] = 10
            output_data[451*3] = 10
            output_data[452*3] = 10
            output_data[453*3] = 10
            #turn off green
            output_data[245*3+1] = 0
            output_data[246*3+1] = 0
            output_data[247*3+1] = 0
            output_data[281*3+1] = 0
            output_data[280*3+1] = 0
            output_data[279*3+1] = 0
            output_data[278*3+1] = 0
            output_data[277*3+1] = 0
            output_data[313*3+1] = 0
            output_data[317*3+1] = 0
            output_data[350*3+1] = 0
            output_data[346*3+1] = 0
            output_data[381*3+1] = 0
            output_data[385*3+1] = 0
            output_data[418*3+1] = 0
            output_data[414*3+1] = 0
            output_data[449*3+1] = 0
            output_data[450*3+1] = 0
            output_data[451*3+1] = 0
            output_data[452*3+1] = 0
            output_data[453*3+1] = 0
        
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