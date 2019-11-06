#!/usr/bin/env python

import system.settings as settings
import rospy

class Charge:
	def __init__(self, charge_msg):
		self.rate = rospy.Rate(3)
		self.isStatic = False
		self.battery_charge = charge_msg

	def render(self, input_data, output_data):
		100/len(output_data)
			

		self.rate.sleep()
