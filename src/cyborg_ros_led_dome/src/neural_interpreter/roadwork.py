#!/usr/bin/env python


import system.settings as settings
import rospy


class Roadwork:
	def __init__(self):
		self.rate = rospy.Rate(3)
		self.isStatic = False


	def render(self, input_data, output_data):
		if not settings.CHANGE_REQUESTED:
			

			self.rate.sleep()
