#!/usr/bin/env python

import system.settings as settings
import numpy as np
import rospy

class Startup:
	def __init__(self):
		self.rate = rospy.Rate(5)
		self.isStatic = False
		self.colour_array = np.array([	[80,0,0],		# red
										[128,64,0],		# orange
										[128,128,0],	# yellow
										[64,128,0],		# light green
										[0,150,0],		# green
										[0,128,64],		# turquoise
										[0,128,128],	# cyan
										[0,64,128],		# light blue
										[0,0,100],		# blue
										[64,0,128],		# violet
										[128,0,128],	# magenta
										[150,0,64]		# pink
										])
		self.counter = 0
		self.colours = 0

	def render(self, input_data, output_data):
		for i in range(0,settings.LED_ROWS-1, 3):
			for led in range(settings.LED_ARRAY_ROWS[i][1], settings.LED_ARRAY_ROWS[i+2][2]+1):
				if self.counter == 12:
					self.counter = 0

				output_data[led*3]	 = self.colour_array[self.counter][0]
				output_data[led*3+1] = self.colour_array[self.counter][1]
				output_data[led*3+2] = self.colour_array[self.counter][2]
			
			self.counter += 1
		
		if self.colours == 12:
			self.colours = 0
		else:
			self.colours +=1

		self.counter = self.colours
		self.rate.sleep()