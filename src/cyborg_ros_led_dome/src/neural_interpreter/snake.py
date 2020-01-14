#!/usr/bin/env python
import system.settings as settings
import rospy

class Snake():
    def __init__(self):
        #refresh rate
        self.rate = rospy.Rate(16)
        self.isStatic = False
        self.index = 3


    def render (self,input_data, output_data):
        output_data[self.index*3 +2] = 100    
        output_data[(self.index-1)*3 +2] = 50  
        output_data[(self.index-2)*3 +2] = 20
        output_data[(self.index-3)*3 +2] = 10
        output_data[(self.index-4)*3 +2] = 0
        output_data[(self.index-5)*3 +2] = 0
        output_data[(self.index-6)*3 +2] = 0
        output_data[(self.index-7)*3 +2] = 0
        if ((settings.LEDS_TOTAL - self.index)<=2):
            #end of led strip, reset index and clear leds
            output_data[(self.index*3) +2] = 0
            output_data[((self.index-1)*3) +2] = 0
            output_data[((self.index-2)*3) +2] = 0
            output_data[((self.index-3)*3) +2] = 0
            self.index = 3
        else:
            self.index+=2

        self.rate.sleep()