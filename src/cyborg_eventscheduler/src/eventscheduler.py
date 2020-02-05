#!/usr/bin/env python
__author__      = "Areg Babayan"
__copyright__   = "Copyright (C) 2019 Areg Babayan"
__license__     = "BSD"
__version__     = "0.0.3"
__all__         = [] 

import rospy
import sys
import os
import time
sys.path.append('/home/lassegoncz/catkin_ws/src/cyborg_ros_navigation/src/')
from databasehandler import DatabaseHandler
import datetime
import threading
from std_msgs.msg import String
#from rosarnl.msg import BatteryStatus
from cyborg_controller.msg import SystemState
from geometry_msgs.msg import PoseWithCovarianceStamped


class EventScheduler():
    """EventScheduler"""
        
    current_location = None
 
    def __init__(self):
        rospy.sleep(2) # Delay startup so navigation is finished setting up database file
        self.current_location = None
        self.SCHEDULER_RATE = rospy.Rate(0.05) #(Hz)
        self.LOW_POWER_THRESHOLD = 20
        self.HOMEDIR = os.path.expanduser("~")
        self.PATH = self.HOMEDIR + "/navigation.db"
        self.MAP_NAME = "map"
        self.current_state = "idle"

        self.publisher_event = rospy.Publisher("/cyborg_controller/register_event", String, queue_size=100)
        self.subscriber_current_location = rospy.Subscriber("cyborg_navigation/current_location", String, self.callback_current_location)
        self.subscriber_state = rospy.Subscriber( "/cyborg_controller/state_change", SystemState, self.callback_subscriber_state, queue_size=100)
        #self.subscriber_battery_status = rospy.Subscriber("/RosAria/battery_state_of_charge", Float32, callback = self.callback_battery_status, queue_size = 10)

        self.database_handler = DatabaseHandler(filename=self.PATH)
        self.scheduler_thread = threading.Thread(target=self.scheduler)
        self.scheduler_thread.daemon = True # Thread terminates when main thread terminates
        self.scheduler_thread.start()


    def callback_current_location(self, data):
        if self.current_location != data.data and data.data != "":
            self.current_location = data.data

    
	# Thread, updating current location name based on current position, checks for ongoing events and current position compared to the event, if ongoing event is an other location it publish a navigation_schedulaer event for the state machine
    def scheduler(self): # Threaded
        rospy.loginfo("EventScheduler: Activated.")
        while not rospy.is_shutdown():
            event = self.database_handler.search_ongoing_events(robot_map_name=self.MAP_NAME, current_date=datetime.datetime.now())
            if event != None:
                if event.location_name != self.current_location:
                    self.publisher_event.publish("navigation_schedular")
            self.SCHEDULER_RATE.sleep()



    # Updates the current system state when the system state subscriber receives data
    def callback_subscriber_state(self, data):
        if data.to_system_state != None:
            self.current_state = data.to_system_state

    
    # Monitor battery percentage and publish power_low event when triggered
    def callback_battery_status(self, BatteryStatus):
        # charge_percent [0,100]
        # charge_state [-1, 4], charging_unknown, charging_bulk, charging_overcharge, charging_float, charging_balance
        if (BatteryStatus.charge_percent < self.LOW_POWER_THRESHOLD) and (self.current_state not in ["exhausted", "sleepy", "sleeping"]):
            self.publisher_event.publish("power_low")




if __name__=="__main__":
    rospy.init_node("cyborg_eventscheduler")
    EventScheduler()
    rospy.spin()
