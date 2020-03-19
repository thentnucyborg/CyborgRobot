#!/usr/bin/env python

__author__ = "Casper Nilsen"
__copyright__ = "Copyright (C) 2019 Casper Nilsen"
__license__ = "BSD"
__version__ = "0.0.1"
__all__ = []

import rospy
from std_msgs.msg import Bool

class TopicReceiver():
    """TopicReceiver"""
def __init__(self):          
    self.prev = None

    self.subscriber_behaviour_state = rospy.Subscriber("cyborg_modeselector/behaviour_state", Bool, callback = self.controller_callback, queue_size=10)
    while not rospy.is_shutdown():

        # self.publisher_behaviour_state.publish(False)
        rospy.sleep(10)

def controller_callback():
    