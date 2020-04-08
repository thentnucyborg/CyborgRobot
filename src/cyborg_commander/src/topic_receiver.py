#!/usr/bin/env python

__author__ = "Casper Nilsen"
__copyright__ = "Copyright (C) 2019 Casper Nilsen"
__license__ = "BSD"
__version__ = "0.0.1"
__all__ = []

import rospy
from std_msgs.msg import Bool, String

import datetime
import threading

class TopicReceiver():
    """TopicReceiver"""
    def __init__(self):
        rospy.loginfo("Cyborg Commander: receiver initializing")

        self.subscriber_behaviour_state = rospy.Subscriber("/cyborg_commander/behaviour_state", Bool, callback = self.robot_behaviour, queue_size=10)
        self.publisher_register_event = rospy.Publisher("/cyborg_controller/register_event", String, queue_size=100)
        self.publisher_emotion_controller = rospy.Publisher("/cyborg_controller/emotional_controller", String, queue_size=100)
        rospy.loginfo("Cyborg Commander: receiver initialized")


    def save_data(self, message):
        if message != None:
            topicString = message._connection_header["topic"]
            topicString = topicString.replace("/","__")
            self.__dict__[topicString] = message.data

    def robot_behaviour(self, message):
        if message != None:
            if message.data == True:
                self.publisher_emotion_controller.publish("ON")
                self.publisher_register_event.publish("start")
            else:
                self.publisher_emotion_controller.publish("OFF")
                self.publisher_register_event.publish("aborted")
                self.publisher_register_event.publish("suspend")
