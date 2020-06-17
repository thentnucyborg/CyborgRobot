#!/usr/bin/env python

__author__ = "Casper Nilsen"
__copyright__ = "Copyright (C) 2020 Casper Nilsen"
__license__ = "BSD"
__version__ = "0.1.1"
__all__ = []

import rospy
from std_msgs.msg import String

class TopicReceiver():
    """TopicReceiver"""
    def __init__(self):
        rospy.loginfo("Cyborg Commander: receiver initializing")

        self.subscriber_robot_mode = rospy.Subscriber("/cyborg_commander/robot_mode", String, callback = self.robot_mode_callback, queue_size=10)
        self.publisher_register_event = rospy.Publisher("/cyborg_controller/register_event", String, queue_size=100)
        self.publisher_emotion_controller = rospy.Publisher("/cyborg_controller/emotional_controller", String, queue_size=100)
        rospy.loginfo("Cyborg Commander: receiver initialized")


    def start_behaviour(self):
        self.publisher_register_event.publish("aborted")
        rospy.sleep(0.5)
        self.publisher_emotion_controller.publish("ON")
        self.publisher_register_event.publish("behaviour_start")


    def start_demo(self):
        self.publisher_register_event.publish("aborted")
        rospy.sleep(0.5)
        self.publisher_emotion_controller.publish("OFF")
        self.publisher_register_event.publish("demo_start")
        
        # demo program


    def start_manual(self):
        self.publisher_register_event.publish("aborted")
        rospy.sleep(0.5)
        self.publisher_emotion_controller.publish("OFF")
        self.publisher_register_event.publish("manual_start")
        
        # manual control program


    def stop(self):
        self.publisher_emotion_controller.publish("OFF")
        self.publisher_register_event.publish("aborted")
        rospy.sleep(0.5)
        self.publisher_register_event.publish("suspend")


    def robot_mode_callback(self, message):
        if message != None:
            modes = {
            "behaviour":self.start_behaviour,
            "demo":self.start_demo,
            "manual control":self.start_manual,
            "stop":self.stop
            }
            func = modes.get(message.data, lambda: "nothing")
            func()
        
