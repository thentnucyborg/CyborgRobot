#!/usr/bin/env python

__author__      = "Areg Babayan"
__date__        = "28.04.2019"
__copyright__   = "Copyright (C) 2019 Areg Babayan"
__license__     = "BSD"
__version__     = "0.0.1"
__all__         = []  


import rospy
import os
import sys
import time
import pyttsx3
import threading
from std_msgs.msg import String


class TextToSpeech():
    def __init__(self):
        self.subscriber_text_to_speech = rospy.Subscriber(rospy.get_name() + "/text_to_speech", String, callback= self.callback_text_to_speech, queue_size=10)
        self.publisher_text_to_speech = rospy.Publisher(rospy.get_name()+ "/feedback_text_to_speech", String, queue_size=10)
        self.text_to_speech_thread = threading.Thread(target=self.text_to_speech)
        self.text_to_speech_thread.daemon = True # Terminates thread when main thread terminates
        self.text_to_speech_thread.start()

    def text_to_speech(self):
        global engine
        engine = pyttsx3.init()
        engine.startLoop(False)
        engine.connect('finished-utterance', self.on_end_tts)
        rospy.loginfo("Cyborg Audio: Text to Speech Activated.")
        rospy.spin() 
        engine.endLoop()
        rospy.loginfo("Cyborg Audio: Text to Speech Deactivated")

    def on_end_tts(self, name, completed):
        message = String(data="finished")
        self.publisher_text_to_speech.publish(message)

    def callback_text_to_speech(self,message):
        global engine
        if message.data =="PreemptUtterance":
            #preempt utterance
            engine.stop()
            reply = String(data="preempted")
            self.publisher_text_to_speech.publish(reply)
        else:
            rospy.loginfo("Cyborg Audio: Executing Text to Speech. ")
            engine.say(message.data)
            engine.iterate()
