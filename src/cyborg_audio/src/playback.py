#!/usr/bin/env python

__author__      = "Areg Babayan"
__date__        = "14.03.2019"
__copyright__   = "Copyright (C) 2019 Areg Babayan"
__license__     = "BSD"
__version__     = "0.0.1"
__all__         = []  

import rospy
import os
import sys
import vlc
import time
import threading
from std_msgs.msg import String


engine = None

class Playback():
    """ Cyborg audio playback"""
    def __init__(self):
        self.playback_command = ""
        self.got_message = False
        self.playback_timeout = 200
        self.homedir = os.path.expanduser("~")
        self.subscriber_playback = rospy.Subscriber(rospy.get_name() + "/playback", String, callback = self.callback_playback, queue_size=10)
        self.publisher_playback = rospy.Publisher(rospy.get_name() + "/feedback_playback", String, queue_size=10)
        rospy.loginfo("Cyborg Audio: Playback initialized")
        self.playback_thread = threading.Thread(target = self.playback)
        self.playback_thread.daemon = True # Terminates thread when main thread terminate
        self.playback_thread.start()


    def playback(self):
        rate = rospy.Rate(5) #(Hz)
        while not rospy.is_shutdown():
            if self.got_message is True:
                if self.playback_command != "PreemptPlayback":
                    rospy.loginfo("Cyborg Audio: Executing Playback.")
                    recording = vlc.MediaPlayer(self.homedir+ "/" + self.playback_command + ".mp3")
                    recording.play()
                    rospy.sleep(0.2)
                    start_time = time.time()
                    while self.got_message is True:
                        if not recording.is_playing():
                            #playback finished
                            rospy.loginfo("Cyborg Audio: Playback finished. ")
                            message = String(data="finished")
                            self.publisher_playback.publish(message)
                            self.playback_command = ""
                            self.got_message = False
                        if self.playback_command =="PreemptPlayback":
                            rospy.loginfo("Cyborg Audio: Playback preempt Requested, preempted. ")
                            recording.stop()
                            message = String(data = "preempted")
                            self.publisher_playback.publish(message)
                            self.playback_command = ""
                            self.got_message = False
                        if (time.time() - start_time > self.playback_timeout):
                            rospy.loginfo("Cyborg Audio: Playback timed out. ")
                            recording.stop()
                            message = String(data="timeout")
                            self.publisher_playback.publish(message)
                            self.playback_command = ""
                            self.got_message = False
                        rate.sleep()
            rate.sleep()


    def callback_playback(self, message):
        self.playback_command = message.data
        self.got_message = True
            