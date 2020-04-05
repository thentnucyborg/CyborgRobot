#!/usr/bin/env python

__author__ = "Casper Nilsen"
__copyright__ = "Copyright (C) 2019 Casper Nilsen"
__license__ = "BSD"
__version__ = "0.0.1"
__all__ = []

import rospy
from std_msgs.msg import Bool, String
from cyborg_controller.msg import SystemState, EmotionalState

import datetime
import threading

class TopicReceiver():
    """TopicReceiver"""
    def __init__(self):
        rospy.loginfo("Cyborg Commander: receiver initializing")

        # self.subscriber_behaviour_state = rospy.Subscriber("/cyborg_commander/behaviour_state", Bool, callback = self.save_data, queue_size=10)
        self.subscriber_behaviour_state = rospy.Subscriber("/cyborg_commander/behaviour_state", Bool, callback = self.robot_behaviour, queue_size=10)
        self.publisher_register_event = rospy.Publisher("/cyborg_controller/register_event", String, queue_size=100)
        self.publisher_state_change = rospy.Publisher("/cyborg_controller/state_change", SystemState, queue_size=100)
        self.publisher_emotion_controller = rospy.Publisher("/cyborg_controller/emotional_controller", String, queue_size=100)
        self.publisher_emotion_state = rospy.Publisher( "/cyborg_controller/emotional_state", EmotionalState, queue_size=100)


        self.__dict__["loop_Thread"] = threading.Thread(target=self.loop)
        self.__dict__["loop_Thread"].daemon = True # Terminates thread when main thread terminate
        # self.__dict__["loop_Thread"].start()

        rospy.loginfo("Cyborg Commander: receiver initialized")


    def save_data(self, message):
        if message != None:
            topicString = message._connection_header["topic"]
            topicString = topicString.replace("/","__")
            self.__dict__[topicString] = message.data


    def loop(self):
        while not rospy.is_shutdown():
            print("try to turn off:")
            if self.__dict__["__cyborg_commander__behaviour_state"]:
                self.publisher_emotion_controller.publish("ON")
            else:
                pass
                # self.publisher_register_event.publish("aborted")

                # system_state = SystemState()
                # system_state.event = "aborted"
                # system_state.from_system_state = "idle"
                # system_state.to_system_state = "idle"
                # self.publisher_state_change.publish(system_state)

                # self.publisher_emotion_controller.publish("OFF")

                # emotion_state = EmotionalState()
                # emotion_state.from_emotional_state = "idle"
                # emotion_state.to_emotional_state = "idle"
                # emotion_state.current_pleasure = 0.1
                # emotion_state.current_arousal = 0.1
                # emotion_state.current_dominance = 0.1
                # self.publisher_emotion_state.publish(emotion_state)




            

            # while not "__cyborg_commander__behaviour_state" in self.__dict__:
                # self.publisher_register_event.publish("aborted")
                # self.publisher_emotion_controller.publish("{event: 'aborted', from_system_state: 'idle', to_system_state: 'idle'}")
                # self.publisher_state_change("OFF")
                # rospy.sleep(1)

            # while not self.__dict__["__cyborg_commander__behaviour_state"]:
            #    self.publisher_register_event.publish("aborted")
            #    self.publisher_emotion_controller.publish("{event: 'aborted', from_system_state: 'idle', to_system_state: 'idle'}")
            #    self.publisher_state_change("OFF")
                # rospy.sleep(1)
            rospy.sleep(1)


    def robot_behaviour(self, message):
        if message != None:
            if message.data == True:
                self.publisher_emotion_controller.publish("ON")
            else:
                self.publisher_register_event.publish("aborted")

                system_state = SystemState()
                system_state.event = "aborted"
                system_state.from_system_state = "idle"
                system_state.to_system_state = "idle"
                self.publisher_state_change.publish(system_state)

                self.publisher_emotion_controller.publish("OFF")

                emotion_state = EmotionalState()
                emotion_state.from_emotional_state = "idle"
                emotion_state.to_emotional_state = "idle"
                emotion_state.current_pleasure = 0.1
                emotion_state.current_arousal = 0.1
                emotion_state.current_dominance = 0.1
                self.publisher_emotion_state.publish(emotion_state)
