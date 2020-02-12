#!/usr/bin/env python
"""Created by Thomas Rostrup Andersen on 11/11/2016.
Copyright (C) 2016 Thomas Rostrup Andersen. All rights reserved."""

import math
import sys
from collections import namedtuple
import rospy
from std_msgs.msg import String
from cyborg_controller.msg import EmotionalFeedback, EmotionalState
from cyborg_controller.srv import EmotionalStateService

__author__ = "Thomas Rostrup Andersen"
__copyright__ = "Copyright (C) 2016 Thomas Rostrup Andersen"
#__license__ = ""
__version__ = "0.0.2"
__all__ = []


class EmotionSystem(object):
    """EmotionSystem uses the 3 dimensional PAD emotion space.

    *Pleasure(P): State of pleasure.
    *Arousal(A): State of mental and/or physical activity/stimuli.
    *Dominance(D): Sense of Controll.
    The range of each value is (-1, 1).

    Usage():
        # Create system
        emotion_system = EmotionSystem()

        # Add an emotion
        emotion_system.add_emotion(name, pleasure, arousal, dominance)

        Emotional feedback (changes in PAD values) can be given at the ROS topic "cyborg_controller/emotional_feedback" using an EmotionalFeedback message.
        The emotional state is published at the ROS topic "cyborg_controller/emotional_feedback" using an EmotionalState message.
    """

    current_emotion = "neutral"
    is_on = True
    r_decay = 0.15

    def __init__(self, pleasure=0.00, arousal=0.00, dominance=0.00, radius=0.40):
        self.emotions = []
        self.pleasure = pleasure
        self.arousal = arousal
        self.dominance = dominance
        self.radius = radius
        self.feedback_subscriber = rospy.Subscriber( rospy.get_name() + "/emotional_feedback", EmotionalFeedback, self.emotional_feedback_callback, queue_size=100)
        self.set_emotional_state_subscriber = rospy.Subscriber( rospy.get_name() + "/set_emotional_state", String, self.set_emotional_state_callback, queue_size=100)
        self.set_emotional_values_subscriber = rospy.Subscriber( rospy.get_name() + "/set_emotional_values", EmotionalFeedback, self.set_emotional_value_callback, queue_size=100)
        self.emotion_publisher = rospy.Publisher( rospy.get_name() + "/emotional_state", EmotionalState, queue_size=100)
        self.emotion_service = rospy.Service( rospy.get_name() + "/get_emotional_state", EmotionalStateService, self.get_emotional_state_callback)
        self.controller_subscriber = rospy.Subscriber( rospy.get_name() + "/emotional_controler", String, self.controller_callback, queue_size=100)
        rospy.loginfo("EmotionSystem: Activated...")


    # Add an emotion to the list of emotions available
    def add_emotion(self, name, pleasure, arousal, dominance):
        Emotion = namedtuple('Emotion', ["name", 'pleasure', "arousal", "dominance"])
        emotion = Emotion(name=name, pleasure=pleasure, arousal=arousal, dominance=dominance)
        self.emotions.append(emotion)
        rospy.logdebug("EmotionSystem: Added emotion " + str(emotion))


    # Called once emotional feedback is received
    # Updates the emotional values and state if the emotion system is on (self.is_on) and publishes the changes to cyborg_controller/emotional_state
    def emotional_feedback_callback(self, data):
        if self.is_on:
            self.update_emotion(pleasure=data.delta_pleasure, arousal=data.delta_arousal, dominance=data.delta_dominance)


    # Updates the emotional values and state if the emotion system is on (self.is_on) and publishes the changes to cyborg_controller/emotional_state
    def update_emotion(self, pleasure=0.00, arousal=0.00, dominance=0.00):
        decay_pleasure = self.pleasure**3*self.r_decay
        decay_arousal = self.arousal**3*self.r_decay
        decay_dominance = self.dominance**3*self.r_decay
        self.pleasure = max(-1, min(1, self.pleasure + pleasure - decay_pleasure))
        self.arousal = max(-1, min(1, self.arousal + arousal - decay_arousal))
        self.dominance = max(-1, min(1, self.dominance + dominance - decay_dominance))
        candidate = None
        candidate_distance = 10000
        for e in self.emotions:
            distance = math.sqrt((self.pleasure - e.pleasure)**2 + (self.arousal - e.arousal)**2 + (self.dominance - e.dominance)**2)
            candidate = e.name if distance < candidate_distance and distance <= self.radius else candidate
            candidate_distance = min(distance, candidate_distance)
        data = EmotionalState()
        data.from_emotional_state = self.current_emotion
        data.to_emotional_state = candidate if candidate is not None else self.current_emotion
        data.current_pleasure = self.pleasure
        data.current_arousal = self.arousal
        data.current_dominance = self.dominance
        self.emotion_publisher.publish(data)
        self.current_emotion = candidate if candidate is not None else self.current_emotion
        rospy.logdebug("EmotionSystem: Updated...")


    # Called when the service requesting current emotional state is called
    # Responds with the current emotional state and values
    def get_emotional_state_callback(self, req):
        data = EmotionalStateServiceResponse(self.current_emotion)
        data.emotional_state = self.current_emotion
        data.current_pleasure = self.pleasure
        data.current_arousal = self.arousal
        data.current_dominance = self.dominance
        return data


    # Called from message subscriber
    # Set the received emotonal state if it exist
    def set_emotional_state_callback(self, data):
        found_emotions = [emotion for emotion in self.emotions if emotion.name == data.data]
        for emotion in found_emotions:
            self.pleasure = max(-1, min(1, emotion.pleasure))
            self.arousal = max(-1, min(1, emotion.arousal))
            self.dominance = max(-1, min(1, emotion.dominance))
            data = EmotionalState()
            data.from_emotional_state = self.current_emotion
            data.to_emotional_state = emotion.name
            data.current_pleasure = self.pleasure
            data.current_arousal = self.arousal
            data.current_dominance = self.dominance
            self.emotion_publisher.publish(data)
            self.current_emotion = emotion


    # Called from message subscriber
    # Sets the emotional value in valid range (-1,1)
    def set_emotional_value_callback(self, data):
        self.pleasure = max(-1, min(1, data.delta_pleasure))
        self.arousal = max(-1, min(1, data.delta_arousal))
        self.dominance = max(-1, min(1, data.delta_dominance))
        self.update_emotion(self, pleasure=0.00, arousal=0.00, dominance=0.00)


    # Called from message subscriber
    # Turns on/off the emotional system
    def controller_callback(self, data):
        self.is_on = True if data.data == "on" else False
