#!/usr/bin/env python
"""Created by Thomas Rostrup Andersen on 11/11/2016.
Copyright (C) 2016 Thomas Rostrup Andersen. All rights reserved."""

import math
import threading
import time
import datetime
import sys
import roslib 
import rospy
import actionlib
import geometry_msgs
from std_msgs.msg import String
from databasehandler import DatabaseHandler
from cyborg_controller.msg import EmotionalFeedback, EmotionalState, SystemState

__author__ = "Thomas Rostrup Andersen"
__copyright__ = "Copyright (C) 2016 Thomas Rostrup Andersen"
#__license__ = ""
__version__ = "0.0.2"
__all__ = []


class Motivator():
    """Motivator for the controller.

    Generate events based on reward/cost functions.

    Usage:
            motivator = Motivator()
            motivator.start()

    Add more events (possible actions):
            Add more events to the sqlitedatabase.
    """

    current_pleasure = 0
    current_arousal = 0
    current_dominance = 0
    current_emotion = "neutral"
    current_state = "idle"

    state_timer = 0 # When arrived in the current system state
    max_time = 60 # Used for calculating cost

    # Priority functions
    priority_pleasure = 0.6
    priority_arousal = 0.6
    priority_dominance = 0.6
    priority_cost = 0.7

    #counter_reduced_social_cost = 0

    def __init__(self, database_file=""):
        self.event_publisher = rospy.Publisher( rospy.get_name() + "/register_event", String, queue_size=100)
        self.emotion_publisher = rospy.Publisher( rospy.get_name() + "/emotional_feedback", EmotionalFeedback, queue_size=100)
        self.emotion_subscriber = rospy.Subscriber( rospy.get_name() + "/emotional_state", EmotionalState, self.emotion_callback, queue_size=100)
        self.state_subscriber = rospy.Subscriber( rospy.get_name() + "/state_change", SystemState, self.state_callback, queue_size=100)
        self.rate = rospy.Rate(.5) # (hz), looping speed
        self.database_handler = DatabaseHandler(filename=database_file)
        self.database_handler.reset_event_values()


    # Starts the motivator in a separate daemon thread
    def start(self):
        motivator_thread = threading.Thread(target=self.run)
        motivator_thread.daemon = True
        motivator_thread.start()


    # The running thread called when Motivator starts
    def run(self): # Threaded
        rospy.loginfo("Motivator: Activated...")
        timer_sc = time.time()
        while not rospy.is_shutdown():
            event = self.find_event()
            if event is not None:
                self.send_emotion(pleasure=event.reward_pleasure, arousal=event.reward_arousal, dominance=event.reward_dominance)
                event_value = event.event_value+event.event_cost if event.event_value+event.event_cost >= 0 else 0
                event_value = event_value if event_value <= 1 else 1
                self.database_handler.update_event_value(event_id=event.event_id, event_value=event_value)
                self.database_handler.update_all_event_values(delta_event_value=-0.05)
                self.event_publisher.publish(event.event)
                rospy.logdebug("Motivator: Current state is " + self.current_state + " and the event " + event.event + " was generated...")
            self.rate.sleep()

            if self.current_state == "idle" and time.time() - self.state_timer > 3*60:
                self.database_handler.reset_event_values()

            if time.time() - timer_sc > 10:
                timer_sc = time.time()
                self.database_handler.update_all_event_values(delta_event_value=-0.01)


    # Finds and returns the event to generate (or None if None to generate)
    def find_event(self):
        events = self.database_handler.get_all_events(state=self.current_state)
        reward = 0
        solution = None

        # A cost or minimum reward. Decreases with time spent in current state.
        time_in_state = time.time() - self.state_timer
        k_s = 1 - (time_in_state*5/self.max_time)**math.tan((self.priority_cost*math.pi)/2.0) #cost
        c_s = k_s * 1
        reward = c_s
        reward = reward if reward > 0 else 0.003

        # Find best reward greater than cost 
        for event in events:
            priority_pleasure = 1 - self.current_pleasure**3
            reward_pleasure = priority_pleasure * ((self.current_pleasure + ((event.reward_pleasure)/2.0)) - self.current_pleasure)

            priority_arousal = 1 - self.current_arousal**3 
            reward_arousal = priority_arousal * ((self.current_arousal + ((event.reward_arousal)/2.0)) - self.current_arousal)

            priority_dominance = 1 - self.current_dominance **3 
            reward_dominance = priority_dominance * ((self.current_dominance + ((event.reward_dominance)/2.0)) - self.current_dominance)

            priority_event = event.event_value**(1.0/3.0) 
            cost_event = priority_event * ((event.reward_pleasure/2.0) + (event.reward_arousal/2.0) + (event.reward_dominance/2.0))

            r = reward_pleasure + reward_arousal + reward_dominance - cost_event # tot_reward = reward_pleasure + reward_arousal + reward_dominance - cost_event
            #print(event.event,r+cost_event, -cost_event, r) # event_name, tot_reward + cost_event, -cost_event, tot_reward
            solution = event if r > reward else solution # solution = event if tot_reward > reward, else solution = solution
            reward = r if r > reward else reward # reward = tot_reward  if tot_reward > reward, else reward = reward
        return solution


    # Send the change (delta) in the emotional state to the emotion system
    def send_emotion(self, pleasure, arousal, dominance):
        msg = EmotionalFeedback()
        msg.delta_pleasure = pleasure
        msg.delta_arousal = arousal
        msg.delta_dominance = dominance
        self.emotion_publisher.publish(msg)


    # Updates the current emotion when the emotion subscriber recives data
    # Normalizes the data.
    def emotion_callback(self, data):
        self.current_emotion = data.to_emotional_state
        self.current_pleasure = (data.current_pleasure + 1)/2.0
        self.current_arousal = (data.current_arousal + 1)/2.0
        self.current_dominance = (data.current_dominance + 1)/2.0


    # Updates the current system state when the system state subscriber receives data
    def state_callback(self, data):
        self.current_state = data.to_system_state
        self.state_timer = time.time()
