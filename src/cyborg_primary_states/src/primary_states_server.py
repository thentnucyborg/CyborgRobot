#!/usr/bin/env python
__author__      = "Areg Babayan"
__copyright__   = "Copyright (C) 2019 Areg Babayan"
__license__     = "BSD"
__version__     = "0.1"
__all__         = []

import rospy
import time
import os
import sys
sys.path.append('/home/cyborg/catkin_ws/src/cyborg_ros_navigation/src/')
from databasehandler import DatabaseHandler
import datetime
import actionlib
from std_msgs.msg import String
from cyborg_controller.msg import StateMachineAction, StateMachineGoal, EmotionalState, EmotionalFeedback
from rosarnl.msg import BatteryStatus


class PrimaryStatesServer():
    """Primary States Server"""

    def __init__(self, database_file =""):
        self.action_name = rospy.get_name()
        self.MAP_NAME = "glassgarden.map"
        self.PLANNING_TIMEOUT = 10
        self.RATE_ACTIONLOOP = rospy.Rate(4) # Hz

        self.next_location = ""
        self.current_emotional_state = "neutral"
        self.current_state = ""
        self.database_handler = DatabaseHandler(filename=database_file)

        self.publisher_emotional_feedback = rospy.Publisher("/cyborg_controller/emotional_feedback", EmotionalFeedback, queue_size=100)
        self.subscriber_emotional_state = rospy.Subscriber("/cyborg_controller/emotional_state", EmotionalState, self.emotional_callback, queue_size= 10)
        self.publisher_event = rospy.Publisher("/cyborg_controller/register_event", String, queue_size=100)
        self.publisher_location_command = rospy.Publisher("/cyborg_behavior/command_location", String, queue_size= 10)

        self.client_behavior = actionlib.SimpleActionClient("/cyborg_behavior", StateMachineAction)
        self.server_primary_states = actionlib.SimpleActionServer(self.action_name, StateMachineAction, execute_cb = self.callback_server_primary_states, auto_start = False)
        self.server_primary_states.start()
        rospy.loginfo("PrimaryStatesServer: Initiated")


    def callback_server_primary_states(self, goal):
        self.state_goal = goal
        if self.state_goal.current_state == "wandering_emotional":
            self.wandering_emotional()
        elif self.state_goal.current_state =="navigation_planning":
            self.navigation_planning_state(self.state_goal)


    def emotional_callback(self, message):
        self.current_emotional_state = message.to_emotional_state


    def create_and_send_behavior_goal(self, behavior, location = None):
        # Create and send goal to behaviorserver
        goal = StateMachineGoal()
        goal.current_state = behavior
        if location != None:
            goal.order = location
        self.client_behavior.send_goal(goal)


    # Check execution of action
    def actionloop_check(self):
        # Check if preempt is requested
        if self.server_primary_states.is_preempt_requested():
            self.client_behavior.cancel_all_goals()
            self.server_primary_states.set_preempted()
            rospy.loginfo("PrimaryStatesServer: %s state preempted.", self.current_state)
            return True
        else:
            # Check behaviorserver state
            behaviorserver_state = self.client_behavior.get_state()
                # not active
            if behaviorserver_state == 3:
                # succeeded
                rospy.loginfo("PrimaryStatesServer: %s state BehaviorServer goal succeeded. ", self.current_state) 
                self.server_primary_states.set_succeeded()
                return True
            elif behaviorserver_state == 2:
                # Preempted
                rospy.loginfo("PrimaryStatesServer: %s state BehaviorServer goal was preempted. ", self.current_state) 
                self.server_primary_states.set_preempted()
                return True
            elif behaviorserver_state == 4:
                # aborted
                rospy.loginfo("PrimaryStatesServer: %s state BehaviorServer goal aborted. ", self.current_state)
                self.server_primary_states.set_aborted()
                return True



    def wandering_emotional(self):
        rospy.loginfo("PrimaryStatesServer: Executing wander emotional state.")
        #connect to server and send goal
        goal = "wandering_emotional"
        self.create_and_send_behavior_goal(goal)
        while not rospy.is_shutdown():
            if self.actionloop_check() == True:
                return
            elif self.current_emotional_state not in ["bored", "curious", "unconcerned"]: # emotions currently not integrated in server
                self.client_behavior.cancel_all_goals()
                rospy.sleep(2)
                self.server_primary_states.set_succeeded()
                rospy.loginfo("PrimaryStatesServer: wandering complete, preempted.")
                return
            self.RATE_ACTIONLOOP.sleep()
        # set terminal goal status in case of shutdown
        self.server_primary_states.set_aborted()



    # Called when the controller (state machine) sets the navigation_planning state as active
    def navigation_planning_state(self, goal):
        rospy.loginfo("PrimaryStatesServer: Executing navigation_planning state.")
        time.sleep(1) # let roscore update connections 
        self.next_location = None
        if goal.event == "navigation_schedular":
            if self.current_emotional_state == "angry":
                self.next_location = self.database_handler.search_for_crowded_locations(robot_map_name=self.MAP_NAME, crowded=False)
                message = String(data = self.next_location.location_name)
                self.publisher_location_command.publish(message)
                self.send_emotion(pleasure=0, arousal=0, dominance=0.1)
                self.change_state(event="navigation_start_moving_schedular")
            else:
                self.next_location = self.database_handler.search_ongoing_events(robot_map_name=self.MAP_NAME, current_date=datetime.datetime.now())
                message = String(data = self.next_location.location_name)
                self.publisher_location_command.publish(message)
                self.send_emotion(pleasure=0, arousal=0, dominance=-0.1)
                self.change_state(event="navigation_start_moving_schedular")

        elif goal.event == "navigation_emotional": 
            if self.current_emotional_state in ["angry", "sad", "fear", "inhibited"]:
                self.next_location = self.database_handler.search_for_crowded_locations(robot_map_name=self.MAP_NAME, crowded=False)
                message = String(data = self.next_location.location_name)
                self.publisher_location_command.publish(message)
                self.send_emotion(pleasure=0, arousal=0, dominance=0.1)
                self.change_state(event="navigation_start_moving_emotional")

            elif self.current_emotional_state in ["happy", "loved", "dignified", "neutral", "elated"]:
                self.next_location = self.database_handler.search_for_crowded_locations(robot_map_name=self.MAP_NAME, crowded=True)
                message = String(data = self.next_location.location_name)
                self.publisher_location_command.publish(message)
                self.send_emotion(pleasure=0, arousal=0, dominance=0.1)
                self.change_state(event="navigation_start_moving_emotional")

            elif self.current_emotional_state in ["bored", "curious", "unconcerned"]:
                self.next_location = "wandering"
                self.send_emotion(pleasure=0, arousal=0, dominance=0.1)
                self.change_state(event="navigation_start_wandering")



    def change_state(self, event=None):
        if event ==None:
            self.server_primary_states.set_aborted()
            return
        else:
            if event =="navigation_start_moving":
                if self.next_location != None:
                    self.publisher_event.publish(event) 
            else:
                self.publisher_event.publish(event)
        # wait until state is preempted, or abort if it takes too long
        started_waiting = time.time()
        while not rospy.is_shutdown():
            if self.server_primary_states.is_preempt_requested():
                self.server_primary_states.set_preempted()
                return
            elif (time.time() - started_waiting > self.PLANNING_TIMEOUT):
                self.server_primary_states.set_aborted()
                return
            self.RATE_ACTIONLOOP.sleep()
        # set terminal goal status in case of shutdown
        self.server_primary_states.set_aborted()
            


    # Sends the emotional change to the controller
    def send_emotion(self, pleasure, arousal, dominance):
        msg = EmotionalFeedback()
        msg.delta_pleasure = pleasure
        msg.delta_arousal = arousal
        msg.delta_dominance = dominance
        self.publisher_emotional_feedback.publish(msg)

