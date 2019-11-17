#!/usr/bin/env python

__author__ = "Casper Nilsen"
__copyright__ = "Copyright (C) 2019 Casper Nilsen"
__license__ = "BSD"
__version__ = "0.0.4"
__all__ = []


import rospy
import time
import actionlib

import json
import yaml

import sys
import binascii
import datetime
import threading

# ROS msgs
from std_msgs.msg import String, Bool
from rosgraph_msgs.msg import Log
from rosbridge_library.internal import message_conversion as c

from cyborg_controller.msg import StateMachineAction, StateMachineGoal, SystemState, EmotionalState, StateMachineActionResult, EmotionalFeedback, StateMachineActionGoal, StateMachineActionFeedback
from cyborg_navigation.msg import NavigationAction, NavigationGoal, NavigationActionFeedback, NavigationActionGoal, NavigationActionResult
from actionlib_msgs.msg import GoalStatusArray, GoalID
from smach_msgs.msg import SmachContainerStatus,SmachContainerInitialStatusCmd,SmachContainerStructure
from pymongo import MongoClient 
from rospy_message_converter import json_message_converter
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Pose2D, PoseStamped
from rosarnl.msg import BatteryStatus, BumperState, JogPositionActionFeedback, JogPositionActionGoal, JogPositionActionResult
from sensor_msgs.msg import LaserScan, PointCloud
from move_base_msgs.msg import MoveBaseActionFeedback, MoveBaseActionResult, MoveBaseActionGoal


client = MongoClient() 
# topic = "/rosarnl_node/battery_status"
# topic = topic.replace("/","__")

def getx(self):
    return self._x

def setx(self, value):
    self._x = value

def delx(self):
    del self._x

class TopicTransmitter():
    """TopicTransmitter"""

    def __init__(self):

        # self.topic_transmitter_thread = threading.Thread(target=sm.execute)
        # self.topic_transmitter_thread.daemon = True
        # self.topic_transmitter_thread.start()
        
        # self.subscriber_audio_feedback_playback = rospy.Subscriber("/cyborg_audio/feedback_playback", String, callback = self.save_data, queue_size = 10)
        
        # self.subscriber_behaviour_cancel = rospy.Subscriber("/cyborg_behavior/cancel", GoalID, callback = self.save_data, queue_size = 10)
        # self.subscriber_behaviour_command_location = rospy.Subscriber("/cyborg_behavior/command_location", String, callback = self.save_data, queue_size = 10)
        # self.subscriber_behaviour_dynamic_behavior = rospy.Subscriber("/cyborg_behavior/dynamic_behavior", String, callback = self.save_data, queue_size = 10)
        # self.subscriber_behaviour_goal = rospy.Subscriber("/cyborg_behavior/goal", StateMachineActionGoal, callback = self.save_data, queue_size = 10)
        # self.subscriber_behaviour_result = rospy.Subscriber("/cyborg_behavior/result", StateMachineActionResult, callback = self.save_data, queue_size = 10)
        # self.subscriber_behaviour_status = rospy.Subscriber("/cyborg_behavior/status", GoalStatusArray, callback = self.save_data, queue_size = 10)

        # self.subscriber_controller_emotional_controller = rospy.Subscriber("/cyborg_controller/emotional_controller", String, callback = self.save_data, queue_size = 10)
        # self.subscriber_controller_emotional_feedback = rospy.Subscriber("/cyborg_controller/emotional_feedback", EmotionalFeedback, callback = self.save_data, queue_size = 10)
        # self.subscriber_controller_emotional_state = rospy.Subscriber("/cyborg_controller/emotional_state", EmotionalState, callback = self.save_data, queue_size = 10)
        # self.subscriber_controller_register_event = rospy.Subscriber("/cyborg_controller/register_event", String, callback = self.save_data, queue_size = 10)
        # self.subscriber_controller_set_emotional_state = rospy.Subscriber("/cyborg_controller/set_emotional_state", String, callback = self.save_data, queue_size = 10)
        # self.subscriber_controller_set_emotional_values = rospy.Subscriber("/cyborg_controller/set_emotional_values", EmotionalFeedback, callback = self.save_data, queue_size = 10)
        # self.subscriber_controller_state_change = rospy.Subscriber("/cyborg_controller/state_change", SystemState, callback = self.save_data, queue_size = 10)
        
        # self.subscriber_navigation_current_location = rospy.Subscriber("/cyborg_navigation/current_location", String, callback = self.save_data, queue_size = 10)
        # self.subscriber_navigation_cancel = rospy.Subscriber("/cyborg_navigation/navigation/cancel", GoalID, callback = self.save_data, queue_size = 10)
        # self.subscriber_navigation_feedback = rospy.Subscriber("/cyborg_navigation/navigation/feedback", NavigationActionFeedback, callback = self.save_data, queue_size = 10)
        # self.subscriber_navigation_goal = rospy.Subscriber("/cyborg_navigation/navigation/goal", NavigationActionGoal, callback = self.save_data, queue_size = 10)
        # self.subscriber_navigation_result = rospy.Subscriber("/cyborg_navigation/navigation/result", NavigationActionResult, callback = self.save_data, queue_size = 10)
        # self.subscriber_navigation_status = rospy.Subscriber("/cyborg_navigation/navigation/status", GoalStatusArray, callback = self.save_data, queue_size = 10)
        
        # self.subscriber_primarystates_cancel = rospy.Subscriber("/cyborg_primary_states/cancel", GoalID, callback = self.save_data, queue_size = 10)
        # self.subscriber_primarystates_feedback = rospy.Subscriber("/cyborg_primary_states/feedback", StateMachineActionFeedback, callback = self.save_data, queue_size = 10)
        # self.subscriber_primarystates_goal = rospy.Subscriber("/cyborg_primary_states/goal", StateMachineActionGoal, callback = self.save_data, queue_size = 10)
        # self.subscriber_primarystates_result = rospy.Subscriber("/cyborg_primary_states/result", StateMachineActionResult, callback = self.save_data, queue_size = 10)
        # self.subscriber_primarystates_status = rospy.Subscriber("/cyborg_primary_states/status", GoalStatusArray, callback = self.save_data, queue_size = 10)
        
        # self.subscriber_visual_domecontrol = rospy.Subscriber("/cyborg_visual/domecontrol", String, callback = self.save_data, queue_size = 10)
        
        # self.subscriber_arnl_pose = rospy.Subscriber("/rosarnl_node/amcl_pose", PoseWithCovarianceStamped, callback = self.save_data, queue_size = 10)
        # self.subscriber_arnl_pathstate = rospy.Subscriber("/rosarnl_node/arnl_path_state", String, callback = self.save_data, queue_size = 10)
        # self.subscriber_arnl_servermode = rospy.Subscriber("/rosarnl_node/arnl_server_mode", String, callback = self.save_data, queue_size = 10)
        # self.subscriber_arnl_serverstatus = rospy.Subscriber("/rosarnl_node/arnl_server_status", String, callback = self.save_data, queue_size = 10)
        self.subscriber_arnl_batterystatus = rospy.Subscriber("/rosarnl_node/battery_status", BatteryStatus, callback = self.save_data, queue_size = 10)
        # self.subscriber_arnl_bumperstate = rospy.Subscriber("/rosarnl_node/bumper_state", BumperState, callback = self.save_data, queue_size = 10)
        # self.subscriber_arnl_currentgoal = rospy.Subscriber("/rosarnl_node/current_goal", Pose, callback = self.save_data, queue_size = 10)
        # self.subscriber_arnl_goalname = rospy.Subscriber("/rosarnl_node/goalname", String, callback = self.save_data, queue_size = 10)
        # self.subscriber_arnl_initialpose = rospy.Subscriber("/rosarnl_node/initialpose",PoseWithCovarianceStamped, callback = self.save_data, queue_size = 10)
        # self.subscriber_arnl_jogpostition_cancel = rospy.Subscriber("/rosarnl_node/jog_position/cancel", GoalID, callback = self.save_data, queue_size = 10)
        # self.subscriber_arnl_jogpostition_feedback = rospy.Subscriber("/rosarnl_node/jog_position/feedback", JogPositionActionFeedback, callback = self.save_data, queue_size = 10)
        # self.subscriber_arnl_jogpostition_goal = rospy.Subscriber("/rosarnl_node/jog_position/goal", JogPositionActionGoal, callback = self.save_data, queue_size = 10)
        # self.subscriber_arnl_jogpostition_result = rospy.Subscriber("/rosarnl_node/jog_position/result", JogPositionActionResult, callback = self.save_data, queue_size = 10)
        # self.subscriber_arnl_jogpostition_status = rospy.Subscriber("/rosarnl_node/jog_position/status", GoalStatusArray, callback = self.save_data, queue_size = 10)
        # self.subscriber_arnl_jogpostitionsimple_goal = rospy.Subscriber("/rosarnl_node/jog_position_simple/goal", Pose2D, callback = self.save_data, queue_size = 10)
        
        self.subscriber_arnl_motorsstate = rospy.Subscriber("/rosarnl_node/motors_state", Bool, callback = self.save_data, queue_size = 10)
        # self.subscriber_arnl_movebase_cancel = rospy.Subscriber("/rosarnl_node/move_base/cancel", GoalID, callback = self.save_data, queue_size = 10)
        # self.subscriber_arnl_status = rospy.Subscriber("/rosarnl_node/move_base/feedback", MoveBaseActionFeedback, callback = self.save_data, queue_size = 10)
        # self.subscriber_arnl_movebase_goal = rospy.Subscriber("/rosarnl_node/move_base/goal", MoveBaseActionGoal, callback = self.save_data, queue_size = 10)
        # self.subscriber_arnl_movebase_result = rospy.Subscriber("/rosarnl_node/move_base/result", MoveBaseActionResult, callback = self.save_data, queue_size = 10)
        # self.subscriber_arnl_movebase_status = rospy.Subscriber("/rosarnl_node/move_base/status", GoalStatusArray, callback = self.save_data, queue_size = 10)
        # self.subscriber_arnl_movebasesimple_goal = rospy.Subscriber("/rosarnl_node/move_base_simple/goal", PoseStamped, callback = self.save_data, queue_size = 10)
        # self.subscriber_arnl_laserscan = rospy.Subscriber("/rosarnl_node/sim_S3Series_1_laserscan", LaserScan, callback = self.save_data, queue_size = 10)
        # self.subscriber_arnl_pointcloud = rospy.Subscriber("/rosarnl_node/sim_S3Series_1_pointcloud", PointCloud, callback = self.save_data, queue_size = 10)

    def save_data(self, message):   
        topicString = message._connection_header["topic"]
        topicString = topicString.replace("/","__")
        if message.data != None:
            self.__dict__[topicString] = message

            if not topicString + "_Thread" in self.__dict__:
                self.__dict__[topicString + "_Thread"] = threading.Thread(target=self.transmit_data(15, topicString))
        

    def transmit_data(self, interval, topic):
        while not rospy.is_shutdown():
            if topic in self.__dict__:
                now = datetime.datetime.now()

                json_str = json_message_converter.convert_ros_message_to_json(self.__dict__[topic])

                client = MongoClient("mongodb+srv://cyborg:hmPHK#4.iunGKD2@cyborg-gui-mg1nk.azure.mongodb.net/test?retryWrites=true&w=majority")
                
                # Access database
                mydb = client['cyborg_data']

                # Access collection of the database
                db_collection = mydb[topic]
                
                # rec = self.PrimaryStates_goal
                rec = json.loads(json_str)
                rec["date"] = now
                
                # inserting the data in the database
                db_collection.insert(rec)

                # Debug
                #print("sent topic: " + topic + " - - - \n containing data: " + json_str + "- - - \n to db")

            rospy.sleep(interval)