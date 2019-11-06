#!/usr/bin/env python

__author__ = "Casper Nilsen"
__copyright__ = "Copyright (C) 2019 Casper Nilsen"
__license__ = "BSD"
__version__ = "0.0.1"
__all__ = []


import rospy
import time
import actionlib

import json
import yaml

from std_msgs.msg import String
from rosgraph_msgs.msg import Log
from cyborg_controller.msg import StateMachineAction, StateMachineGoal, SystemState, EmotionalState, EmotionalFeedback, StateMachineActionGoal
from cyborg_navigation.msg import NavigationAction, NavigationGoal
from pymongo import MongoClient 

client = MongoClient() 

class WebTransmitter():
    """WebData"""

    def __init__(self):
        # self.command_location = None
        # self.status_navigation_server = None
        # self.behavior_finished = None
        # self.RATE = rospy.Rate(10)
        # self.EMOTIONAL_FEEDBACK_CYCLE = 15
        # self.behavior_goal = None
        # self.navigation_order = None
        # self.state_name = ""
        # self.behavior_name = ""
        # self.playback = ""
        # self.current_emotional_state = "neutral"
        # self.visual_mode = ""
        # self.utterance  = ""
        # self.is_state_dynamic = False
        # self.completion_trigger = ""
        # self.behavior_duration = ""
        # self.behavior_timeout = "" # defaults to 600s
        # self.emotional_feedback = None

        #/cyborg_controller/state_change

        self.subscriber_smach_container_init = rospy.Subscriber("/controller_viewer/smach/container_init", String, callback = self.print_data, queue_size = 10)
        # self.subscriber_smach_container_status = rospy.Subscriber("/controller_viewer/smach/container_status", String, callback = self.callback_playback, queue_size = 10)
        # self.subscriber_smach_container_structure = rospy.Subscriber("/controller_viewer/smach/container_structure", String, callback = self.callback_playback, queue_size = 10)
        # self.subscriber_audio_feedback_playback = rospy.Subscriber("/cyborg_audio/feedback_playback", String, callback = self.callback_playback, queue_size = 10)
        # self.subscriber_audio_feedback_text_to_speech = rospy.Subscriber("/cyborg_audio/feedback_text_to_speech", String, callback = self.callback_playback, queue_size = 10)
        # self.subscriber_audio_playback = rospy.Subscriber("/cyborg_audio/playback", String, callback = self.print_data, queue_size = 10)
        # self.subscriber_audio_text_to_speech = rospy.Subscriber("/cyborg_audio/text_to_speech", String, callback = self.callback_playback, queue_size = 10)
        
        #self.subscriber_behaviour_cancel = rospy.Subscriber("/cyborg_behavior/cancel", String, callback = self.print_data, queue_size = 10)
        #self.subscriber_behaviour_command_location = rospy.Subscriber("/cyborg_behavior/command_location", String, callback = self.print_data, queue_size = 10)
        #self.subscriber_behaviour_dynamic_behavior = rospy.Subscriber("/cyborg_behavior/dynamic_behavior", String, callback = self.print_data, queue_size = 10)
        #self.subscriber_behaviour_feedback = rospy.Subscriber("/cyborg_behavior/feedback", String, callback = self.print_data, queue_size = 10)
        #self.subscriber_behaviour_goal = rospy.Subscriber("/cyborg_behavior/goal", String, callback = self.print_data, queue_size = 10)
        #self.subscriber_behaviour_result = rospy.Subscriber("/cyborg_behavior/result", String, callback = self.print_data, queue_size = 10)
        #self.subscriber_behaviour_status = rospy.Subscriber("/cyborg_behavior/status", String, callback = self.print_data, queue_size = 10)
        
        # self.subscriber_controller_emotional_controller = rospy.Subscriber("/cyborg_controller/emotional_controller", String, callback = self.callback_playback, queue_size = 10)
        # self.subscriber_controller_emotional_feedback = rospy.Subscriber("/cyborg_controller/emotional_feedback", String, callback = self.callback_playback, queue_size = 10)
        # self.subscriber_controller_emotional_state = rospy.Subscriber("/cyborg_controller/emotional_state", String, callback = self.callback_playback, queue_size = 10)
        # self.subscriber_controller_register_event = rospy.Subscriber("/cyborg_controller/register_event", String, callback = self.callback_playback, queue_size = 10)
        # self.subscriber_controller_set_emotional_state = rospy.Subscriber("/cyborg_controller/set_emotional_state", String, callback = self.callback_playback, queue_size = 10)
        # self.subscriber_controller_set_emotional_values = rospy.Subscriber("/cyborg_controller/set_emotional_values", String, callback = self.callback_playback, queue_size = 10)
        # self.subscriber_controller_state_change = rospy.Subscriber("/cyborg_controller/state_change", SystemState, callback = self.print_data, queue_size = 10)
        # self.subscriber_primarystates_cancel = rospy.Subscriber("/cyborg_primary_states/cancel", String, callback = self.callback_playback, queue_size = 10)
        # self.subscriber_primarystates_feedback = rospy.Subscriber("/cyborg_primary_states/feedback", String, callback = self.callback_playback, queue_size = 10)
        #FUNKER self.subscriber_primarystates_goal = rospy.Subscriber("/cyborg_primary_states/goal", StateMachineActionGoal, callback = self.print_data, queue_size = 10)
        # self.subscriber_primarystates_result = rospy.Subscriber("/cyborg_primary_states/result", String, callback = self.store("primary_states_result"), queue_size = 10)
        # self.subscriber_primarystates_status = rospy.Subscriber("/cyborg_primary_states/status", String, callback = self.callback_playback, queue_size = 10)
        # self.subscriber_visual_domecontrol = rospy.Subscriber("/cyborg_visual/domecontrol", String, callback = self.callback_playback, queue_size = 10)
        # self.subscriber_rosout = rospy.Subscriber("/rosout", String, callback = self.print_data, queue_size = 10)
        
        #FUNKER self.subscriber_rosout_agg = rospy.Subscriber("/rosout_agg", Log, callback = self.print_data, queue_size = 10)
        
        # self.subscriber_unnamed_register_event = rospy.Subscriber("/unnamed/register_event", String, callback = self.callback_playback, queue_size = 10)
        # self.subscriber_unnamed_state_change = rospy.Subscriber("/unnamed/state_change", String, callback = self.callback_playback, queue_size = 10)
        
        # self.publisher_audio_playback = rospy.Publisher("/cyborg_audio/playback", String, queue_size = 10)
        # self.subscriber_audio_playback = rospy.Subscriber("/cyborg_audio/feedback_playback", String, callback = self.callback_playback, queue_size = 10)
        # self.publisher_text_to_speech = rospy.Publisher("/cyborg_audio/text_to_speech", String, queue_size = 10)
        # self.subscriber_text_to_speech = rospy.Subscriber("/cyborg_audio/feedback_text_to_speech", String, callback = self.callback_text_to_speech, queue_size = 10)
        # self.publisher_visual = rospy.Publisher("/cyborg_visual" + "/domecontrol", String, queue_size = 10)
        # self.subscriber_callback_dynamic_behavior = rospy.Subscriber(rospy.get_name() + "/dynamic_behavior", String, callback = self.callback_dynamic_behavior, queue_size= 10)
        # self.subscriber_command_location = rospy.Subscriber(rospy.get_name() + "/command_location", String, callback = self.callback_command_location, queue_size = 10)
        # self.subscriber_emotional_state = rospy.Subscriber("/cyborg_controller/emotional_state", EmotionalState, self.emotional_callback, queue_size= 10)
        # self.publisher_emotional_feedback = rospy.Publisher("/cyborg_controller/emotional_feedback", EmotionalFeedback, queue_size=100)
        # self.server_behavior = actionlib.SimpleActionServer( rospy.get_name() , StateMachineAction, execute_cb=self.server_behavior_callback, auto_start= False)
        # self.server_behavior.start()
        # rospy.loginfo("BehaviorServer: Activated" )


    def print_data(self, message):
        print(message.data)
        # y = yaml.load(str(message))
        # rec = json.dumps(y,indent=4)

        # client = MongoClient("mongodb+srv://cyborg:hmPHK#4.iunGKD2@cyborg-gui-mg1nk.azure.mongodb.net/test?retryWrites=true&w=majority")
        # # Access database 
        # web_database = client['cyborg_data'] 

        # # Access collection of the database 
        # web_database_collection = web_database['primarystates_goal'] 

        # # inserting the data in the database 
        # x = web_database_collection.insert(rec) 
        
        #if message.data != None:
            #self.playback = message.data
            # print(message.data + "MESSAGE DATA HERE ----------------") 

