#!/usr/bin/env python
##!/usr/bin/python

"""Created by Thomas Rostrup Andersen on 11/11/2016.
Copyright (C) 2016 Thomas Rostrup Andersen. All rights reserved."""

"""Modified by Areg Babayan spring 2019"""

__author__ = "Thomas Rostrup Andersen"
__copyright__ = "Copyright (C) 2016 Thomas Rostrup Andersen"
#__license__ = ""
__version__ = "0.0.3"
__all__ = []

import os
import sys
import threading

import rospy
import smach
import smach_ros
from smach import Sequence
from smach import StateMachine

import statemachines

from databasehandler import DatabaseHandler
from emotionsystem import EmotionSystem
from module import Module
from motivator import Motivator

def main():
    rospy.init_node("cyborg_controller")

    # Create emotions
    emotion_system = EmotionSystem()
    emotion_system.add_emotion(name="angry", pleasure=-0.51, arousal=0.59, dominance=0.25)
    emotion_system.add_emotion(name="bored", pleasure=-0.65, arousal=-0.62, dominance=-0.33)
    emotion_system.add_emotion(name="curious", pleasure=0.22, arousal=0.62, dominance=-0.10)
    emotion_system.add_emotion(name="dignified", pleasure=0.55, arousal=0.22, dominance=0.61)
    emotion_system.add_emotion(name="elated", pleasure=0.50, arousal=0.42, dominance=0.23) # Happy
    emotion_system.add_emotion(name="inhibited", pleasure=-0.54, arousal=-0.04, dominance=-0.41) # Sadness
    emotion_system.add_emotion(name="puzzled", pleasure=-0.41, arousal=0.48, dominance=-0.33) # Surprized
    emotion_system.add_emotion(name="loved", pleasure=0.89, arousal=0.54, dominance=-0.18)
    emotion_system.add_emotion(name="unconcerned", pleasure=-0.13, arousal=-0.41, dominance=0.08)

    homedir = os.path.expanduser("~")
    path = homedir + "/controller.db"

    # Fill database with default values
    if (os.path.exists(path) == False):
        event_cost = 0.6 #
        database_handler = DatabaseHandler(filename=path)
        database_handler.create()
        database_handler.add_event(state="idle", event="music_play", reward_pleasure=0.08, reward_arousal=0.02, reward_dominance=-0.03, event_cost=event_cost*2)
        database_handler.add_event(state="idle", event="astrolanguage", reward_pleasure=0.04, reward_arousal=0.05, reward_dominance=0.04, event_cost=event_cost*1.5)
        database_handler.add_event(state="idle", event="navigation_emotional", reward_pleasure=0.02, reward_arousal=0.04, reward_dominance=-0.02, event_cost=event_cost)
        database_handler.add_event(state="idle", event="show_off_mea", reward_pleasure=0.02, reward_arousal=0.01, reward_dominance=0.01, event_cost=event_cost/2)
        database_handler.add_event(state="idle", event="convey_emotion", reward_pleasure=0.01, reward_arousal=-0.01, reward_dominance=0.02, event_cost= event_cost/3) 

    # Create motivator
    motivator = Motivator(database_file=path)
    motivator.start()

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=["error"])

    sm.userdata.state_machine_events = []
    sm.userdata.last_state = "initializing" 
    sm.userdata.last_event = "start_up"

    with sm:
        sm_remapping = {"input_events":"state_machine_events",
            "output_events":"state_machine_events",
            "previous_state":"last_state",
            "current_state":"last_state",
            "previous_event":"last_event",
            "current_event":"last_event"}


        idle_transitions = {"navigation_schedular":"navigation",
                            "navigation_emotional":"navigation",
                            "convey_emotion":"conveying_emotional_state",
                            "show_off_mea":"show_off_mea",
                            "music_play":"music_horror",
                            "astrolanguage":"astrolanguage",
                            "aborted":"idle",
                            "power_low":"exhausted",
                            "bedtime":"sleepy"}
        idle_resources = {} # Idle does not require any resources

        smach.StateMachine.add("idle",
                        Module("idle", "cyborg_primary_states", idle_transitions, idle_resources),
                        idle_transitions,
                        sm_remapping)

        sleepy_transitions = {"aborted":"idle",
                            "succeeded":"sleeping"}
        sleepy_resources = {}
        show_off_mea_transitions={"aborted":"idle",
                                "succeeded":"idle"}
        smach.StateMachine.add("show_off_mea",
                            Module("show_off_mea", "cyborg_behavior",show_off_mea_transitions, sleepy_resources),
                            show_off_mea_transitions,
                            sm_remapping)
                            
        smach.StateMachine.add("sleepy",
                                Module("sleepy", "cyborg_behavior", sleepy_transitions, sleepy_resources),
                                sleepy_transitions,
                                sm_remapping)


        exhausted_transitions = {"aborted":"idle",
                            "succeeded":"sleeping"}
        exhausted_resources = {}
        smach.StateMachine.add("exhausted",
                                Module("exhausted", "cyborg_behavior", exhausted_transitions, exhausted_resources),
                                exhausted_transitions,
                                sm_remapping)


        sleeping_transitions = {"preempted": "idle",
                                "cyborg_wake_up":"waking_up"}

        sleeping_resources = {}
        smach.StateMachine.add("sleeping",
                                Module("sleeping", "cyborg_primary_states", sleeping_transitions, sleeping_resources),
                                sleeping_transitions,
                                sm_remapping)


        waking_up_transitions = {"aborted":"idle",
                                "succeeded":"idle"}
        waking_up_resources = {}

        smach.StateMachine.add("waking_up",
                                Module("waking_up", "cyborg_behavior", waking_up_transitions, waking_up_resources),
                                waking_up_transitions,
                                sm_remapping)
        
        
        conveying_emotion_transitions={"aborted":"idle",
                                    "succeeded":"idle"}
        conveying_emotion_resources={}

        smach.StateMachine.add("conveying_emotional_state",
                            Module("conveying_emotional_state", "cyborg_behavior", conveying_emotion_transitions, conveying_emotion_resources),
                                conveying_emotion_transitions,
                                sm_remapping)


        sm_nav = smach.StateMachine(outcomes=["succeeded","aborted","power_low"])

        with sm_nav:
        
            navigation_planning_transitions = {"navigation_start_moving_schedular":"navigation_go_to_schedular",
                                                "navigation_start_moving_emotional":"navigation_go_to_emotional",
                                                "navigation_start_wandering":"wandering_emotional",
                                                "aborted":"aborted"}
            navigation_planning_resources = {}

            smach.StateMachine.add("navigation_planning",
                                    Module("navigation_planning", "cyborg_primary_states",navigation_planning_transitions, navigation_planning_resources),
                                    navigation_planning_transitions,
                                    sm_remapping)

            navigation_go_to_transitions = {"succeeded":"succeeded", "aborted":"aborted", "preempted":"aborted"}

            smach.StateMachine.add("navigation_go_to_emotional", statemachines.sequence_navigation_go_to_emotional,
                            transitions = navigation_go_to_transitions,
                            remapping = sm_remapping)

            navigation_go_to_schedular_resources = {}
            smach.StateMachine.add("navigation_go_to_schedular",
                                    Module("navigation_go_to_schedular", "cyborg_behavior", navigation_go_to_transitions, navigation_go_to_schedular_resources),
                                    navigation_go_to_transitions,
                                    sm_remapping)


            navigation_wandering_emotional_transitions = {"navigation_wandering_complete":"succeeded",
                                                "aborted":"aborted",
                                                "power_low":"power_low"}
            navigation_wandering_emotional_resources  = {}

            smach.StateMachine.add("wandering_emotional",
                                    Module("wandering_emotional", "cyborg_primary_states", navigation_wandering_emotional_transitions, navigation_wandering_emotional_resources),
                                    navigation_wandering_emotional_transitions,
                                    sm_remapping)

        sm_nav_transitions = {"aborted":"idle","succeeded":"idle", "power_low":"exhausted"}
        smach.StateMachine.add("navigation", sm_nav, sm_nav_transitions, sm_remapping)

        music_transitions = {"aborted":"idle",
                                "succeeded":"idle"}
        music_resources = {}
        smach.StateMachine.add("music_horror",
                                Module("music_horror","cyborg_behavior", music_transitions, music_resources),
                                    music_transitions,
                                    sm_remapping)
        
        astrolanguage_transitions = {"aborted":"idle",
                                "succeeded":"idle"}
        astrolanguage_resources = {}
        smach.StateMachine.add("astrolanguage",
                                Module("astrolanguage","cyborg_behavior", astrolanguage_transitions, astrolanguage_resources),
                                    astrolanguage_transitions,
                                    sm_remapping)


        sm.register_io_keys({"state_machine_events","last_state","last_event"})
        sm_nav.register_io_keys({"state_machine_events","last_state","last_event"})


    rospy.sleep(3)
    sis = smach_ros.IntrospectionServer('controller_viewer', sm, '/controller_viewer')
    sis.start()

    smach_thread = threading.Thread(target=sm.execute)
    smach_thread.daemon = True
    smach_thread.start()
    # Start ROS main looping
    rospy.loginfo("Controller: Activated...")
    rospy.spin()
    sis.stop()
    rospy.loginfo("Controller: Terminated...")


if __name__ == "__main__":
    print("Cyborg Controller: Starting Program...")

    if sys.version_info < (2,5):
        print("Cyborg Controller: Running Python version " + str(sys.version_info.major) + "." + str(sys.version_info.minor) + "." + str(sys.version_info.micro) + " (Python version 2.5 or grater is required)...")
        exit()

    main()

    print("Cyborg Controller: End of Program...")
