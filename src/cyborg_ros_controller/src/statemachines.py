#!/usr/bin/env python
##!/usr/bin/python
__author__ = "Areg Babayan"
__version__= "0.0.2"

import rospy
import smach
import smach_ros
from smach import Sequence
from smach import StateMachine
from module import Module

sm_remapping = {"input_events":"state_machine_events",
            "output_events":"state_machine_events",
            "previous_state":"last_state",
            "current_state":"last_state",
            "previous_event":"last_event",
            "current_event":"last_event"}

sequence_navigation_go_to_emotional = Sequence(
                            outcomes = ['succeeded','aborted','preempted'],
                            input_keys=["state_machine_events","last_state","last_event"],
                            output_keys=["state_machine_events","last_state","last_event"],
                            connector_outcome = 'succeeded')


with sequence_navigation_go_to_emotional:
    sequence_navigation_go_to_transitions = {"succeeded":"succeeded", "aborted":"aborted", "preempted":"preempted"}
    sequence_navigation_go_to_resources = {}
    Sequence.add("transport",
                Module("transport", "cyborg_behavior", sequence_navigation_go_to_transitions, sequence_navigation_go_to_resources),
                remapping= sm_remapping)
    Sequence.add("arrival",
                Module("arrival","cyborg_behavior", sequence_navigation_go_to_transitions, sequence_navigation_go_to_resources),
                remapping=sm_remapping)
    Sequence.add("show_off",
                Module("show_off", "cyborg_behavior", sequence_navigation_go_to_transitions, sequence_navigation_go_to_resources),
                remapping=sm_remapping)
