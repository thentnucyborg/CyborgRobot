#!/usr/bin/env python
import npyscreen
import time
import roslib 
import rospy
import actionlib
from std_msgs.msg import String
from cyborg_controller.msg import StateMachineAction, StateMachineGoal, StateMachineResult, StateMachineFeedback, EmotionalState, EmotionalFeedback, SystemState

""" This module contains the cyborg command line form
"""

__author__ = "Thomas Rostrup Andersen"
__copyright__ = "Created by Thomas Rostrup Andersen on 11/01/2017. Copyright (C) 2016 Thomas Rostrup Andersen. All rights reserved."
#__license__ = ""
__version__ = "0.0.2"
__all__ = []


class CommandForm(npyscreen.FormWithMenus):

    """
    Main form window
    """
    keypress_timeout = 1
    state_timer = 0

    def create(self):

        self.add_handlers({
                          "^E": self.execute
                          })

        self.value = None
        self.name = "The NTNU Cyborg"

        self.previous_state = self.add(npyscreen.TitleFixedText, name="Previous State: ", value="", use_two_lines=False)
        self.previous_state.editable = False

        self.event = self.add(npyscreen.TitleFixedText, name="Event: ", value="", use_two_lines=False)
        self.event.editable = False

        self.current_state = self.add(npyscreen.TitleFixedText, name="Current State: ", value="", use_two_lines=False)
        self.current_state.editable = False

        self.timer = self.add(npyscreen.TitleFixedText, name="Timer(s): ", value="", use_two_lines=False)
        self.timer.editable = False

        self.nextrely += 1
        self.border = self.add(npyscreen.FixedText, name="", value="-----------------------------------------------------------------------------", use_two_lines=False)
        self.border.editable = False
        self.nextrely += 1

        self.emotional_state = self.add(npyscreen.TitleFixedText, name="Emotional State: ", value="", use_two_lines=False)
        self.emotional_state.editable = False

        self.emotional_pleasure = self.add(npyscreen.TitleFixedText, name="Pleasure Value: ", value="", use_two_lines=False)
        self.emotional_pleasure.editable = False

        self.emotional_arousal = self.add(npyscreen.TitleFixedText, name="Arousal Value: ", value="", use_two_lines=False)
        self.emotional_arousal.editable = False

        self.emotional_dominance = self.add(npyscreen.TitleFixedText, name="Dominance Value: ", value="", use_two_lines=False)
        self.emotional_dominance.editable = False

        self.nextrely += 1
        self.border = self.add(npyscreen.FixedText, name="", value="-----------------------------------------------------------------------------", use_two_lines=False)
        self.border.editable = False
        self.nextrely += 1

        self.action = self.add(npyscreen.TitleText, name="Action: ", value="", use_two_lines=False)
        self.command = self.add(npyscreen.TitleText, name="Command: ", value="", use_two_lines=False)

        self.nextrely += 1
        self.border = self.add(npyscreen.FixedText, name="", value="-----------------------------------------------------------------------------", use_two_lines=False)
        self.border.editable = False
        self.nextrely += 1

        self.emotion_subscriber = rospy.Subscriber("/cyborg_controller/emotional_state", EmotionalState, self.emotion_callback, queue_size=100)
        self.state_subscriber = rospy.Subscriber("/cyborg_controller/state_change", SystemState, self.state_callback, queue_size=100)

        self.speech_publisher = rospy.Publisher("/text_from_speech", String, queue_size=100)
        self.event_publisher = rospy.Publisher("/cyborg_controller/register_event", String, queue_size=100)
        self.emotion_publisher = rospy.Publisher("/cyborg_controller/set_emotional_state", String, queue_size=100)
        self.emotion_controller_publisher = rospy.Publisher("/cyborg_controller/emotional_controler", String, queue_size=100)

        self.menu = self.new_menu(name="Menu", shortcut='^X')
        self.menu.addItem(text="Execute action and command", shortcut="^E", onSelect=self.execute)
        self.menu.addItem(text="Close Menu", shortcut="q")
        self.menu.addItem(text="Exit", shortcut="^Q", onSelect=self.afterEditing)

    
    def beforeEditing(self):
        self.state_timer = time.time()
        pass


    def on_ok(self):
        self.parentApp.switchForm(None)

    def afterEditing(self):
        self.parentApp.switchForm(None)

                    
    def on_cancel(self):
        pass


    def while_waiting(self):
        self.timer.value = str(time.time() - self.state_timer)
        self.display()


    def while_editing(self, *args, **kwargs):
        pass


    def execute(self, *args, **keywords):
        if self.action.value == "s":
            self.speech_publisher.publish(self.command.value)
        elif self.action.value == "e":
            self.event_publisher.publish(self.command.value)
        elif self.action.value == "m":
            self.emotion_publisher.publish(self.command.value)
        elif self.action.value == "c":
            self.emotion_controller_publisher.publish(self.command.value)
        else:
            message_to_display = "Available actions are: \n event(e) \n speech(s) \n emotion(m) \n emotion_controller(c) "
            npyscreen.notify_confirm(message_to_display, title= "Action not recognitzed! ")


    # Updates the current emotion when the emotion subscriber recives data
    def emotion_callback(self, data):
        self.emotional_state.value = str(data.to_emotional_state)
        self.emotional_pleasure.value = str(data.current_pleasure)
        self.emotional_arousal.value = str(data.current_arousal)
        self.emotional_dominance.value = str(data.current_dominance)
        self.display()


    # Updates the current system state when the system state subscriber recieves data
    def state_callback(self, data):
        self.previous_state.value = data.from_system_state
        self.event.value = data.event
        self.current_state.value = data.to_system_state
        self.state_timer = time.time()
        self.timer.value = str(0)
        self.display()

