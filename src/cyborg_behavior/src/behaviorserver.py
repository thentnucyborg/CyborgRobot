#!/usr/bin/env python

__author__ = "Areg Babayan"
__copyright__ = "Copyright (C) 2019 Areg Babayan"
__license__ = "BSD"
__version__ = "0.0.6"
__all__ = []


import rospy
import time
import actionlib
from std_msgs.msg import String
from cyborg_controller.msg import StateMachineAction, StateMachineGoal, EmotionalState, EmotionalFeedback
from cyborg_navigation.msg import NavigationAction, NavigationGoal


class BehaviorServer():
    """BehaviorServer"""

    def __init__(self):
        self.command_location = None
        self.status_navigation_server = None
        self.behavior_finished = None
        self.RATE = rospy.Rate(10)
        self.EMOTIONAL_FEEDBACK_CYCLE = 15
        self.behavior_goal = None
        self.navigation_order = None
        self.state_name = ""
        self.behavior_name = ""
        self.playback = ""
        self.current_emotional_state = "neutral"
        self.visual_mode = ""
        self.utterance  = ""
        self.is_state_dynamic = False
        self.completion_trigger = ""
        self.behavior_duration = ""
        self.behavior_timeout = "" # defaults to 600s
        self.emotional_feedback = None

        self.publisher_audio_playback = rospy.Publisher("/cyborg_audio/playback", String, queue_size = 10)
        self.subscriber_audio_playback = rospy.Subscriber("/cyborg_audio/feedback_playback", String, callback = self.callback_playback, queue_size = 10)
        self.publisher_text_to_speech = rospy.Publisher("/cyborg_audio/text_to_speech", String, queue_size = 10)
        self.subscriber_text_to_speech = rospy.Subscriber("/cyborg_audio/feedback_text_to_speech", String, callback = self.callback_text_to_speech, queue_size = 10)
        self.publisher_visual = rospy.Publisher("/cyborg_visual" + "/domecontrol", String, queue_size = 10)
        self.subscriber_callback_dynamic_behavior = rospy.Subscriber(rospy.get_name() + "/dynamic_behavior", String, callback = self.callback_dynamic_behavior, queue_size= 10)
        self.subscriber_command_location = rospy.Subscriber(rospy.get_name() + "/command_location", String, callback = self.callback_command_location, queue_size = 10)
        self.subscriber_emotional_state = rospy.Subscriber("/cyborg_controller/emotional_state", EmotionalState, self.emotional_callback, queue_size= 10)
        self.publisher_emotional_feedback = rospy.Publisher("/cyborg_controller/emotional_feedback", EmotionalFeedback, queue_size=100)
        self.server_behavior = actionlib.SimpleActionServer( rospy.get_name() , StateMachineAction, execute_cb=self.server_behavior_callback, auto_start= False)
        self.server_behavior.start()
        rospy.loginfo("BehaviorServer: Activated" )


    def emotional_callback(self, message):
        self.current_emotional_state = message.to_emotional_state

    def callback_command_location(self, message):
        if message.data != None:
            self.command_location = message.data



    def server_behavior_callback(self, goal):
        self.behavior_goal = goal
        rospy.loginfo("Behavior server: GOAL IS: " + str(self.behavior_goal))

        # check if preset exists and execute, else abort
        if rospy.has_param(rospy.get_name() +"/"+ self.behavior_goal.current_state + "_" + self.current_emotional_state):
            self.state_name = self.behavior_goal.current_state + "_" + self.current_emotional_state
            self.behavior_finished = False 
            self.enter()
            self.execute_behavior()
        elif rospy.has_param(rospy.get_name() +"/"+ self.behavior_goal.current_state):
            self.state_name = self.behavior_goal.current_state
            self.behavior_finished = False
            self.enter()
            self.execute_behavior()
        else:
            rospy.loginfo("ERROR: BehaviorServer: Parameters for '"+ self.behavior_goal.current_state +"' behavior and '" + self.current_emotional_state + "' mood could not be loaded." )
            self.server_behavior.set_aborted()
            return
            
    
    def enter(self):
        # get behavior parameters from parameter server
        self.behavior_name = self.state_name
        # Get parameter if set, else set default value
        self.completion_trigger = rospy.get_param( rospy.get_name() + "/" + self.state_name + "/completion_trigger", "")
        self.playback = rospy.get_param(rospy.get_name() + "/" + self.state_name + "/playback", "")
        self.visual_mode = rospy.get_param(rospy.get_name() + "/" + self.state_name + "/visual_mode", "")
        self.utterance = rospy.get_param(rospy.get_name() + "/" + self.state_name + "/utterance", "")
        self.is_state_dynamic = rospy.get_param( rospy.get_name() + "/" + self.state_name + "/dynamic", False)
        self.navigation_order = rospy.get_param( rospy.get_name() + "/" + self.state_name + "/navigation_order", None)
        rospy.loginfo("navigation_order is: " + str(self.navigation_order))

        if self.navigation_order == "navigation_go_to":
            if rospy.has_param( rospy.get_name() + "/" + self.state_name + "/location"):
                self.command_location = rospy.get_param( rospy.get_name() + "/" + self.state_name + "/location", None)
        self.behavior_timeout = rospy.get_param( rospy.get_name() + "/" + self.state_name + "/timeout", 300)
        self.emotional_feedback = rospy.get_param(rospy.get_name() + "/" + self.state_name + "/emotional_feedback" , None)

        if self.navigation_order is not None:
            self.client_navigation = actionlib.SimpleActionClient('/cyborg_navigation/navigation', NavigationAction)
            if not self.client_navigation.wait_for_server(rospy.Duration(2.0)):
                rospy.logwarn("ERROR: Behaviorserver unable to connect to navigation server.")
                return
            if self.command_location is None and (self.behavior_goal.order not in ("EXECUTE","CANCEL", None)):
                self.command_location = self.behavior_goal.order

        # Set duration if time trigger
        if "time" in self.completion_trigger:
            _, _, self.behavior_duration = self.completion_trigger.partition(' ')
            self.behavior_duration = float(self.behavior_duration)

        rospy.loginfo("BehaviorServer: Parameters loaded: playback = %s, visual_mode = %s, utterance = %s, completion_trigger = %s, navigation_order = %s. ",
                    self.playback, self.visual_mode, self.utterance, self.completion_trigger, str(self.navigation_order))

    

    def execute_behavior(self):
        rospy.loginfo("BehaviorServer: Executing "+ self.state_name + " behavior")
        #check and send navigational order
        if self.navigation_order is not None:
            self.status_navigation_server = -1
            goal_navigation = NavigationGoal(order=self.navigation_order)
            goal_navigation.location_name = self.command_location if self.command_location != None else ""
            rospy.loginfo("BehaviorServer: goal_navigation is: " + str(goal_navigation))
            rospy.loginfo("BehaviorServer: sending navigation order '" + self.navigation_order +"' and navigation goal '" + str(goal_navigation.location_name) +"' to navigation client.")
            self.client_navigation.send_goal(goal_navigation, done_cb = self.callback_navigation_done)
        #check and send visual command
        if self.visual_mode != "":
            message_visual = String(data=self.visual_mode)
            self.publisher_visual.publish(message_visual)
        #check and send audio command
        if self.playback != "":
            message_audio = String(data = self.playback)
            self.publisher_audio_playback.publish(message_audio)
        #check and send utterance
        elif self.utterance != "":
            message_audio = String(data= self.utterance)
            self.publisher_text_to_speech.publish(message_audio)
        #give more details in loginfo
        rospy.loginfo("BehaviorServer: Behavior configurations executing.")
        rospy.loginfo("BehaviorServer: emotional_feedback is: " + str(self.emotional_feedback))
        if self.emotional_feedback != None:
            self.send_emotion(self.emotional_feedback['p'], self.emotional_feedback['a'], self.emotional_feedback['d'])
        start_time = time.time()
        feedback_time = time.time()
        while not rospy.is_shutdown():
            #rospy.loginfo("loop")
            #check if trigger is met, aka goal completed       
            if self.behavior_finished is True:
                if self.completion_trigger == "navigation":
                    if self.status_navigation_server == 3: #succeeded
                        rospy.loginfo("BehaviorServer: Succeeded." )
                        self.send_emotion(pleasure=0.01, arousal=0.01, dominance=0.01)
                        self.server_behavior.set_succeeded()
                        return
                    else:
                        rospy.loginfo("BehaviorServer: Navigation goal failed to succeed.")
                        self.send_emotion(pleasure=-0.01, arousal=0, dominance=-0.01)                        
                        self.server_behavior.set_aborted()
                        return  
                        # if aborted, rejected, recalled or lost
                if (self.status_navigation_server in [4, 5, 8, 9]):
                    if self.playback != "":
                        self.publisher_audio_playback.publish("PreemptPlayback")
                    if self.utterance !="":
                        self.publisher_text_to_speech.publish("PreemptUtterance")
                    rospy.loginfo("BehaviorServer: aborted.")
                    self.send_emotion(pleasure=0, arousal=-0.3, dominance=0.0)
                    self.server_behavior.set_aborted()
                    return
                else:
                    if self.navigation_order != None:
                        self.client_navigation.cancel_all_goals()
                        rospy.sleep(1)        
                    rospy.loginfo("BehaviorServer: Succeeded")
                    self.send_emotion(pleasure=0.01, arousal=0.01, dominance=0.01)
                    self.server_behavior.set_succeeded()
                    return 
            #check for preempt request, and check executions of behavior and timer 
            if self.server_behavior.is_preempt_requested():
                rospy.loginfo("BehaviorServer preempt requested: "+ self.state_name + " behavior preempted." )
                if self.navigation_order != None:
                    self.client_navigation.cancel_all_goals()
                    self.send_emotion(pleasure=0, arousal=0, dominance=0.0)
                    if self.playback != "":
                        self.publisher_audio_playback.publish("PreemptPlayback")
                    if self.utterance !="":
                        self.publisher_text_to_speech.publish("PreemptUtterance")
                self.server_behavior.set_preempted()
                return
            # check  behavior_timeout to prevent eternal looping
            if self.behavior_timeout is not False:
                if ((time.time() - start_time) > self.behavior_timeout):
                    #stop execution
                    rospy.loginfo("BehaviorServer: Preset " + self.state_name + " timed out, behavior aborted.")
                    if self.navigation_order != None:
                        self.client_navigation.cancel_all_goals()
                    self.server_behavior.set_aborted()
                    return
            # Check behavior duration if time trigger
            if "time" in self.completion_trigger:
                if ((time.time() - start_time) > self.behavior_duration):
                    self.behavior_finished = True
                    rospy.loginfo("BehaviorServer: Completion trigger " + self.completion_trigger +" has been met.")
            # provide emotional feedback if continuous
            if self.emotional_feedback is not None:
                if (time.time() - feedback_time > self.EMOTIONAL_FEEDBACK_CYCLE) and "continuous" in self.emotional_feedback:
                    self.send_emotion(self.emotional_feedback['p'], self.emotional_feedback['a'], self.emotional_feedback['d'])
                    feedback_time = time.time()
            self.RATE.sleep()
        # set terminal goal status in case of shutdown
        rospy.loginfo("Aborted...")
        self.server_behavior.set_aborted()

        
    def callback_dynamic_behavior(self, data):
        if self.is_state_dynamic:
            #handle messages from motivator/emotional interpreter, or other modules
            data = data.data
            mode, _, command = data.partition(' ') #partition mode and command
            message  = String(data = command)
            if mode =="playback":
                self.publisher_audio_playback.publish(message)
            elif mode =="text_to_speech":
                self.publisher_text_to_speech.publish(message)
            elif mode =="visual":
                self.publisher_visual.publish(message)
            elif mode =="behavior":
                self.change_behavior(command)
            else:
                #error handling
                rospy.loginfo("BehaviorServer: audiovisual_callback received illegal command: %s" % command)
        else: 
            rospy.loginfo("BehaviorServer: behavior change requested, but state is not dynamic.")



    # Called when behavior goal is completed
    def callback_navigation_done(self, status, result):
        rospy.loginfo("BehaviorServer entered callback_navigation_done, with status: " + str(status) + " and result: " + str(result))

        if self.behavior_finished != True:
            self.status_navigation_server = status
            if self.completion_trigger == "navigation" and self.status_navigation_server==3:
                self.behavior_finished = True
                self.command_location = None
                rospy.loginfo("BehaviorServer: Completion trigger " + self.completion_trigger +" has been met.")
                rospy.loginfo("BehaviorServer: Navigation finished order " + str(self.navigation_order))
            else:
                self.behavior_finished = True
                self.command_location = None
                rospy.loginfo("BehaviorServer: Navigation Action aborted.")


    def change_behavior(self, behavior):
        requested_behavior = behavior
        if rospy.has_param(rospy.get_name() + "/" + requested_behavior):
            self.behavior_name = requested_behavior
            self.playback = rospy.get_param(rospy.get_name() + "/" + self.behavior_name + "/playback", "")
            self.visual_mode = rospy.get_param(rospy.get_name() + "/" + self.behavior_name + "/visual_mode", "")
            self.utterance = rospy.get_param(rospy.get_name() + "/" + self.behavior_name + "/utterance", "")
            self.emotional_feedback = rospy.get_param(rospy.get_name() + "/" + self.state_name + "/emotional_feedback" , None)
            rospy.loginfo("BehaviorServer: changing behavior to %s.", self.behavior_name)
            if self.emotional_feedback != None: 
                self.send_emotion(self.emotional_feedback['p'], self.emotional_feedback['a'], self.emotional_feedback['d'])

            if self.visual_mode !="":
                message_visual = String(data  =self.visual_mode)
                self.publisher_visual.publish(message_visual)

            if self.playback !="":
                message_audio = String(data = self.playback)
                self.publisher_audio_playback.publish(message_audio)
            elif self.utterance !="":
                message_audio = String(data = self.utterance)
                self.publisher_text_to_speech.publish(message_audio)
        else:
            rospy.loginfo("BehaviorServer: change behavior could not load request behavior: %s.", requested_behavior)


    def callback_playback(self, message):
        if message.data == "finished":
            if self.completion_trigger == "playback":
                self.behavior_finished = True
                rospy.loginfo("BehaviorServer: Completion trigger " + self.completion_trigger + " has been met.")
        else:
            rospy.loginfo("BehaviorServer: " + "playback has " + str(message))


    def callback_text_to_speech(self, message):
        if message.data == "finished":
            if self.completion_trigger == "utterance":
                self.behavior_finished = True
                rospy.loginfo("BehaviorServer: Completion trigger " + self.completion_trigger + " has been met")
        else:
            rospy.loginfo("BehaviorServer: " + "playback has " + str(message))

    # Sends the emotional change to the controller
    def send_emotion(self, pleasure, arousal, dominance):
        msg = EmotionalFeedback()
        msg.delta_pleasure = pleasure
        msg.delta_arousal = arousal
        msg.delta_dominance = dominance
        self.publisher_emotional_feedback.publish(msg)
