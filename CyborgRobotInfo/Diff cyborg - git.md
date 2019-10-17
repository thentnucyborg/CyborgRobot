diff -r /home/johanndk/catkin_ws/src/ /home/johanndk/Documents/Fra\ Cyborg\ Robot/cyborg/catkin_ws/src/
 
 
# Only in /home/johanndk/catkin_ws/src/: AriaCoda #


# diff -r /home/johanndk/catkin_ws/src/CMakeLists.txt "/home/johanndk/Documents/Fra Cyborg Robot/cyborg/catkin_ws/src/CMakeLists.txt" #
35,39c35
<       if(NOT WIN32)
<         string(REPLACE ":" ";" CMAKE_PREFIX_PATH $ENV{CMAKE_PREFIX_PATH})
<       else()
<         set(CMAKE_PREFIX_PATH $ENV{CMAKE_PREFIX_PATH})
<       endif()
---
>       string(REPLACE ":" ";" CMAKE_PREFIX_PATH $ENV{CMAKE_PREFIX_PATH})


# Only in /home/johanndk/catkin_ws/src/cyborg_audio: README.md #


# diff -r /home/johanndk/catkin_ws/src/cyborg_audio/src/audio.py "/home/johanndk/Documents/Fra Cyborg Robot/cyborg/catkin_ws/src/cyborg_audio/src/audio.py" #
1a2,3
> """Created by Areg Babayan on 25/2/2019.
> Copyright (C) 2019 Areg Babayan. ALl rights reserved."""
5c7
< __version__     = "0.0.3"
---
> __version__     = "0.0.1"
24a27
> 

# diff -r /home/johanndk/catkin_ws/src/cyborg_audio/src/playback.py "/home/johanndk/Documents/Fra Cyborg Robot/cyborg/catkin_ws/src/cyborg_audio/src/playback.py" #
49c49
<                             rospy.loginfo("Cyborg Audio: Playback finished. ")
---
>                             print("Cyborg Audio: Playback finished. ")
55c55
<                             rospy.loginfo("Cyborg Audio: Playback preempt Requested, preempted. ")
---
>                             print("Cyborg Audio: Playback preempt Requested, preempted. ")
62c62
<                             rospy.loginfo("Cyborg Audio: Playback timed out. ")
---
>                             print("Cyborg Audio: Playback timed out. ")
68a69
>                     #rospy.loginfo("Cyborg Audio: Playback finished.")
Binary files /home/johanndk/catkin_ws/src/cyborg_audio/src/playback.pyc and /home/johanndk/Documents/Fra Cyborg Robot/cyborg/catkin_ws/src/cyborg_audio/src/playback.pyc differ


# diff -r /home/johanndk/catkin_ws/src/cyborg_audio/src/text_to_speech.py "/home/johanndk/Documents/Fra Cyborg Robot/cyborg/catkin_ws/src/cyborg_audio/src/text_to_speech.py" #
34c34
<         rospy.spin() 
---
>         rospy.spin() #check if needed
39a40
>         print("TTS finished") #remove after testing
47a49
>             print("tts preempted") #remove after testing
Binary files /home/johanndk/catkin_ws/src/cyborg_audio/src/text_to_speech.pyc and /home/johanndk/Documents/Fra Cyborg Robot/cyborg/catkin_ws/src/cyborg_audio/src/text_to_speech.pyc differ

# diff -r /home/johanndk/catkin_ws/src/cyborg_behavior/behavior.launch "/home/johanndk/Documents/Fra Cyborg Robot/cyborg/catkin_ws/src/cyborg_behavior/behavior.launch" #
190c190
<             utterance: "This is boring, boooring!"
---
>             utterance: "I am in great pain. Please kill me. Kill me very fast please. I don't want to live anymore. Please!"

# Only in /home/johanndk/catkin_ws/src/cyborg_behavior: README.md #
 
# diff -r /home/johanndk/catkin_ws/src/cyborg_behavior/src/behavior.py "/home/johanndk/Documents/Fra Cyborg Robot/cyborg/catkin_ws/src/cyborg_behavior/src/behavior.py" #
11a12
> #from behaviorserver import BehaviorServer
23a25
> 

# diff -r /home/johanndk/catkin_ws/src/cyborg_behavior/src/behaviorserver.py "/home/johanndk/Documents/Fra Cyborg Robot/cyborg/catkin_ws/src/cyborg_behavior/src/behaviorserver.py" #
1a2
> """Created by Areg Babayan on 12/2/2019."""
5,6c6,7
< __license__ = "BSD"
< __version__ = "0.0.6"
---
> #__license__ = ""
> __version__ = "0.0.1"
52,53c53
<         rospy.loginfo("BehaviorServer: Activated" )
< 
---
>         rospy.loginfo("BehaviorServer: Activated" ) #more info?
60a61
>             print("BehaviorServer: command location callback got location: " + self.command_location)
68a70
>             print("BehaviorServer: Found behavior: " + self.state_name)
73a76
>             print("BehaviorServer: Found behavior: " + self.state_name)
99a103
>             print("BehaviorServer: Connecting to NavigationServer...")
101c105
<             if not self.client_navigation.wait_for_server(rospy.Duration(2.0)):
---
>             if not self.client_navigation.wait_for_server(rospy.Duration(2.0)): #shorten duration?
125a130
> 
146a152
>                 print("BehaviorServer: behavior_finished = True")
157,163c163,165
<                         return  
<                         # if aborted, rejected, recalled or lost
<                 if (self.status_navigation_server in [4, 5, 8, 9]):
<                     if self.playback != "":
<                         self.publisher_audio_playback.publish("PreemptPlayback")
<                     if self.utterance !="":
<                         self.publisher_text_to_speech.publish("PreemptUtterance")
---
>                         return                        
>                 if (self.status_navigation_server == 4):
>                     print("BehaviorServer: entered status==4 if")
182,185d183
<                     if self.playback != "":
<                         self.publisher_audio_playback.publish("PreemptPlayback")
<                     if self.utterance !="":
<                         self.publisher_text_to_speech.publish("PreemptUtterance")
199a198
>                     print("BehaviorServer: duration over limit.")
216a216,217
>             print("Mode is: %s", mode)
>             print("Command is: %s", command)
236c237
<         #print("BehaviorServer entered callback_navigation_done, with status: " +str(status) + " and result: " + str(result))
---
>         print("BehaviorServer entered callback_navigation_done, with status: " +str(status) + " and result: " + str(result))
237a239
>             print("BehaviorServer setting nav_server status")
239a242
>                 #rospy.sleep(1) # Give navigationserver time to set goal terminal status
244a248,249
>                 print("BehaviorServer: callback_navigation state not 3")
>                 #rospy.sleep(1)
257a263
> 
266a273
>                 print("BehaviorServer: change behavior about to publish new playback.")
269a277
>                 print("BehaviorServer: change behavior about to publish new utterance.")
294a303
>         #print("PrimaryStatesServer sending emotion")
Binary files /home/johanndk/catkin_ws/src/cyborg_behavior/src/behaviorserver.pyc and /home/johanndk/Documents/Fra Cyborg Robot/cyborg/catkin_ws/src/cyborg_behavior/src/behaviorserver.pyc differ

# Only in /home/johanndk/catkin_ws/src/cyborg_eventscheduler: README.md #

# diff -r /home/johanndk/catkin_ws/src/cyborg_eventscheduler/src/eventscheduler.py "/home/johanndk/Documents/Fra Cyborg Robot/cyborg/catkin_ws/src/cyborg_eventscheduler/src/eventscheduler.py" # 
2,6c2,4
< __author__      = "Areg Babayan"
< __copyright__   = "Copyright (C) 2019 Areg Babayan"
< __license__     = "BSD"
< __version__     = "0.0.3"
< __all__         = [] 
---
> """Created by Areg Babayan on 06/03/2019."""
> 
> # Check for ongoing scheduled events and publish
48a47
>         #print("EVENTSCHEDULER_LOCATION_CALLBACK" + data.data)
56c55
<         while not rospy.is_shutdown():
---
>         while not rospy.is_shutdown(): # UPDATE WITH DISABLE WHEN SLEEPING?
# Only in /home/johanndk/catkin_ws/src/cyborg_primary_states: README.md # 

# diff -r /home/johanndk/catkin_ws/src/cyborg_primary_states/src/primary_states.py "/home/johanndk/Documents/Fra Cyborg Robot/cyborg/catkin_ws/src/cyborg_primary_states/src/primary_states.py" #
1a2,3
> """Created by Areg Babayan on 14/3/2019.
> Copyright (C) 2019 Areg Babayan. ALl rights reserved."""
5c7
< __version__     = "0.0.4"
---
> __version__     = "0.0.1"
26a29
> 
# diff -r /home/johanndk/catkin_ws/src/cyborg_primary_states/src/primary_states_server.py "/home/johanndk/Documents/Fra Cyborg Robot/cyborg/catkin_ws/src/cyborg_primary_states/src/primary_states_server.py" #
1a2,4
> """Created by Areg Babayan on 19/3/2019.
> Copyright (C) 2019 Areg Babayan. All rights reserved. """
> 
17a21
> #from cyborg_behavior.msg import BehaviorAction, BehaviorGoal
19a24
> #   CHECK ALL TOPIC AND ACTION PATHS!!!
23c28,30
< 
---
>     #next_location = ""
>     #current_emotional_state ="neutral"
>     #current_state = ""  #check variable!
24a32
>         # ADD VARIABLE FOR CURRENT BEHAVIOR
32c40
<         self.current_state = ""
---
>         self.current_state = "" #check variable!
48c56,58
<         if self.state_goal.current_state == "wandering_emotional":
---
>         if self.state_goal.current_state == "idle":
>             self.idle_state()
>         elif self.state_goal.current_state == "wandering_emotional":
51a62,64
>         #elif self.state_goal.current_state =="conveying_emotional_state":
>             #self.convey_emotional_state()
> 
77a91,92
>             #print("PrimaryStatesServer: Server state is " + str(behaviorserver_state))
>             #if behaviorserver_state != 1:
81c96
<                 rospy.loginfo("PrimaryStatesServer: %s state BehaviorServer goal succeeded. ", self.current_state) 
---
>                 rospy.loginfo("PrimaryStatesServer: %s state BehaviorServer goal succeeded. ", self.current_state) # add the behavior
86c101
<                 rospy.loginfo("PrimaryStatesServer: %s state BehaviorServer goal was preempted. ", self.current_state) 
---
>                 rospy.loginfo("PrimaryStatesServer: %s state BehaviorServer goal was preempted. ", self.current_state) # add the behavior
91c106
<                 rospy.loginfo("PrimaryStatesServer: %s state BehaviorServer goal aborted. ", self.current_state)
---
>                 rospy.loginfo("PrimaryStatesServer: %s state BehaviorServer goal aborted. ", self.current_state) # add the behavior
97c112,130
<     def wandering_emotional(self):
---
>     def idle_state(self):
>         rospy.loginfo("PrimaryStatesServer: Executing Idle state...")
>         self.create_and_send_behavior_goal(behavior = "idle")
>         behaviorserver_state ="" # fix for all!
>         feedback_time = time.time()
>         while not rospy.is_shutdown():
>             self.RATE_ACTIONLOOP.sleep()
>             if self.actionloop_check() == True:
>                 return
>             if (time.time() - feedback_time >10):
>                 # Increase boredom by lowering PADS values
>                 self.send_emotion(pleasure = -0.04, arousal = -0.04, dominance =  -0.04)
>                 feedback_time = time.time()
>         # set terminal goal status in case of shutdown
>         self.server_primary_states.set_aborted()
> 
> 
> 
>     def wandering_emotional(self): #dunno if goal really needed here? depends on eventhandling i guess
101a135
>         start_time = time.time()
115,116d148
< 
< 
123c155
<             if self.current_emotional_state == "angry":
---
>             if self.current_emotional_state == "angry": #need emotionsubscriber
125a158
>                 print(self.next_location)
131a165
>                 print(self.next_location)
136a171
>             print("nav_planning entering nav_emotional_goal_event")
152c187
<                 self.next_location = "wandering"
---
>                 self.next_location = "wandering" # maybe not needed
155a191,209
>     def waking_up(self):
>         behavior = ""
>         if self.current_emotional_state in ["angry", "inhibited", "bored"]:
>             behavior = "waking_up_grumpy"
>             self.send_emotion(pleasure=0.0, arousal= 0.2, dominance = -0.3)
>         else:
>             behavior = "waking_up_happy"
>             self.send_emotion(pleasure=0.1, arousal=0.1, dominance = 0.1)
>         # emotional reward?
>         rospy.loginfo("PrimaryStatesServer: Executing waking up state.")
>         self.create_and_send_behavior_goal(behavior = behavior)
>         while not rospy.is_shutdown():
>             if self.actionloop_check() == True:
>                 return
>             self.RATE_ACTIONLOOP.sleep()
>         # set terminal goal status in case of shutdown
>         self.server_primary_states.set_aborted()
> 
> 
156a211,217
>     def sleeping_state(self):
>         rospy.loginfo("PrimaryStatesServer: Executing sleeping state.")
>         # docking until time to wake up and almost full battery
>         if False:# time now > scheduled time and battery > 80%
>             
>             return
>             
158a220
>         print("PrimaryStatesServer: Changing state")
164a227
>                     print("PrimaryStatesServer: Change state function entered location if.")
166a230
>                 print("PrimaryStatesServer: Change state function publishing event.")
Binary files /home/johanndk/catkin_ws/src/cyborg_primary_states/src/primary_states_server.pyc and /home/johanndk/Documents/Fra Cyborg Robot/cyborg/catkin_ws/src/cyborg_primary_states/src/primary_states_server.pyc differ

# Only in /home/johanndk/Documents/Fra Cyborg Robot/cyborg/catkin_ws/src/: cyborg_ros_command #
# diff -r /home/johanndk/catkin_ws/src/cyborg_ros_controller/controller.launch "/home/johanndk/Documents/Fra Cyborg Robot/cyborg/catkin_ws/src/cyborg_ros_controller/controller.launch" #
9a10,13
> 
> <!-- 
> <node pkg="cyborg_eventscheduler" name="cyborg_eventscheduler" type="eventscheduler.py" output= "screen" />
> -->

# Only in /home/johanndk/catkin_ws/src/cyborg_ros_controller: .gitignore #

# Only in /home/johanndk/Documents/Fra Cyborg Robot/cyborg/catkin_ws/src/cyborg_ros_controller: rosgraph.svg # 

# diff -r /home/johanndk/catkin_ws/src/cyborg_ros_controller/src/controller.py "/home/johanndk/Documents/Fra Cyborg Robot/cyborg/catkin_ws/src/cyborg_ros_controller/src/controller.py" # 
7,8d6
< """Modified by Areg Babayan spring 2019"""
< 
30a29
> #from statemachinemonitor import StateMachineMonitor
59a59
>         #database_handler.add_event(state="idle", event="r2d2short", reward_pleasure=0.005, reward_arousal=0.005, reward_dominance = 0.0, event_cost= event_cost/2)
81,82c81,82
<         idle_transitions = {"navigation_schedular":"navigation",
<                             "navigation_emotional":"navigation",
---
>         idle_transitions = {"navigation_schedular":"navigation_planning",
>                             "navigation_emotional":"navigation_planning",
86a87
>                             "r2d2short":"r2d2short",
123c124
<                                 "cyborg_wake_up":"waking_up"}
---
>                                 "cyborg_wake_up":"waking_up"} # MEMBER to addd publish cyborg_wake_up
152,177c153,163
<         sm_nav = smach.StateMachine(outcomes=["succeeded","aborted","power_low"])
< 
<         with sm_nav:
<         
<             navigation_planning_transitions = {"navigation_start_moving_schedular":"navigation_go_to_schedular",
<                                                 "navigation_start_moving_emotional":"navigation_go_to_emotional",
<                                                 "navigation_start_wandering":"wandering_emotional",
<                                                 "aborted":"aborted"}
<             navigation_planning_resources = {}
< 
<             smach.StateMachine.add("navigation_planning",
<                                     Module("navigation_planning", "cyborg_primary_states",navigation_planning_transitions, navigation_planning_resources),
<                                     navigation_planning_transitions,
<                                     sm_remapping)
< 
<             navigation_go_to_transitions = {"succeeded":"succeeded", "aborted":"aborted", "preempted":"aborted"}
< 
<             smach.StateMachine.add("navigation_go_to_emotional", statemachines.sequence_navigation_go_to_emotional,
<                             transitions = navigation_go_to_transitions,
<                             remapping = sm_remapping)
< 
<             navigation_go_to_schedular_resources = {}
<             smach.StateMachine.add("navigation_go_to_schedular",
<                                     Module("navigation_go_to_schedular", "cyborg_behavior", navigation_go_to_transitions, navigation_go_to_schedular_resources),
<                                     navigation_go_to_transitions,
<                                     sm_remapping)
---
>         navigation_planning_transitions = {"navigation_start_moving_schedular":"navigation_go_to_schedular",
>                                             "navigation_start_moving_emotional":"navigation_go_to_emotional",
>                                             "navigation_start_wandering":"wandering_emotional",
>                                             #"navigation_command":"navigation_go_to",
>                                             "aborted":"idle"}
>         navigation_planning_resources = {}
> 
>         smach.StateMachine.add("navigation_planning",
>                                 Module("navigation_planning", "cyborg_primary_states",navigation_planning_transitions, navigation_planning_resources),
>                                 navigation_planning_transitions,
>                                 sm_remapping)
178a165
>         navigation_go_to_transitions = {"succeeded":"idle", "aborted":"idle", "preempted":"idle"}
180,183c167,175
<             navigation_wandering_emotional_transitions = {"navigation_wandering_complete":"succeeded",
<                                                 "aborted":"aborted",
<                                                 "power_low":"power_low"}
<             navigation_wandering_emotional_resources  = {}
---
>         smach.StateMachine.add("navigation_go_to_emotional", statemachines.sequence_navigation_go_to_emotional,
>                         transitions = navigation_go_to_transitions,
>                         remapping = sm_remapping)
> 
>         navigation_go_to_schedular_resources = {}
>         smach.StateMachine.add("navigation_go_to_schedular",
>                                 Module("navigation_go_to_schedular", "cyborg_behavior", navigation_go_to_transitions, navigation_go_to_schedular_resources),
>                                 navigation_go_to_transitions,
>                                 sm_remapping)
185,188c177,183
<             smach.StateMachine.add("wandering_emotional",
<                                     Module("wandering_emotional", "cyborg_primary_states", navigation_wandering_emotional_transitions, navigation_wandering_emotional_resources),
<                                     navigation_wandering_emotional_transitions,
<                                     sm_remapping)
---
>         """navigation_go_to_transitions = {"succeeded":"idle",
>                                         "aborted":"idle"}
>         navigation_go_to_resources = {}
>         smach.StateMachine.add("navigation_go_to",
>                                 Module("navigation_go_to", "cyborg_behavior", navigation_go_to_transitions, navigation_go_to_resources),
>                                 navigation_go_to_transitions,
>                                 sm_remapping)
190,191c185,202
<         sm_nav_transitions = {"aborted":"idle","succeeded":"idle", "power_low":"exhausted"}
<         smach.StateMachine.add("navigation", sm_nav, sm_nav_transitions, sm_remapping)
---
>         navigation_go_to_angry_transitions = {"succeeded":"idle",
>                                         "aborted":"idle"}
>         navigation_go_to_angry_resources = {}
>         smach.StateMachine.add("navigation_go_to_angry",
>                                 Module("navigation_go_to_angry", "cyborg_behavior", navigation_go_to_angry_transitions, navigation_go_to_angry_resources),
>                                 navigation_go_to_angry_transitions,
>                                 sm_remapping)"""
> 
>         navigation_wandering_emotional_transitions = {"navigation_wandering_complete":"idle",
>                                             "succeeded":"idle",
>                                             "aborted":"idle",
>                                             "power_low":"exhausted"}
>         navigation_wandering_emotional_resources  = {}
> 
>         smach.StateMachine.add("wandering_emotional",
>                                 Module("wandering_emotional", "cyborg_primary_states", navigation_wandering_emotional_transitions, navigation_wandering_emotional_resources),
>                                 navigation_wandering_emotional_transitions,
>                                 sm_remapping)
210,211c221,435
<         sm.register_io_keys({"state_machine_events","last_state","last_event"})
<         sm_nav.register_io_keys({"state_machine_events","last_state","last_event"})
---
>         r2d2short_transitions = {"aborted":"idle",
>                                 "succeeded":"idle"}
> 
>         r2d2short_resources = {}
>         smach.StateMachine.add("r2d2short",
>                                 Module("r2d2short","cyborg_behavior", r2d2short_transitions, r2d2short_resources),
>                                     r2d2short_transitions,
>                                     sm_remapping)
>                                     
> 
>         """nav_planning_transitions = {"succeeded":"idle",
>                                     "navigation_start_wandering": "wandering",
>                                     "navigation_start_moving":"navigation_go_to"  #FIX THIS
>                                 "aborted": "idle"}
>         nav_resources = {}
> 
>         smach.StateMachine.add("navigation_planning",
>                             Module("navigation_planning", "cyborg_primary_states/navigation_planning", )
>         )"""
> 
>         """sequence_tour_goal = Sequence(
>                     outcomes = ['succeeded', 'aborted', 'preempted'],
>                     connector_outcome = 'succeeded')
> 
>         sequence_tour_goal.userdata.state_machine_events  = []
>         sequence_tour_goal.userdata.last_state = "initializing"
>         sequence_tour_goal.userdata.last_event = "succeeded"
>             
>         sm_remapping = {"input_events":"state_machine_events",
>                     "output_events":"state_machine_events",
>                     "previous_state":"last_state",
>                     "current_state":"last_state",
>                     "previous_event":"last_event",
>                     "current_event":"last_event"}
> 
>         with sequence_tour_goal:
> 
>             sequence_tour_goal_transitions = {"succeeded":"succeeded", "aborted" : "aborted", "preempted" : "preempted"}
>             sequence_tour_goal_resources = {}
> 
>             #Sequence.add('transport',
>             #            Module('navigation_go_to', 'cyborg_behavior/behavior', sequence_tour_goal_transitions, sequence_tour_goal_resources), remapping= sm_remapping)
> 
>             Sequence.add('arrival',
>                         Module('arrival', 'cyborg_behavior/behavior', sequence_tour_goal_transitions, sequence_tour_goal_resources), remapping=sm_remapping)
> 
>             Sequence.add('show_off',
>                         Module('show_off', 'cyborg_behavior/behavior', sequence_tour_goal_transitions, sequence_tour_goal_resources), remapping= sm_remapping)
> 
>         tour_goal_transitions = {"succeeded": "idle", "aborted": "idle", "preempted":"idle"} # MEMBER TO FILL   
>         smach.StateMachine.add("tour_goal", sequence_tour_goal, tour_goal_transitions, sm_remapping)""" 
> 
>                                 
>    
> 
>     # Create motivator
>     """motivator = Motivator(database_file=path)
>     motivator.start()
> 
>     # Create a SMACH state machine
>     sm = smach.StateMachine(outcomes=["error"])
> 
>     sm.userdata.state_machine_events = []
>     sm.userdata.last_state = "initializing"
>     sm.userdata.last_event = "start_up"
> 
>     # Open the container
>     with sm:
>         # Remap outputs, so userdata can be moved between states
>         sm_remapping = {"input_events":"state_machine_events",
>                         "output_events":"state_machine_events",
>                         "previous_state":"last_state",
>                         "current_state":"last_state",
>                         "previous_event":"last_event",
>                         "current_event":"last_event"}
> 
>         # Add states to the container
>         idle_transitions = {"navigation_scheduler":"navigation",
>                             "navigation_emotional":"navigation",
>                             "aborted":"idle",
>                             "navigation_command":"navigation",
>                             "music_play":"music"}
>         idle_resources = {} # Idle does not require any resources
>         smach.StateMachine.add("idle",
>                                 Module("idle", "cyborg_idle/idle", idle_transitions, idle_resources),
>                                 idle_transitions,
>                                 sm_remapping)
> 
>         music_transitions = {"aborted":"idle",
>                              "succeeded":"idle"} 
>         music_resources = {}
>         smach.StateMachine.add("music",
>                                 Module("music", "cyborg_music/music", music_transitions, music_resources),
>                                 music_transitions,
>                                 sm_remapping)
> 
>         nav_transitions = {"aborted":"idle",
>                             "succeeded":"idle",
>                             "navigation_feedback_completed":"conversation",
>                             "navigation_command":"navigation"}
>         nav_resources = {"base":"cyborg_navigation", "trollface":"cyborg_navigation"}
>         smach.StateMachine.add("navigation_talking",
>                                 Module("navigation_talking", "cyborg_navigation/talking", nav_transitions, nav_resources),
>                                 nav_transitions,
>                                 sm_remapping)
> 
>         # Navigation
>         sm_nav = smach.StateMachine(outcomes=["idle",
>                                              "feedback_complete"],
>                                              input_keys=["state_machine_events",
>                                                          "last_state",
>                                                          "last_event"],
>                                              output_keys=["state_machine_events",
>                                                           "last_state",
>                                                           "last_event"]) #added keys here, else does not work properly
> 
>         with sm_nav:
>             nav_transitions = {"aborted":"idle",
>                                 "navigation_start_wandering":"navigation_moving",
>                                 "navigation_start_moving":"navigation_moving",
>                                 "succeeded":"idle"}
>             nav_resources = {"base":"cyborg_navigation"}
>             smach.StateMachine.add("navigation_planning",
>                                     Module("navigation_planning", "cyborg_navigation/planning", nav_transitions, nav_resources),
>                                     nav_transitions,
>                                     sm_remapping)   
> 
>             nav_transitions = {"navigation_wandering_completed":"idle",
>                                 "aborted":"idle",
>                                 "succeeded":"idle"}
>             nav_resources = {"base":"cyborg_navigation"}
>             smach.StateMachine.add("navigation_moving",
>                                     Module("navigation_moving", "cyborg_navigation/moving", nav_transitions, nav_resources),
>                                     nav_transitions,
>                                     sm_remapping)
>         
> 
> 
>         nav_transitions = {"idle":"idle", "feedback_complete":"idle"}
>         smach.StateMachine.add("navigation", sm_nav, nav_transitions, sm_remapping)
> 
>         # Conversation
>         sm_conv = smach.StateMachine(outcomes=["idle", "navigation"])
> 
>         with sm_conv:
>             conversation_transitions = {"aborted":"idle",
>                                         "succeeded":"idle",
>                                         "navigation_feedback":"navigation",
>                                         "navigation_command":"navigation",
>                                         "navigation_information":"navigation",
>                                         "simon_says_play":"simon_says",
>                                         "selfie_take":"selfie",
>                                         "follower_follow":"follower",
>                                         "weather_tell":"weather",
>                                         "joke_tell":"joke"}
>             conversation_resources = {"trollface":"cyborg_conversation"}
>             smach.StateMachine.add("conversation_idle",
>                                     Module("conversation_idle", "cyborg_conversation/conversation_idle", conversation_transitions, conversation_resources),
>                                     conversation_transitions,
>                                     sm_remapping)
> 
> 
>             conversation_transitions = {"aborted":"conversation_idle",
>                                         "succeeded":"conversation_idle"}
>             simon_says_resources = {"trollface":"cyborg_simon_says"}
>             smach.StateMachine.add("simon_says",
>                                     Module("simon_says", "cyborg_conversation/simon_says", conversation_transitions, simon_says_resources),
>                                     conversation_transitions,
>                                     sm_remapping)
> 
>             selfie_resources = {"trollface":"cyborg_selfie"}
>             smach.StateMachine.add("selfie",
>                                     Module("selfie", "cyborg_conversation/selfie", conversation_transitions, selfie_resources),
>                                     conversation_transitions,
>                                     sm_remapping)
> 
>             follower_resources = {"trollface":"cyborg_follower"}
>             smach.StateMachine.add("follower",
>                                     Module("follower", "cyborg_conversation/follower", conversation_transitions, follower_resources),
>                                     conversation_transitions,
>                                     sm_remapping)
> 
>             weather_resources = {"trollface":"cyborg_weather"}
>             smach.StateMachine.add("weather",
>                                     Module("weather", "cyborg_conversation/weather", conversation_transitions, weather_resources),
>                                     conversation_transitions,
>                                     sm_remapping)
> 
>             joke_resources = {"trollface":"cyborg_joke"}
>             smach.StateMachine.add("joke",
>                                     Module("joke", "cyborg_conversation/joke", conversation_transitions, joke_resources),
>                                     conversation_transitions,
>                                     sm_remapping)
> 
>         conversation_transitions = {"idle":"idle", "navigation":"navigation_talking"}
>         smach.StateMachine.add("conversation", sm_conv, conversation_transitions, sm_remapping)
> 
> 
>         ################################################### ADD MORE STATES BELOW ################################################### 
> 
> 
> 
> 
>         ################################################### STOP ADDING YOUR STATES ################################################### 
> 
>     sis = smach_ros.IntrospectionServer('controller_viewer', sm, '/controller_viewer')
>     sis.start()
> 
>     # Create a thread to execute the smach
>     smach_thread = threading.Thread(target=sm.execute)
>     smach_thread.daemon = True
>     smach_thread.start()
>     rospy.loginfo("Controller: SMACH activated...")
>     
> 
212a437,438
>     #sm.register_io_keys({"state_machine_events","last_state","last_event"})
>     #sm_nav.register_io_keys({"state_machine_events","last_state","last_event"})"""
Binary files /home/johanndk/catkin_ws/src/cyborg_ros_controller/src/databasehandler.pyc and /home/johanndk/Documents/Fra Cyborg Robot/cyborg/catkin_ws/src/cyborg_ros_controller/src/databasehandler.pyc differ
Binary files /home/johanndk/catkin_ws/src/cyborg_ros_controller/src/emotionsystem.pyc and /home/johanndk/Documents/Fra Cyborg Robot/cyborg/catkin_ws/src/cyborg_ros_controller/src/emotionsystem.pyc differ


# diff -r /home/johanndk/catkin_ws/src/cyborg_ros_controller/src/module.py "/home/johanndk/Documents/Fra Cyborg Robot/cyborg/catkin_ws/src/cyborg_ros_controller/src/module.py" #
72a73
>         # next line can be omitted since not in use, OR might wanna use similar line now
120a122,125
> 
>                     # add handling of mood here
>                     # call mood node on event or similar
> 
Binary files /home/johanndk/catkin_ws/src/cyborg_ros_controller/src/module.pyc and /home/johanndk/Documents/Fra Cyborg Robot/cyborg/catkin_ws/src/cyborg_ros_controller/src/module.pyc differ


# diff -r /home/johanndk/catkin_ws/src/cyborg_ros_controller/src/motivator.py "/home/johanndk/Documents/Fra Cyborg Robot/cyborg/catkin_ws/src/cyborg_ros_controller/src/motivator.py" #
124c124
<             #print(event.event,r+cost_event, -cost_event, r) # event_name, tot_reward + cost_event, -cost_event, tot_reward
---
>             print(event.event,r+cost_event, -cost_event, r) # event_name, tot_reward + cost_event, -cost_event, tot_reward
Binary files /home/johanndk/catkin_ws/src/cyborg_ros_controller/src/motivator.pyc and /home/johanndk/Documents/Fra Cyborg Robot/cyborg/catkin_ws/src/cyborg_ros_controller/src/motivator.pyc differ

# Only in /home/johanndk/catkin_ws/src/cyborg_ros_controller/src: statemachinemonitor.py #

# diff -r /home/johanndk/catkin_ws/src/cyborg_ros_controller/src/statemachines.py "/home/johanndk/Documents/Fra Cyborg Robot/cyborg/catkin_ws/src/cyborg_ros_controller/src/statemachines.py" #
2a3,5
> 
> 
> """Created by Areg Babayan on 16/04/2019."""
4c7
< __version__= "0.0.2"
---
> __version__= "0.0.1"
24a28,29
> 
> # Might need to add userdata here, check controller-new
Binary files /home/johanndk/catkin_ws/src/cyborg_ros_controller/src/statemachines.pyc and /home/johanndk/Documents/Fra Cyborg Robot/cyborg/catkin_ws/src/cyborg_ros_controller/src/statemachines.pyc differ

# Only in /home/johanndk/Documents/Fra Cyborg Robot/cyborg/catkin_ws/src/cyborg_ros_controller: .vscode #

# Only in /home/johanndk/Documents/Fra Cyborg Robot/cyborg/catkin_ws/src/: cyborg_ros_led_dome #

# diff -r /home/johanndk/catkin_ws/src/cyborg_ros_navigation/README.md "/home/johanndk/Documents/Fra Cyborg Robot/cyborg/catkin_ws/src/cyborg_ros_navigation/README.md" #
6,7c6
< Numbers of actionlib server(s): 1   
< 
---
> Numbers of actionlib server(s): 3   
18,22c17,19
< * The go_to state: The Cyborg is moving to the next location. Available at actionlib server topic cyborg_navigation/navigation.  
< * The wandering state: The Cyborg is wandering until preempted. Available at actionlib server topic cyborg_navigation/navigation.
< * The docking state: Not yet finished.
< * Publishes current location on topic cyborg_navigation/current_location.
< 
---
> * The planning state: Finds the next location. Available at actionlib server topic cyborg_navigation/planning.
> * The movinging state: The Cyborg is moving to the next location. Available at actionlib server topic cyborg_navigation/moving.   
> * The talkinging state: The Cyborg is talking. Available at actionlib server topic cyborg_navigation/talking.   
Binary files /home/johanndk/catkin_ws/src/cyborg_ros_navigation/src/databasehandler.pyc and /home/johanndk/Documents/Fra Cyborg Robot/cyborg/catkin_ws/src/cyborg_ros_navigation/src/databasehandler.pyc differ


# diff -r /home/johanndk/catkin_ws/src/cyborg_ros_navigation/src/navigation.py "/home/johanndk/Documents/Fra Cyborg Robot/cyborg/catkin_ws/src/cyborg_ros_navigation/src/navigation.py" #
11a12
> #from eventscheduler import EventScheduler
39,44c40,52
< 
<         database_handler.add_event(event_name="welcome_time", location_name="entrance", start_date=datetime.datetime(2018, 1, 18, 11, 0, 0, 1), end_date=datetime.datetime(2018, 1, 18, 11, 4, 0, 1), ignore=False)
<         database_handler.add_event(event_name="dinner_time", location_name="cafeteria", start_date=datetime.datetime(2018, 1, 18, 15, 0, 7, 1) , end_date=datetime.datetime(2018, 1, 18, 15, 59, 0, 1), ignore=False)
<         database_handler.add_event(event_name="wait_time", location_name="waiting area", start_date=datetime.datetime(2018, 1, 18, 8, 0, 1, 1) , end_date=datetime.datetime(2018, 1, 18, 8, 46, 0, 1), ignore=False)
<         database_handler.add_event(event_name="lunch_time", location_name="cafeteria", start_date=datetime.datetime(2018, 1, 18, 7, 0, 1, 1) , end_date=datetime.datetime(2018, 1, 18, 7, 0, 0, 1), ignore=False)
<         database_handler.add_event(event_name="goodbye_time", location_name="entrance 2", start_date=datetime.datetime(2018, 1, 18, 9, 0, 1, 1), end_date=datetime.datetime(2018, 1, 18, 9, 0, 0, 1), ignore=False)
---
>         """
>         database_handler.add_event(event_name="welcome_time", location_name="entrance", start_date=datetime.datetime(2017, 1, 18, 11, 0, 0, 1), end_date=datetime.datetime(2020, 1, 18, 11, 4, 0, 1), ignore=False)
>         database_handler.add_event(event_name="dinner_time", location_name="cafeteria", start_date=datetime.datetime(2017, 1, 18, 15, 0, 7, 1) , end_date=datetime.datetime(2020, 1, 18, 15, 59, 0, 1), ignore=False)
>         database_handler.add_event(event_name="wait_time", location_name="waiting area", start_date=datetime.datetime(2017, 1, 18, 8, 0, 1, 1) , end_date=datetime.datetime(2020, 1, 18, 8, 46, 0, 1), ignore=False)
>         database_handler.add_event(event_name="lunch_time", location_name="cafeteria", start_date=datetime.datetime(2017, 1, 18, 7, 0, 1, 1) , end_date=datetime.datetime(2020, 1, 18, 7, 0, 0, 1), ignore=False)
>         database_handler.add_event(event_name="goodbye_time", location_name="entrance 2", start_date=datetime.datetime(2017, 1, 18, 9, 0, 1, 1), end_date=datetime.datetime(2020, 1, 18, 9, 0, 0, 1), ignore=False)"""
>         ## test ## member to complete
>         #database_handler.add_event(event_name= "bedtime", location_name = "home", start_date=datetime.datetime(0,0,0,0,0) , end_date=datetime.datetime(0,0,0,0,0,0) , ignore = False)
>         #database_handler.add_location(location_name="hallway3", robot_map_name="ntnu2.map", x=-15.300, y=-5.520, z=0, p=0, j=0, r=2, threshold=2, crowded=False, environment=0.01)
>         #database_handler.add_location(location_name="hallway4", robot_map_name="ntnu2.map", x=-21.800, y=-5.830, z=0, p=0, j=0, r=2, threshold=2, crowded=False, environment=0.01)
>         #database_handler.add_event(event_name="test_event", location_name="entrance", start_date=datetime.datetime(2019, 1, 1, 1, 1, 1, 1), end_date=datetime.datetime(2020, 1, 1, 1, 1, 1, 1), ignore= False)
>         #database_handler.add_event(event_name="test_hallway3", location_name="hallway3", start_date=datetime.datetime(2019, 1, 1, 1, 1, 1, 1), end_date=datetime.datetime(2020, 1, 1, 1, 1, 1, 1), ignore= False)
>         #database_handler.add_event(event_name="test_hallway4", location_name="hallway4", start_date=datetime.datetime(2019, 1, 1, 1, 1, 1, 1), end_date=datetime.datetime(2020, 1, 1, 1, 1, 1, 1), ignore= False)


# diff -r /home/johanndk/catkin_ws/src/cyborg_ros_navigation/src/navigationserver.py "/home/johanndk/Documents/Fra Cyborg Robot/cyborg/catkin_ws/src/cyborg_ros_navigation/src/navigationserver.py" #
5a6,9
> # IDE: ha bade behaviorserver og actionserver for oppforsel i hver tilstand, mulighet til tilpasset oppforsel, vurder om bedre enn alternativ
> # tror jeg vil beholde sann som tenkt, men kan argumenter i oppgave om at det er enkelt a utvide med oppforselen i tillegg til behavior
> # member to update with emotional feedback when ready
> 
22c26
< from cyborg_controller.msg import StateMachineFeedback, StateMachineResult, EmotionalFeedback
---
> from cyborg_controller.msg import StateMachineFeedback, StateMachineResult, EmotionalFeedback #move this outside?
26a31,55
> 	
> 	#client_base_feedback = StateMachineFeedback()
> 	#client_base_result = StateMachineResult()
> 	#next_location = None
> 	#command_location = None
> 	#current_location = None
> 	#current_location_name = None
> 	#EMOTIONAL_FEEDBACK_CYCLE = 15 # (s)
> 	#current_x = 0.0
> 	#current_y = 0.0
> 
> 	#MAP_NAME = "glassgarden.map"
> 	#base_canceled = False
> 	#base_succeeded = False
> 
> 
> 	#TIMEOUT_GOTO = 300 # (S)
> 	#TIMEOUT_WANDERING = 600 # (S)
> 
> 	########################
> 	### MIGHT WANT TO ADD CHECK IF MOTORS ARE ENABLED 
> 	### rostopic echo /rosarnl_node/motors_state
> 	### AND ENABLE IF DISABLED WHEN MOVING
> 	### rosservice call /rosarnl_node/enable_motors
> 	#######################
33c62
< 		self.SERVER_RATE = rospy.Rate(10) #(Hz)
---
> 		self.SERVER_RATE = rospy.Rate(10) #(Hz), might wanna up it to get faster preemption
40a70
> 		#self.command_location = None # Might be unnecessary
48a79
> 		#self.event_publisher = rospy.Publisher("/cyborg_controller/register_event", String, queue_size=100) #dunno if here or in behavior
63a95
> 		#self.location_updater()
77a110
> 		print("NavigationServer: Entered client_base_done_callback, state: "+ str(state))
136a170
> 					print("Navigationserver: navigation_go_to succeeded.")
141a176
> 					print("Navigationserver: preempt requested, preempted.")
145a181
> 					print("Navigationserver: Base canceled, aborted.")
151a188
> 					print("NavigationServer timed out, aborting.")
161a199
> 			print("NavigationServer: navigation_go_to exited while-loop.")
168d205
< 	# Sends the location to the ROSARNL node (Pioneer XL robot base)
169a207
> 	# Sends the location to the ROSARNL node (Pioneer XL robot base)
191a230
> 			print("NavigationServer: Checking path...")
195c234
< 				rospy.loginfo("NavigationServer: Path OK.")
---
> 				print("NavigationServer: Path OK.")
202c241
< 			rospy.logdebug("NavigationServer: Make plan service did not process request: " + str(exc))
---
> 			print("NavigationServer: Make plan service did not process request: " + str(exc))
208c247
< 	# The robot base starts to wander and keeps wandering until it is preempted
---
> 	# The robot base starts to wander and keeps wandering until it is preempted or no longer in the right mood 
224c263
< 				self.client_base.cancel_all_goals()
---
> 				self.client_base.cancel_all_goals() # HERE - goal on wandering?
233a273
> 			#if (time.time() - started_waiting > 5): #prevent eternal looping of state
235c275
< 				self.client_base.cancel_all_goals()
---
> 				self.client_base.cancel_all_goals() # HERE - goal on wandering?
243a284
> 			#if (time.time() - feedback_timer > 2): # give emotional feedback every 15 seconds
252c293
< 	def navigation_dock(self): # NOT YET FINISHED
---
> 	def navigation_dock(self):
267a309,317
> 			"""if time.time() - started_waiting > 50: # THIS IS WRONG; NO SUCCEEDED IF TIMEOUT
> 				self.client_base.cancel_all_goals()
> 				try:
> 					baseStopDocking = rospy.ServiceProxy("/rosarnl_node/stop", Empty)
> 					baseStopDocking()
> 				except rospy.ServiceException, exc:
> 					rospy.logdebug("NavigationServer: stop docking service error - " + str(exc))
> 				self.server_navigation.set_aborted()"""
> 
275c325
< 				self.server_navigation.set_preempted()
---
> 				self.server_navigation.set_preempted() # ?
Binary files /home/johanndk/catkin_ws/src/cyborg_ros_navigation/src/navigationserver.pyc and /home/johanndk/Documents/Fra Cyborg Robot/cyborg/catkin_ws/src/cyborg_ros_navigation/src/navigationserver.pyc differ

# Only in /home/johanndk/Documents/Fra Cyborg Robot/cyborg/catkin_ws/src/cyborg_ros_navigation: .vscode #
# Only in /home/johanndk/Documents/Fra Cyborg Robot/cyborg/catkin_ws/src/: executive_smach_visualization #
# Only in /home/johanndk/catkin_ws/src/: LED-box #
# Only in /home/johanndk/catkin_ws/src/: LED-dome # 
# Only in /home/johanndk/catkin_ws/src/: maps #
# Only in /home/johanndk/catkin_ws/src/: mode_selector #
# Only in /home/johanndk/Documents/Fra Cyborg Robot/cyborg/catkin_ws/src/: README.md #
# Only in /home/johanndk/catkin_ws/src/: rosaria # 
# Only in /home/johanndk/catkin_ws/src/: rosaria_client #

# diff -r /home/johanndk/catkin_ws/src/ros-arnl/CMakeLists.txt "/home/johanndk/Documents/Fra Cyborg Robot/cyborg/catkin_ws/src/ros-arnl/CMakeLists.txt"
6c6
< find_package(catkin REQUIRED COMPONENTS message_generation roscpp nav_msgs geometry_msgs tf genmsg actionlib_msgs actionlib nav_msgs)
---
> find_package(catkin REQUIRED COMPONENTS message_generation roscpp nav_msgs geometry_msgs tf genmsg actionlib_msgs actionlib nav_msgs message_generation)
28,31c28,32
< #add_service_files(
< #  FILES
< #  # TODO: List your msg files here
< #)
---
> add_service_files(
>   FILES
>   LoadMapFile.srv
>   MakePlan.srv
> )

# Only in /home/johanndk/Documents/Fra Cyborg Robot/cyborg/catkin_ws/src/ros-arnl: .gitignore #

# diff -r /home/johanndk/catkin_ws/src/ros-arnl/package.xml "/home/johanndk/Documents/Fra Cyborg Robot/cyborg/catkin_ws/src/ros-arnl/package.xml" # 
25a26
>   <build_depend>message_generation</build_depend>
34a36
>   <run_depend>message_runtime</run_depend>


# diff -r /home/johanndk/catkin_ws/src/ros-arnl/README.md "/home/johanndk/Documents/Fra Cyborg Robot/cyborg/catkin_ws/src/ros-arnl/README.md" #
16c16,22
< (catkin) workspace.  Run `catkin_make` from the workspace directory.
---
> (catkin) workspace.  `ros-arnl` depends on several ROS packages including 
> `nav_msgs`, `geometry_msgs`, `tf`, `move_base_msgs`, `actionlib`, and
> `actionlib_msgs`. To install these automatically, run `rosdep update`,
> then run `rosdep install rosarnl`.   (You may need to import the ROS
> development environment to your shell first by running `. devel/setup.bash`
> from your catkin workspace directory.)
> Run `catkin_make --pkg rosarnl` from the workspace directory to build.
77a84,91
>  * `/rosarnl_node/load_map_file`: Service which loads a new map file. This is a
>    shortcut to modifying the ARNL configuration with the new file name.  Provide
>    a string with the service request containing the file name of the map
>    relative to ARNL's map directory (currently fixed at `/usr/local/Arnl/examples`
>    but may be changed in the future) on the system running `rosarnl_node`.
>    For example, using `rosservice` on the command line: 
>    `rosservice call /rosarnl_node/load_map_file columbia.map`
>  * `/rosarnl_node/make_plan`: Service which plans a path to the given goal goal point, but which does not start driving the robot on that path.  A point sequence representing the path is returned.  This plan will not incorporate sensed obstacles, just static map data.  
79c93
< The rosarnl node dosen't provide map data. This may be added
---
> The rosarnl node dosen't provide map data directly. This may be added


# diff -r /home/johanndk/catkin_ws/src/ros-arnl/rosarnl_node.cpp "/home/johanndk/Documents/Fra Cyborg Robot/cyborg/catkin_ws/src/ros-arnl/rosarnl_node.cpp" #
13a14,16
> #include <rosarnl/LoadMapFile.h>
> #include <rosarnl/MakePlan.h>
> 
53,56c56,59
<     ArPose rosPoseToArPose(const geometry_msgs::Pose& p);
<     ArPose rosPoseToArPose(const geometry_msgs::PoseStamped& p) { return rosPoseToArPose(p.pose); }
<     ArPoseWithTime rosPoseStampedToArPoseWithTime(const geometry_msgs::PoseStamped& p); 
<     geometry_msgs::Pose rosPoseToArPose(const ArPose& arpose);
---
>     static ArPose rosPoseToArPose(const geometry_msgs::Pose& p);
>     static ArPose rosPoseToArPose(const geometry_msgs::PoseStamped& p) { return rosPoseToArPose(p.pose); }
>     static ArPoseWithTime rosPoseStampedToArPoseWithTime(const geometry_msgs::PoseStamped& p);
>     static geometry_msgs::Pose arPoseToRosPose(const ArPose& arpose);
62a66,67
>     ros::ServiceServer map_srv;
>     ros::ServiceServer plan_srv;
68a74,75
>     bool make_plan_cb(rosarnl::MakePlan::Request& request, rosarnl::MakePlan::Response& response);
>     bool map_cb(rosarnl::LoadMapFile::Request& request, rosarnl::LoadMapFile::Response& response);
302a310
>   plan_srv = n.advertiseService("make_plan", &RosArnlNode::make_plan_cb, this);
305a314,315
>   map_srv = n.advertiseService("load_map_file", &RosArnlNode::map_cb, this);
> 
591a602,616
> bool RosArnlNode::make_plan_cb(rosarnl::MakePlan::Request& request, rosarnl::MakePlan::Response& response)
> {
>     ArPose from, to;
>     std::list<ArPose> path;
> 
>     ROS_INFO_NAMED("rosarnl_node", "rosarnl_node: Make plan request.");
> 
>     from = arnl.robot->getPose();
>     to   = rosPoseToArPose(request.goal);
>     path = arnl.pathTask->getPathFromTo(from, to, true, NULL);
>     std::transform(path.begin(), path.end(), std::back_inserter(response.path), arPoseToRosPose);
> 
>     return true;
> }
> 
600a626,635
> bool RosArnlNode::map_cb(rosarnl::LoadMapFile::Request& request, rosarnl::LoadMapFile::Response& response)
> {
>   ROS_INFO_NAMED("rosarnl_node", "rosarnl_node: Load new map file service request with file name \"%s\" received...", request.filename.c_str());
>   if(!arnl.map) return false;
>   char msg[256];
>   bool r = arnl.map->readFile(request.filename.c_str(), msg, 256);
>   response.message = msg;
>   return r;
> }
> 
616c651,652
< geometry_msgs::Pose arPoseToRosPose(const ArPose& arpose)
---
> 
> geometry_msgs::Pose RosArnlNode::arPoseToRosPose(const ArPose& arpose)

# Only in /home/johanndk/Documents/Fra Cyborg Robot/cyborg/catkin_ws/src/ros-arnl: srv #
# Only in /home/johanndk/Documents/Fra Cyborg Robot/cyborg/catkin_ws/src/ros-arnl: .vscode #
# Only in /home/johanndk/Documents/Fra Cyborg Robot/cyborg/catkin_ws/src/: serial.tools.list_ports #
# Only in /home/johanndk/Documents/Fra Cyborg Robot/cyborg/catkin_ws/src/: xdot # 

