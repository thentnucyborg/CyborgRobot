## [WARN] [1573068534.310870]: Inbound TCP/IP connection failed: connection from sender terminated before handshake header received. 0 bytes were received. Please check sender for additional details. ##
Dukket opp i terminaler ved testing på egen pc. Kommer og går litt. Den ville ikke sette noen lys på leddomen mens dette skjedde. 
- Kan kanskje hjelpe: https://answers.ros.org/question/142271/inbound-tcpip-connection-failed-help/

## 'type' object has no attribute ##
> [ERROR] [1573068647.114910]: Exception in your execute callback: 'NoneType' object has no attribute 'location_name'
Traceback (most recent call last):
  File "/opt/ros/kinetic/lib/python2.7/dist-packages/actionlib/simple_action_server.py", line 289, in executeLoop
    self.execute_callback(goal)
  File "/home/johanndk/catkin_ws/src/cyborg_primary_states/src/primary_states_server.py", line 57, in callback_server_primary_states
    self.navigation_planning_state(self.state_goal)
  File "/home/johanndk/catkin_ws/src/cyborg_primary_states/src/primary_states_server.py", line 162, in navigation_planning_state
    message = String(data = self.next_location.location_name)
AttributeError: 'NoneType' object has no attribute 'location_name'


## Cannot concatenate 'str' and 'String' objects ##
> [ERROR] [1573068731.447111]: bad callback: <bound method BehaviorServer.callback_text_to_speech of <behaviorserver.BehaviorServer instance at 0x7ffa5b8db7e8>>
Traceback (most recent call last):
  File "/opt/ros/kinetic/lib/python2.7/dist-packages/rospy/topics.py", line 750, in _invoke_callback
    cb(msg)
  File "/home/johanndk/catkin_ws/src/cyborg_behavior/src/behaviorserver.py", line 291, in callback_text_to_speech
    rospy.loginfo("BehaviorServer: " + "playback has " + message)
TypeError: cannot concatenate 'str' and 'String' objects


> [ERROR] [1573068758.295406]: bad callback: <bound method BehaviorServer.callback_playback of <behaviorserver.BehaviorServer instance at 0x7ffa5b8db7e8>>
Traceback (most recent call last):
  File "/opt/ros/kinetic/lib/python2.7/dist-packages/rospy/topics.py", line 750, in _invoke_callback
    cb(msg)
  File "/home/johanndk/catkin_ws/src/cyborg_behavior/src/behaviorserver.py", line 283, in callback_playback
    rospy.loginfo("BehaviorServer: " + "playback has " + message)
TypeError: cannot concatenate 'str' and 'String' objects


Begge har dukker opp flere ganger i terminalen under testing på egen pc, og kommer med gjevne mellomrom når koden har stårr og kjørt en stund.


## Userdata key '' not avaliable ## 
> [ERROR] [1573071138.870617]: Userdata key 'last_state' not available. Available keys are: []
[INFO] [1573071138.870779]: Controller: Activated...
[WARN] [1573071138.871108]: Attempting to copy input key 'last_state', but this key does not exist.
[ERROR] [1573071138.871384]: Userdata key 'last_event' not available. Available keys are: []
[WARN] [1573071138.871540]: Attempting to copy input key 'last_event', but this key does not exist.
[ERROR] [1573071138.871700]: Userdata key 'state_machine_events' not available. Available keys are: []
[WARN] [1573071138.871841]: Attempting to copy input key 'state_machine_events', but this key does not exist.
[INFO] [1573071138.871979]: State machine starting in initial state 'idle' with userdata: 
	['last_state', 'last_event', 'state_machine_events']

Muligens fordi noe et sted prøver å lese av userdata keys før de er initialisert. Se også "prints when stopping roslaunch.txt"


## InvalidTransitionError ## 
> [ERROR] [1573071145.171069]: InvalidTransitionError: Attempted to return outcome 'None' from state 'Startup' of type '<domecontrol.startup object at 0x7f7e75480b10>' which only has registered outcomes: ('text', 'nonmea', 'meafromfile')
Exception in thread Thread-6:
Traceback (most recent call last):
  File "/usr/lib/python2.7/threading.py", line 801, in __bootstrap_inner__
    self.run()
  File "/usr/lib/python2.7/threading.py", line 754, in run
    self.__target(*self.__args, **self.__kwargs)
  File "/opt/ros/kinetic/lib/python2.7/dist-packages/smach/state_machine.py", line 359, in execute
    container_outcome = self._update_once()
  File "/opt/ros/kinetic/lib/python2.7/dist-packages/smach/state_machine.py", line 266, in _update_once
    self._current_state.get_registered_outcomes()))
InvalidTransitionError: Attempted to return outcome 'None' from state 'Startup' of type '<domecontrol.startup object at 0x7f7e75480b10>' which only has registered outcomes: ('text', 'nonmea', 'meafromfile')


## ROS interrupt exception ##
> [ERROR] [1573076438.280342]: Exception in your execute callback: rospy shutdown
Traceback (most recent call last):
  File "/opt/ros/kinetic/lib/python2.7/dist-packages/actionlib/simple_action_server.py", line 289, in executeLoop
    self.execute_callback(goal)
  File "/home/johanndk/catkin_ws/src/cyborg_ros_navigation/src/navigationserver.py", line 112, in navigationserver_callback
    self.start_wandering()
  File "/home/johanndk/catkin_ws/src/cyborg_ros_navigation/src/navigationserver.py", line 212, in start_wandering
    rospy.wait_for_service("/rosarnl_node/wander")
  File "/opt/ros/kinetic/lib/python2.7/dist-packages/rospy/impl/tcpros_service.py", line 159, in wait_for_service
    raise ROSInterruptException("rospy shutdown")
ROSInterruptException: rospy shutdown
