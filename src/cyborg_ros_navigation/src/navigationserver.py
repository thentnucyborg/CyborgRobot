#!/usr/bin/env python
"""Created by Thomas Rostrup Andersen on 11/11/2016.
Copyright (C) 2016 Thomas Rostrup Andersen. All rights reserved.
Rewritten by Areg Babayan for NTNU Cyborg Spring 2019 """

import rospy
import actionlib
import time
import sys
import datetime
import threading
import geometry_msgs
import tf.transformations
from std_srvs.srv import Empty
from std_msgs.msg import String
from databasehandler import DatabaseHandler
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from cyborg_controller.msg import StateMachineFeedback, StateMachineResult
from cyborg_navigation.msg import NavigationAction, NavigationResult,NavigationFeedback
from geometry_msgs.msg import PoseWithCovarianceStamped
from rosarnl.srv import MakePlan
from cyborg_controller.msg import StateMachineFeedback, StateMachineResult, EmotionalFeedback


class NavigationServer():
	"""NavigationServer"""

	def __init__(self, database_file=""):
		self.MAP_NAME = "glassgarden.map"
		self.TIMEOUT_GOTO = 300 # (S)
		self.TIMEOUT_WANDERING = 600 # (S)
		self.EMOTIONAL_FEEDBACK_CYCLE = 15 # (s)
		self.SERVER_RATE = rospy.Rate(10) #(Hz)

		self.client_base_feedback = StateMachineFeedback()
		self.client_base_result = StateMachineResult()
		
		self.current_x = 0.0
		self.current_y = 0.0
		self.next_location = None
		self.current_location = None
		self.current_location_name = None
		self.base_canceled = False
		self.base_succeeded = False

		self.location_subscriber = rospy.Subscriber("/rosarnl_node/amcl_pose", PoseWithCovarianceStamped, self.location_callback)
		self.publisher_current_location = rospy.Publisher(rospy.get_name() + "/current_location", String, queue_size= 10)
		self.emotion_publisher = rospy.Publisher("/cyborg_controller/emotional_feedback", EmotionalFeedback, queue_size=100)

		self.server_navigation = actionlib.SimpleActionServer(rospy.get_name() + "/navigation", NavigationAction, execute_cb=self.navigationserver_callback, auto_start = False)
		self.server_navigation.start()
		self.client_base = actionlib.SimpleActionClient("/rosarnl_node/move_base", MoveBaseAction)

		self.database_handler = DatabaseHandler(filename=database_file)
		self.location_updater_thread = threading.Thread(target=self.location_updater)
		self.location_updater_thread.daemon = True
		self.location_updater_thread.start()
		rospy.loginfo("NavigationServer: Activated.")

	# Updates the current position when the position subscriber receives data
	def location_callback(self, data):
		self.current_x = data.pose.pose.position.x
		self.current_y = data.pose.pose.position.y


	# Updates current location based on position and publishes to other nodes
	def location_updater(self): # Threaded
		rospy.sleep(1)
		while not rospy.is_shutdown():
			self.current_location = self.database_handler.find_location(robot_map_name=self.MAP_NAME, location_x=self.current_x, location_y=self.current_y)
			self.current_location_name = self.current_location.location_name if self.current_location != None else ""
			self.publisher_current_location.publish(data=self.current_location_name)
			rospy.sleep(1)


	# Called once when the robot base (ROSARNL) goal completes
	def client_base_done_callback(self, state, result):
		if self.is_controlling_base:
			if (state == 3): # Succeeded aka arrived at location
				self.base_succeeded = True
			elif (state == 2): # Happens when we cancel goals aka preempted
				self.base_canceled = True
			elif (state == 1): # Canceled?
				self.base_canceled = True
			elif (state ==4): # aborted
				self.base_canceled = True
			self.client_base_state = state
			self.client_base_result = result
			rospy.logdebug("NavigationServer: Base has completed its execution with " + str(state) + " and result " + str(result) + ".")


	# Called once when the robot base (ROSARNL) goal becomes active
	def client_base_active_callback(self):
		rospy.logdebug("NavigationServer: Base goal has gone active.")
		self.is_controlling_base = True
		

	# Called every time feedback is received for the goal for the robot base (ROSARNL)
	def client_base_feedback_callback(self, feedback):
		rospy.logdebug("NavigationServer: Received feedback from base - " + str(feedback) + ".")
		self.client_base_feedback = feedback	
	

	#Callback for navigation, used for initiating wandering, docking, or go to location
	def navigationserver_callback(self, goal):
		self.base_canceled = False
		self.base_succeeded = False
		rospy.loginfo("NavigationServer Callback initiating:"+ str(goal.order) + " order, and " + str(goal.location_name)+" location.")
		self.is_controlling_base = False

		if goal.order =="navigation_wander":
			self.start_wandering()
		elif goal.order =="navigation_go_to" and goal.location_name !=None:
			self.navigation_go_to(goal)
		elif goal.order == "navigation_dock":
			self.navigation_dock() 
		else:
			self.server_navigation.set_aborted()
			rospy.logdebug("NavigationServer: Received event that cant be handled - " + str(goal.order) + ".")


	
	# This can be used by other nodes for moving the cyborg to a known location.
	# Preemptable, ends in Succeeded, Aborted or Preempted.
	# Increases PAD values while moving
	def navigation_go_to(self, goal):
		self.goal = goal
		rospy.loginfo("NavigationServer: go to server received a goal - " + str(self.goal.location_name))
		self.next_location = self.database_handler.search_for_location(location_name=self.goal.location_name)
		if self.next_location != None:
			self.check_and_send_goal(location=self.next_location)
			start_time = time.time()
			feedback_timer = time.time()
			# Wait until state is preempted or succeeded
			while not rospy.is_shutdown():
				if self.base_succeeded:
					self.send_emotion(pleasure=self.next_location.environment, arousal=0, dominance=0.0)
					self.server_navigation.set_succeeded()
					self.next_location = None
					return
				if self.server_navigation.is_preempt_requested():
					self.client_base.cancel_all_goals()
					self.server_navigation.set_preempted
					return
				if self.base_canceled:
					self.server_navigation.set_aborted()
					return
				else:
					server_feedback = NavigationFeedback(status = "moving")
					self.server_navigation.publish_feedback(server_feedback)
				if (time.time() - start_time > self.TIMEOUT_GOTO): #prevent eternal looping of state
					self.client_base.cancel_all_goals()
					self.server_navigation.set_aborted()
					return
				#give emotional feedback
				if (time.time() - feedback_timer > self.EMOTIONAL_FEEDBACK_CYCLE):
					self.send_emotion(pleasure = 0.0, arousal = 0.02, dominance = -0.01)
					feedback_timer = time.time()
					#return
				self.SERVER_RATE.sleep()
			# set terminal goal status in case of shutdown	
			self.server_navigation.set_aborted()
		else:
			rospy.logdebug("NavigationServer: Go to server received a location with unrecognized name - " + str(goal.location_name))
			self.server_navigation.set_aborted()


	# Sends the location to the ROSARNL node (Pioneer XL robot base)
	# Added check path of goal before sending
	def check_and_send_goal(self, location):
		if (self.client_base.wait_for_server(rospy.Duration.from_sec(5.0)) == False ):
			rospy.logwarn("NavigationServer: ERROR - Unable to connect to Pioneer XL.")
			self.base_canceled = True
			return
		pose = geometry_msgs.msg.Pose()
		pose.position.x = location.x
		pose.position.y = location.y
		pose.position.z = location.z
		q = tf.transformations.quaternion_from_euler(location.p, location.j, location.r)
		pose.orientation = geometry_msgs.msg.Quaternion(*q)
		goal = MoveBaseGoal()
		goal.target_pose.pose = pose
		goal.target_pose.header.frame_id = location.robot_map_name
		goal.target_pose.header.stamp = rospy.Time.now()
		rospy.wait_for_service("/rosarnl_node/make_plan")
		baseCheckPath = rospy.ServiceProxy("/rosarnl_node/make_plan", MakePlan())
		makeplan = MakePlan()
		makeplan.position = location
		makeplan.orientation = pose.orientation
		try:
			path = baseCheckPath(makeplan)
			if (len(path.path) >0):
				goal.target_pose.header.stamp = rospy.Time.now()
				self.client_base.send_goal(goal, self.client_base_done_callback, self.client_base_active_callback, self.client_base_feedback_callback)
				rospy.loginfo("NavigationServer: Path OK.")
				rospy.logdebug("NavigationServer: Location goal is - " + str(self.next_location))
			else:
				rospy.logdebug("NavigationServer: Could not compute path to " + str(self.next_location) + ".")
				self.next_location = None
				self.base_canceled = True
		except rospy.ServiceException as exc:
			rospy.logdebug("NavigationServer: Make plan service did not process request: " + str(exc))
			self.next_location = None
			self.base_canceled = True
			return


	# The robot base starts to wander and keeps wandering until it is preempted
	def start_wandering(self):
		# Tell base to start wandering
		rospy.logdebug("NavigationServer: Waiting for base wandering service.")
		rospy.wait_for_service("/rosarnl_node/wander")
		rospy.logdebug("NavigationServer: wandering service available.")
		try:
			baseStartWandering = rospy.ServiceProxy("/rosarnl_node/wander", Empty)
			baseStartWandering()
		except rospy.ServiceException, e:
			rospy.logdebug("NavigationServer: wandering service error - " + str(e))

		started_waiting = time.time() # Prevent eternal looping
		feedback_timer = time.time()
		while not rospy.is_shutdown():
			if self.server_navigation.is_preempt_requested():
				self.client_base.cancel_all_goals()
				try:
					baseStop = rospy.ServiceProxy("/rosarnl_node/stop", Empty)
					baseStop()
				except rospy.ServiceException, e:
					rospy.logdebug("NavigationServer: stop service error - " + str(e))
				self.server_navigation.set_preempted()
				return
			# Check wandering timeout to prevent eternal looping
			if (time.time() - started_waiting > self.TIMEOUT_WANDERING):
				rospy.loginfo("NavigationServer: Timed out, aborting.")
				self.client_base.cancel_all_goals()
				try:
					baseStop = rospy.ServiceProxy("/rosarnl_node/stop", Empty)
					baseStop()
				except rospy.ServiceException, e:
					rospy.logdebug("NavigationServer: stop service error - " + str(e))
				self.server_navigation.set_aborted()
				return
			if (time.time() - feedback_timer > self.EMOTIONAL_FEEDBACK_CYCLE): # give emotional feedback every 15 seconds
				self.send_emotion(pleasure=0.01, arousal=0.01, dominance=0.01)
				feedback_timer = time.time()
			self.SERVER_RATE.sleep()
		# set terminal goal status in case of shutdown
		self.server_navigation.set_aborted()



	def navigation_dock(self): # NOT YET FINISHED
		# Tell base to navigate to charger and dock
		rospy.loginfo("NavigationServer: Waiting for base dock service.")
		rospy.wait_for_service("/rosarnl_node/dock")
		rospy.loginfo("NavigationServer: dock service available.")
		try: 
			baseStartDocking = rospy.ServiceProxy("/rosarnl_node/dock", Empty)
			baseStartDocking()
		except rospy.ServiceException, exc:
			rospy.loginfo("NavigationServer: dock service error - " + str(exc))

		# Add wait for confirmation before continue
		###############
		
		started_waiting  = time.time() # Prevent eternal looping
		while not rospy.is_shutdown():
			if self.server_navigation.is_preempt_requested():
				self.client_base.cancel_all_goals()
				try:
					baseStopDocking = rospy.ServiceProxy("/rosarnl_node/stop", Empty)
					baseStopDocking()
				except rospy.ServiceException, exc:
					rospy.logdebug("NavigationServer: stop docking service error - " + str(exc))
				self.server_navigation.set_preempted()
				return

			# Check state of docking, and act accordingly if problems
			elif False:
			# CHECK STATE AND RESULT!
				return

			self.SERVER_RATE.sleep()
		# set terminal goal status in case of shutdown
		self.server_navigation.set_aborted()



	# Sends the emotional change to the controller
	def send_emotion(self, pleasure, arousal, dominance):
		msg = EmotionalFeedback()
		msg.delta_pleasure = pleasure
		msg.delta_arousal = arousal
		msg.delta_dominance = dominance 
		self.emotion_publisher.publish(msg)