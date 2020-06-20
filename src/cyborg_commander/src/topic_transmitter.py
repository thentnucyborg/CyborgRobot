#!/usr/bin/env python

__author__ = "Casper Nilsen"
__copyright__ = "Copyright (C) 2020 Casper Nilsen"
__license__ = "BSD"
__version__ = "0.1.1"
__all__ = []


import rospy
import json
import sys
import binascii
import datetime
import threading
import time
from pymongo import MongoClient 
from rospy_message_converter import json_message_converter

# ROS msgs
from std_msgs.msg import String, Bool, Int32, Int8, Float32, Float64
from rosgraph_msgs.msg import Log
from cyborg_controller.msg import StateMachineAction, StateMachineGoal, SystemState, EmotionalState, StateMachineActionResult, EmotionalFeedback, StateMachineActionGoal, StateMachineActionFeedback
from cyborg_navigation.msg import NavigationAction, NavigationGoal, NavigationActionFeedback, NavigationActionGoal, NavigationActionResult
from actionlib_msgs.msg import GoalStatusArray, GoalID
from smach_msgs.msg import SmachContainerStatus,SmachContainerInitialStatusCmd,SmachContainerStructure
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Pose2D, PoseStamped, Twist, PolygonStamped, PoseArray, PointStamped
from sensor_msgs.msg import LaserScan, PointCloud
from move_base_msgs.msg import MoveBaseActionFeedback, MoveBaseActionResult, MoveBaseActionGoal
from rosbridge_msgs.msg import ConnectedClients
from sensor_msgs.msg import JointState, PointCloud2
from nav_msgs.msg import OccupancyGrid, MapMetaData, Path, Odometry
from map_msgs.msg import OccupancyGridUpdate
from tf2_msgs.msg import TFMessage
from dynamic_reconfigure.msg import ConfigDescription, Config
from rosaria.msg import BumperState
class TopicTransmitter():
    """TopicTransmitter"""

    def __init__(self):
        rospy.loginfo("Cyborg Commander: transmitter initializing")
        
        # self.subscriber_amcl_parameter_descriptions = rospy.Subscriber('/amcl/parameter_descriptions', ConfigDescription, callback = self.callback_send_periodic, queue_size = 10)
        # self.subscriber_amcl_parameter_updates = rospy.Subscriber('/amcl/parameter_updates', Config, callback = self.callback_send_periodic, queue_size = 10)
        # self.subscriber_amcl_pose = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, callback = self.callback_send_periodic, queue_size = 10)
        # self.subscriber_clicked_point = rospy.Subscriber('/clicked_point', PointStamped, callback = self.callback_send_periodic, queue_size = 10)
        self.subscriber_client_count = rospy.Subscriber('/client_count', Int32, callback = self.callback_send_periodic, queue_size = 10)
        # self.subscriber_cmd_vel = rospy.Subscriber('/cmd_vel', Twist, callback = self.callback_send_periodic, queue_size = 10)
        # self.subscriber_connected_clients = rospy.Subscriber('/connected_clients', ConnectedClients, callback = self.callback_send_periodic, queue_size = 10)
        # self.subscriber_controller_viewer_smach_container_init = rospy.Subscriber('/controller_viewer/smach/container_init', SmachContainerInitialStatusCmd, callback = self.callback_send_periodic, queue_size = 10)
        # self.subscriber_controller_viewer_smach_container_status = rospy.Subscriber('/controller_viewer/smach/container_status', SmachContainerStatus, callback = self.callback_send_periodic, queue_size = 10)
        # self.subscriber_controller_viewer_smach_container_structure = rospy.Subscriber('/controller_viewer/smach/container_structure', SmachContainerStructure, callback = self.callback_send_periodic, queue_size = 10)
        # self.subscriber_cyborg_audio_feedback_playback = rospy.Subscriber('/cyborg_audio/feedback_playback', String, callback = self.callback_send_periodic, queue_size = 10)
        # self.subscriber_cyborg_audio_feedback_text_to_speech = rospy.Subscriber('/cyborg_audio/feedback_text_to_speech', String, callback = self.callback_send_periodic, queue_size = 10)
        # self.subscriber_cyborg_audio_playback = rospy.Subscriber('/cyborg_audio/playback', String, callback = self.callback_send_periodic, queue_size = 10)
        # self.subscriber_cyborg_audio_text_to_speech = rospy.Subscriber('/cyborg_audio/text_to_speech', String, callback = self.callback_send_periodic, queue_size = 10)
        # self.subscriber_cyborg_behavior_cancel = rospy.Subscriber('/cyborg_behavior/cancel', GoalID, callback = self.callback_send_periodic, queue_size = 10)
        # self.subscriber_cyborg_behavior_command_location = rospy.Subscriber('/cyborg_behavior/command_location', String, callback = self.callback_send_periodic, queue_size = 10)
        # self.subscriber_behaviour_dynamic_behavior = rospy.Subscriber("/cyborg_behavior/dynamic_behavior", String, callback = self.callback_send_periodic, queue_size = 10)
        # self.subscriber_cyborg_behavior_feedback = rospy.Subscriber('/cyborg_behavior/feedback', StateMachineActionFeedback, callback = self.callback_send_periodic, queue_size = 10)
        # self.subscriber_cyborg_behavior_goal = rospy.Subscriber('/cyborg_behavior/goal', StateMachineActionGoal, callback = self.callback_send_periodic, queue_size = 10)
        # self.subscriber_cyborg_behavior_result = rospy.Subscriber('/cyborg_behavior/result', StateMachineActionResult, callback = self.callback_send_periodic, queue_size = 10)
        # self.subscriber_cyborg_behavior_status = rospy.Subscriber('/cyborg_behavior/status', GoalStatusArray, callback = self.callback_send_periodic, queue_size = 10)
        # self.subscriber_controller_emotional_controller = rospy.Subscriber("/cyborg_controller/emotional_controller", String, callback = self.callback_send_periodic, queue_size = 10)
        # self.subscriber_cyborg_controller_emotional_feedback = rospy.Subscriber('/cyborg_controller/emotional_feedback', EmotionalFeedback, callback = self.callback_send_periodic, queue_size = 10)
        # self.subscriber_cyborg_controller_emotional_state = rospy.Subscriber('/cyborg_controller/emotional_state', EmotionalState, callback = self.callback_send_periodic, queue_size = 10)
        # self.subscriber_cyborg_controller_register_event = rospy.Subscriber('/cyborg_controller/register_event', String, callback = self.callback_send_periodic, queue_size = 10)
        self.subscriber_cyborg_controller_state_change = rospy.Subscriber('/cyborg_controller/state_change', SystemState, callback = self.callback_send_periodic, queue_size = 10)
        # self.subscriber_navigation_current_location = rospy.Subscriber("/cyborg_navigation/current_location", String, callback = self.callback_send_periodic, queue_size = 10)
        # self.subscriber_navigation_cancel = rospy.Subscriber("/cyborg_navigation/navigation/cancel", GoalID, callback = self.callback_send_periodic, queue_size = 10)
        # self.subscriber_navigation_feedback = rospy.Subscriber("/cyborg_navigation/navigation/feedback", NavigationActionFeedback, callback = self.callback_send_periodic, queue_size = 10)
        # self.subscriber_navigation_goal = rospy.Subscriber("/cyborg_navigation/navigation/goal", NavigationActionGoal, callback = self.callback_send_periodic, queue_size = 10)
        # self.subscriber_navigation_result = rospy.Subscriber("/cyborg_navigation/navigation/result", NavigationActionResult, callback = self.callback_send_periodic, queue_size = 10)
        # self.subscriber_navigation_status = rospy.Subscriber("/cyborg_navigation/navigation/status", GoalStatusArray, callback = self.callback_send_periodic, queue_size = 10)
        # self.subscriber_primarystates_cancel = rospy.Subscriber("/cyborg_primary_states/cancel", GoalID, callback = self.callback_send_periodic, queue_size = 10)
        # self.subscriber_primarystates_feedback = rospy.Subscriber("/cyborg_primary_states/feedback", StateMachineActionFeedback, callback = self.callback_send_periodic, queue_size = 10)
        # self.subscriber_primarystates_goal = rospy.Subscriber("/cyborg_primary_states/goal", StateMachineActionGoal, callback = self.callback_send_periodic, queue_size = 10)
        # self.subscriber_primarystates_result = rospy.Subscriber("/cyborg_primary_states/result", StateMachineActionResult, callback = self.callback_send_periodic, queue_size = 10)
        # self.subscriber_primarystates_status = rospy.Subscriber("/cyborg_primary_states/status", GoalStatusArray, callback = self.callback_send_periodic, queue_size = 10)
        # self.subscriber_cyborg_visual_domecontrol = rospy.Subscriber('/cyborg_visual/domecontrol', String, callback = self.callback_send_periodic, queue_size = 10)
        # self.subscriber_initialpose = rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, callback = self.callback_send_periodic, queue_size = 10)
        # self.subscriber_joint_states = rospy.Subscriber('/joint_states', JointState, callback = self.callback_send_periodic, queue_size = 10)
        # self.subscriber_map = rospy.Subscriber('/map', OccupancyGrid, callback = self.callback_send_periodic, queue_size = 10)
        # self.subscriber_map_metadata = rospy.Subscriber('/map_metadata', MapMetaData, callback = self.callback_send_periodic, queue_size = 10)
        # self.subscriber_map_updates = rospy.Subscriber('/map_updates', OccupancyGridUpdate, callback = self.callback_send_periodic, queue_size = 10)
        # self.subscriber_move_base_cancel = rospy.Subscriber('/move_base/cancel', GoalID, callback = self.callback_send_periodic, queue_size = 10)
        # self.subscriber_move_base_current_goal = rospy.Subscriber('/move_base/current_goal', PoseStamped, callback = self.callback_send_periodic, queue_size = 10)
        # self.subscriber_move_base_feedback = rospy.Subscriber('/move_base/feedback', MoveBaseActionFeedback, callback = self.callback_send_periodic, queue_size = 10)
        # self.subscriber_move_base_global_costmap_costmap = rospy.Subscriber('/move_base/global_costmap/costmap', OccupancyGrid, callback = self.callback_send_periodic, queue_size = 10)
        # self.subscriber_move_base_global_costmap_costmap_updates = rospy.Subscriber('/move_base/global_costmap/costmap_updates', OccupancyGridUpdate, callback = self.callback_send_periodic, queue_size = 10)
        # self.subscriber_move_base_global_costmap_footprint = rospy.Subscriber('/move_base/global_costmap/footprint', PolygonStamped, callback = self.callback_send_periodic, queue_size = 10)
        # self.subscriber_move_base_global_costmap_inflation_layer_parameter_descriptions = rospy.Subscriber('/move_base/global_costmap/inflation_layer/parameter_descriptions', ConfigDescription, callback = self.callback_send_periodic, queue_size = 10)
        # self.subscriber_move_base_global_costmap_inflation_layer_parameter_updates = rospy.Subscriber('/move_base/global_costmap/inflation_layer/parameter_updates', Config, callback = self.callback_send_periodic, queue_size = 10)
        # self.subscriber_move_base_global_costmap_obstacle_layer_parameter_descriptions = rospy.Subscriber('/move_base/global_costmap/obstacle_layer/parameter_descriptions', ConfigDescription, callback = self.callback_send_periodic, queue_size = 10)
        # self.subscriber_move_base_global_costmap_obstacle_layer_parameter_updates = rospy.Subscriber('/move_base/global_costmap/obstacle_layer/parameter_updates', Config, callback = self.callback_send_periodic, queue_size = 10)
        # self.subscriber_move_base_global_costmap_parameter_descriptions = rospy.Subscriber('/move_base/global_costmap/parameter_descriptions', ConfigDescription, callback = self.callback_send_periodic, queue_size = 10)
        # self.subscriber_move_base_global_costmap_parameter_updates = rospy.Subscriber('/move_base/global_costmap/parameter_updates', Config, callback = self.callback_send_periodic, queue_size = 10)
        # self.subscriber_move_base_global_costmap_static_layer_parameter_descriptions = rospy.Subscriber('/move_base/global_costmap/static_layer/parameter_descriptions', ConfigDescription, callback = self.callback_send_periodic, queue_size = 10)
        # self.subscriber_move_base_global_costmap_static_layer_parameter_updates = rospy.Subscriber('/move_base/global_costmap/static_layer/parameter_updates', Config, callback = self.callback_send_periodic, queue_size = 10)
        # self.subscriber_move_base_goal = rospy.Subscriber('/move_base/goal', MoveBaseActionGoal, callback = self.callback_send_periodic, queue_size = 10)
        # self.subscriber_move_base_local_costmap_costmap = rospy.Subscriber('/move_base/local_costmap/costmap', OccupancyGrid, callback = self.callback_send_periodic, queue_size = 10)
        # self.subscriber_move_base_local_costmap_costmap_updates = rospy.Subscriber('/move_base/local_costmap/costmap_updates', OccupancyGridUpdate, callback = self.callback_send_periodic, queue_size = 10)
        # self.subscriber_move_base_local_costmap_footprint = rospy.Subscriber('/move_base/local_costmap/footprint', PolygonStamped, callback = self.callback_send_periodic, queue_size = 10)
        # self.subscriber_move_base_local_costmap_inflation_layer_parameter_descriptions = rospy.Subscriber('/move_base/local_costmap/inflation_layer/parameter_descriptions', ConfigDescription, callback = self.callback_send_periodic, queue_size = 10)
        # self.subscriber_move_base_local_costmap_inflation_layer_parameter_updates = rospy.Subscriber('/move_base/local_costmap/inflation_layer/parameter_updates', Config, callback = self.callback_send_periodic, queue_size = 10)
        # self.subscriber_move_base_local_costmap_obstacle_layer_parameter_descriptions = rospy.Subscriber('/move_base/local_costmap/obstacle_layer/parameter_descriptions', ConfigDescription, callback = self.callback_send_periodic, queue_size = 10)
        # self.subscriber_move_base_local_costmap_obstacle_layer_parameter_updates = rospy.Subscriber('/move_base/local_costmap/obstacle_layer/parameter_updates', Config, callback = self.callback_send_periodic, queue_size = 10)
        # self.subscriber_move_base_local_costmap_parameter_descriptions = rospy.Subscriber('/move_base/local_costmap/parameter_descriptions', ConfigDescription, callback = self.callback_send_periodic, queue_size = 10)
        # self.subscriber_move_base_local_costmap_parameter_updates = rospy.Subscriber('/move_base/local_costmap/parameter_updates', Config, callback = self.callback_send_periodic, queue_size = 10)
        # self.subscriber_move_base_NavfnROS_plan = rospy.Subscriber('/move_base/NavfnROS/plan', Path, callback = self.callback_send_periodic, queue_size = 10)
        # self.subscriber_move_base_parameter_descriptions = rospy.Subscriber('/move_base/parameter_descriptions', ConfigDescription, callback = self.callback_send_periodic, queue_size = 10)
        # self.subscriber_move_base_parameter_updates = rospy.Subscriber('/move_base/parameter_updates', Config, callback = self.callback_send_periodic, queue_size = 10)
        # self.subscriber_move_base_result = rospy.Subscriber('/move_base/result', MoveBaseActionResult, callback = self.callback_send_periodic, queue_size = 10)
        # self.subscriber_move_base_simple_goal = rospy.Subscriber('/move_base_simple/goal', PoseStamped, callback = self.callback_send_periodic, queue_size = 10)
        # self.subscriber_move_base_status = rospy.Subscriber('/move_base/status', GoalStatusArray, callback = self.callback_send_periodic, queue_size = 10)
        # self.subscriber_move_base_TrajectoryPlannerROS_cost_cloud = rospy.Subscriber('/move_base/TrajectoryPlannerROS/cost_cloud', PointCloud2, callback = self.callback_send_periodic, queue_size = 10)
        # self.subscriber_move_base_TrajectoryPlannerROS_global_plan = rospy.Subscriber('/move_base/TrajectoryPlannerROS/global_plan', Path, callback = self.callback_send_periodic, queue_size = 10)
        # self.subscriber_move_base_TrajectoryPlannerROS_local_plan = rospy.Subscriber('/move_base/TrajectoryPlannerROS/local_plan', Path, callback = self.callback_send_periodic, queue_size = 10)
        # self.subscriber_move_base_TrajectoryPlannerROS_parameter_descriptions = rospy.Subscriber('/move_base/TrajectoryPlannerROS/parameter_descriptions', ConfigDescription, callback = self.callback_send_periodic, queue_size = 10)
        # self.subscriber_move_base_TrajectoryPlannerROS_parameter_updates = rospy.Subscriber('/move_base/TrajectoryPlannerROS/parameter_updates', Config, callback = self.callback_send_periodic, queue_size = 10)
        # self.subscriber_odom = rospy.Subscriber('/odom', Odometry, callback = self.callback_send_periodic, queue_size = 10)
        # self.subscriber_particlecloud = rospy.Subscriber('/particlecloud', PoseArray, callback = self.callback_send_periodic, queue_size = 10)
        # self.subscriber_robot_pose = rospy.Subscriber('/robot_pose', Pose, callback = self.callback_send_periodic, queue_size = 10)
        self.subscriber_RosAria_battery_recharge_state = rospy.Subscriber('/RosAria/battery_recharge_state', Int8, callback = self.callback_send_periodic, queue_size = 10)
        self.subscriber_RosAria_battery_state_of_charge = rospy.Subscriber('/RosAria/battery_state_of_charge', Float32, callback = self.callback_send_periodic, queue_size = 10)
        self.subscriber_RosAria_battery_voltage = rospy.Subscriber('/RosAria/battery_voltage', Float64, callback = self.callback_send_periodic, queue_size = 10)
        # self.subscriber_RosAria_bumper_state = rospy.Subscriber('/RosAria/bumper_state', BumperState, callback = self.callback_send_periodic, queue_size = 10)
        self.subscriber_RosAria_motors_state = rospy.Subscriber('/RosAria/motors_state', Bool, callback = self.callback_send_periodic, queue_size = 10)
        # self.subscriber_RosAria_parameter_descriptions = rospy.Subscriber('/RosAria/parameter_descriptions', ConfigDescription, callback = self.callback_send_periodic, queue_size = 10)
        # self.subscriber_RosAria_parameter_updates = rospy.Subscriber('/RosAria/parameter_updates', Config, callback = self.callback_send_periodic, queue_size = 10)
        # self.subscriber_RosAria_sim_S3Series_1_laserscan = rospy.Subscriber('/RosAria/sim_S3Series_1_laserscan', LaserScan, callback = self.callback_send_periodic, queue_size = 10)
        # self.subscriber_RosAria_sim_S3Series_1_pointcloud = rospy.Subscriber('/RosAria/sim_S3Series_1_pointcloud', PointCloud, callback = self.callback_send_periodic, queue_size = 10)
        # self.subscriber_RosAria_sonar = rospy.Subscriber('/RosAria/sonar', PointCloud, callback = self.callback_send_periodic, queue_size = 10)
        # self.subscriber_RosAria_sonar_pointcloud2 = rospy.Subscriber('/RosAria/sonar_pointcloud2', PointCloud2, callback = self.callback_send_periodic, queue_size = 10)
        # self.subscriber_rosout = rospy.Subscriber('/rosout', Log, callback = self.callback_send_periodic, queue_size = 10)
        # self.subscriber_rosout_agg = rospy.Subscriber('/rosout_agg', Log, callback = self.callback_send_periodic, queue_size = 10)
        # self.subscriber_tf = rospy.Subscriber('/tf', TFMessage, callback = self.callback_send_periodic, queue_size = 10)
        # self.subscriber_tf_static = rospy.Subscriber('/tf_static', TFMessage, callback = self.callback_send_periodic, queue_size = 10)
        # self.subscriber_behaviour_status = rospy.Subscriber("/cyborg_behavior/status", GoalStatusArray, callback = self.callback_send_periodic, queue_size = 10)

        rospy.loginfo("Cyborg Commander: transmitter initialized")

    # transmits data immediately
    def callback_send(self, message):
        if message != None:
            topic = message._connection_header["topic"]
            self.transmit_data(topic)

    # stores data and transmits latest topic data periodically.
    def callback_send_periodic(self, message):
        if message != None:
            topic = message._connection_header["topic"]
            self.__dict__[topic] = message

            if not topic + "/Thread" in self.__dict__:
                self.__dict__[topic + "/Thread"] = threading.Thread(target=self.transmit_loop, args=(topic,))
                self.__dict__[topic + "/Thread"].daemon = True # Terminates thread when main thread terminate
                self.__dict__[topic + "/Thread"].start()
    
    def transmit_loop(self, topic):
        # Rate of transmission, rosparam: /cyborg_commander/transmitter_interval
        interval = 10

        while not rospy.is_shutdown():
            self.transmit_data(topic)
            
            # change interval if params for specific or shared intervals are set.
            # specific interval is prioritized.
            if rospy.has_param(topic + "/transmission_interval"):
                topic_interval = rospy.get_param(topic + "/transmission_interval")
                if interval != topic_interval:
                    interval = topic_interval
                    rospy.loginfo("Commander: interval for topic: {0} set to: {1}".format(topic, interval))
            elif rospy.has_param("/cyborg_commander/transmitter_interval"):
                shared_interval = rospy.get_param("/cyborg_commander/transmitter_interval")
                if interval != shared_interval:
                    interval = shared_interval
                    rospy.loginfo("Commander: interval for all topics set to: {0}. I am transmitter for topic: {1}".format(interval, topic))
            
            rospy.sleep(interval)

    
    def transmit_data(self, topic):
        topic_name = topic
        if topic in self.__dict__:
            json_data = json_message_converter.convert_ros_message_to_json(self.__dict__[topic])

            client = MongoClient("mongodb://cyborg:FerdigKlar3@cluster0-shard-00-00-hmfl4.azure.mongodb.net:27017,cluster0-shard-00-01-hmfl4.azure.mongodb.net:27017,cluster0-shard-00-02-hmfl4.azure.mongodb.net:27017/test?ssl=true&replicaSet=Cluster0-shard-0&authSource=admin&retryWrites=true&w=majority")
            
            # Access database
            mydb = client['cyborg_data']

            # MongoDB Doesn't like storing topics with "/" in them.
            topic = topic.replace("/","__")
            # Access collection of the database
            db_collection = mydb[topic]
            
            # Deserialize json object to Python Object
            rec = json.loads(json_data)

            # Add current time to record
            now = datetime.datetime.utcnow()
            rec["date"] = now
            
            # insert the data into database
            try:
                db_collection.insert(rec)

                ## Debug
                # rospy.loginfo("Commander: Sent data to mongodb for topic {0}, data: {1}".format(topic_name, str(self.__dict__[topic_name])))
            except ValueError:
                rospy.logerr("Commander: Error sending to mongodb for topic: {0}, error: {1}".format(topic_name,ValueError))
