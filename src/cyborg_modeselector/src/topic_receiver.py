#!/usr/bin/env python

__author__ = "Casper Nilsen"
__copyright__ = "Copyright (C) 2019 Casper Nilsen"
__license__ = "BSD"
__version__ = "0.0.1"
__all__ = []

import rospy
from std_msgs.msg import Bool

class TopicReceiver():
    """TopicReceiver"""

    def __init__(self):
        # self.publisher_behaviour_state = rospy.Publisher(rospy.get_name() + "/behaviour_state", Bool, queue_size=10)
        while not rospy.is_shutdown():
            # self.publisher_behaviour_state.publish(False)
            rospy.sleep(10)