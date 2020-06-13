#!/usr/bin/env python

from __future__ import print_function

import rospy
from cyborg_msgs.srv import DistanceToGoal, DistanceToGoalResponse
from cyborg_types import Path, Pose
from rosarnl.srv import MakePlan

NAME = 'distance_to_goal_server'


class DistanceToGoalHandler():
    def __init__(self):
        rospy.init_node(NAME)

        srv_name = '/cyborg/nav/get_distance_to_goal'
        rospy.Service(srv_name, DistanceToGoal, self.__distance_cb)

        srv_name = '/rosarnl_node/make_plan'
        rospy.wait_for_service(srv_name)
        try:
            self._plan_svc = rospy.ServiceProxy(srv_name, MakePlan)
        except rospy.ServiceException as e:
            rospy.logerr("Service call to %s failed: %s" % (srv_name, e))

        rospy.spin()

    def __distance_cb(self, data):
        # Create a Pose message to validate our input data
        goal = Pose.from_pose(data.goal)

        # Get a path to the given goal
        path = Path.from_posearray(self._plan_svc(goal).path)

        # Calculate distance if a path was found, or return inf
        distance = path.length if path else float('inf')

        rospy.loginfo(path.length)

        return DistanceToGoalResponse(distance=distance)


if __name__ == "__main__":
    DistanceToGoalHandler()
