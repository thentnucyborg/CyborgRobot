#!/usr/bin/env python

from __future__ import print_function

import rospy
from cyborg_msgs.srv import DistanceToGoal
from cyborg_msgs.srv import ClosestGoal, ClosestGoalResponse
from cyborg_util import Locations

NAME = 'closest_goal_server'


class ClosestGoalHandler():
    def __init__(self):
        rospy.init_node(NAME)

        # Distance to goal service proxy
        srv_name = '/cyborg/nav/get_distance_to_goal'
        rospy.wait_for_service(srv_name)
        try:
            self._dist_svc = rospy.ServiceProxy(srv_name, DistanceToGoal)
        except rospy.ServiceException as e:
            rospy.logerr("Service call to %s failed: %s" % (srv_name, e))

        # Closest goal service
        srv_name = '/cyborg/nav/get_closest_goal'
        rospy.Service(srv_name, ClosestGoal, self.goal_cb)

        self.locations = Locations()

        rospy.spin()

    # Get the closest point of interest
    def goal_cb(self, data):
        # Get the distances to each location.
        # If no path is found, the path will be empty
        dist = dict((k, self._dist_svc(v).distance) for k, v in self.locations)

        # Get entry with the shortest distance
        name, dist = min(dist.iteritems(), key=lambda p: p[1])

        return ClosestGoalResponse(name=name, distance=dist)


if __name__ == "__main__":
    ClosestGoalHandler()
