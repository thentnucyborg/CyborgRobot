#!/usr/bin/env python

import csv
import rospy
from collections import namedtuple
from itertools import ifilter, imap
from geometry_msgs.msg import Pose2D
from cyborg_msgs.msg import GoalWithHeading
from cyborg_msgs.srv import AvailableGoals, AvailableGoalsResponse

NAME = 'available_goals_server'

GoalRecord = namedtuple('GoalRecord', 'cairn type x y theta comment icon name')


class AvailableGoalsHandler():
    def __init__(self):
        rospy.init_node(NAME)

        srv_name = '/cyborg/nav/get_available_goals'
        rospy.Service(srv_name, AvailableGoals, self.goals_cb)

        # Will be created on demand
        self.response = None

        rospy.spin()

    def create_response(self, filename):
        """ Create AvailableGoalsResponse from map file """
        response = AvailableGoalsResponse()
        with open(filename) as f:
            # Get only lines with the keyword 'Goal' or 'GoalWithHeading'
            lines = ifilter(lambda line: 'Cairn: Goal' in line, f)

            # Parse lines, and create a GoalRecord, then create a
            # GoalWithHeading message for each waypoint
            reader = csv.reader(lines, delimiter=' ')
            for rec in imap(GoalRecord._make, reader):
                pose = Pose2D()
                pose.x = float(rec.x) / 1000
                pose.y = float(rec.y) / 1000
                pose.theta = float(rec.theta)

                goal = GoalWithHeading()
                goal.name = rec.name
                goal.comment = rec.comment
                goal.position = pose

                response.goals.append(goal)

        return response

    def goals_cb(self, data):
        """ Return list of available goals """
        if self.response is None:
            map_file = rospy.get_param('~map_file')
            self.response = self.create_response(map_file)
        return self.response


if __name__ == "__main__":
    rospy.set_param('~map_file', '/home/mortenmj/maps/maps/glassgarden.map')
    AvailableGoalsHandler()
