#!/usr/bin/env python

import os
import rospy
from rosarnl.srv import LoadMapFile

homedir = os.path.expanduser("~")


def load_map():
    filename = rospy.get_param('~map_file')

    rospy.wait_for_service('/rosarnl_node/load_map_file')
    s = rospy.ServiceProxy('/rosarnl_node/load_map_file', LoadMapFile)

    try:
        s(filename)
    except rospy.ServiceException as e:
        print("Service failed: %s" % e)


if __name__ == "__main__":
    rospy.set_param('~map_file', homedir + '/catkin_ws/src/maps/maps/glassgarden.map')
    load_map()
