#!/usr/bin/env python

"""Created by Thomas Rostrup Andersen on 11/11/2016.
Copyright (C) 2016 Thomas Rostrup Andersen. All rights reserved."""

import rospy
import sys
import os
import datetime
from navigationserver import NavigationServer
from databasehandler import DatabaseHandler

__author__ = "Thomas Rostrup Andersen"
__copyright__ = "Copyright (C) 2016 Thomas Rostrup Andersen"
#__license__ = ""
__version__ = "0.0.2"
__all__ = []

"""Cyborg Navigation Module"""
def main():

    homedir = os.path.expanduser("~")
    path = homedir + "/navigation.db"

    if (os.path.exists(path) == False):
        database_handler = DatabaseHandler(filename=path)
        database_handler.create()
        database_handler.add_location(location_name="entrance", robot_map_name="glassgarden.map", x=-18.440, y=6.500, z=0, p=0, j=0, r=2, threshold=3, crowded=False,environment=-0.10)
        database_handler.add_location(location_name="home", robot_map_name="glassgarden.map", x=-29.500, y=8.700, z=0, p=0, j=0, r=2, threshold=3, crowded=False, environment=0.20)
        database_handler.add_location(location_name="waiting area", robot_map_name="glassgarden.map", x=-33.600, y=10.600, z=0, p=0, j=0, r=2, threshold=3, crowded=False, environment=-0.10)
        database_handler.add_location(location_name="cafeteria", robot_map_name="glassgarden.map", x=-33.090, y=-55.700, z=0, p=0, j=0, r=2, threshold=3, crowded=True, environment=0.20)
        database_handler.add_location(location_name="elevator", robot_map_name="glassgarden.map", x=-29.500, y=-50.200, z=0, p=0, j=0, r=2, threshold=3, crowded=True, environment=-0.02)
        database_handler.add_location(location_name="entrance 2", robot_map_name="glassgarden.map", x=-18.300, y=-66.000, z=0, p=0, j=0, r=33, threshold=3, crowded=True, environment=0.05)
        database_handler.add_location(location_name="information", robot_map_name="glassgarden.map", x=-33.490, y=1.160, z=0, p=0, j=0, r=2, threshold=3, crowded=True, environment=0.05)
        database_handler.add_location(location_name="el5", robot_map_name="glassgarden.map", x=-33.720, y=-32.500, z=0, p=0, j=0, r=-2, threshold=3, crowded=True, environment=0.00)
        database_handler.add_location(location_name="el6", robot_map_name="glassgarden.map", x=-30.300, y=-12.840, z=0, p=0, j=0, r=2, threshold=3, crowded=True, environment=0.00)
        database_handler.add_location(location_name="bridge", robot_map_name="glassgarden.map", x=-28.090, y=-63.500, z=0, p=0, j=0, r=2, threshold=3, crowded=True, environment=0.00)
        

        database_handler.add_event(event_name="welcome_time", location_name="entrance", start_date=datetime.datetime(2018, 1, 18, 11, 0, 0, 1), end_date=datetime.datetime(2018, 1, 18, 11, 4, 0, 1), ignore=False)
        database_handler.add_event(event_name="dinner_time", location_name="cafeteria", start_date=datetime.datetime(2018, 1, 18, 15, 0, 7, 1) , end_date=datetime.datetime(2018, 1, 18, 15, 59, 0, 1), ignore=False)
        database_handler.add_event(event_name="wait_time", location_name="waiting area", start_date=datetime.datetime(2018, 1, 18, 8, 0, 1, 1) , end_date=datetime.datetime(2018, 1, 18, 8, 46, 0, 1), ignore=False)
        database_handler.add_event(event_name="lunch_time", location_name="cafeteria", start_date=datetime.datetime(2018, 1, 18, 7, 0, 1, 1) , end_date=datetime.datetime(2018, 1, 18, 7, 0, 0, 1), ignore=False)
        database_handler.add_event(event_name="goodbye_time", location_name="entrance 2", start_date=datetime.datetime(2018, 1, 18, 9, 0, 1, 1), end_date=datetime.datetime(2018, 1, 18, 9, 0, 0, 1), ignore=False)

    rospy.init_node("cyborg_navigation")
    navigation_server = NavigationServer(database_file=path)
    rospy.spin()

if __name__ == "__main__":
    print("Cyborg Navigation: Starting Program...")

    if sys.version_info < (2,5):
        print("Cyborg Navigation: Running Python version " + str(sys.version_info.major) + "." + str(sys.version_info.minor) + "." + str(sys.version_info.micro) + " (Python version 2.5 or grater is required)...")
        exit()

    main()

    print("Cyborg Navigation: End of Program...")

