#!/usr/bin/env python
__author__      = "Areg Babayan"
__copyright__   = "Copyright (C) 2019 Areg Babayan"
__license__     = "BSD"
__version__     = "0.0.4"
__all__         = [] 


import rospy
import sys
import os
from primary_states_server import PrimaryStatesServer

def main():
    homedir = os.path.expanduser("~")
    path = homedir + "/navigation.db"

    rospy.init_node("cyborg_primary_states")
    primary_states_server = PrimaryStatesServer(database_file = path)
    rospy.spin()



if __name__ =="__main__":
    print("Cyborg Primary States: Starting Program")
    main()
    print("Cyborg Primary States: Shutting Down")