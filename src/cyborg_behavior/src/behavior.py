#!/usr/bin/env python
"""Created by Areg Babayan on 25/2/2019.
Copyright (C) 2019 Areg Babayan. ALl rights reserved."""
__author__      = "Areg Babayan"
__copyright__   = "Copyright (C) 2019 Areg Babayan"
__license__     = "BSD"
__version__     = "0.0.1"
__all__         = [] 


import rospy
from behaviorserver import BehaviorServer

def main():
    rospy.init_node("cyborg_behavior")
    behavior_server = BehaviorServer()
    rospy.spin()



if __name__ =="__main__":
    print("Cyborg Behavior: Starting Program")
    main()
    print("Cyborg Behavior: Shutting Down")