#!/usr/bin/env python
__author__      = "Casper Nilsen"
__copyright__   = "Copyright (C) 2019 Casper Nilsen"
__license__     = "BSD"
__version__     = "0.0.1"
__all__         = [] 


import rospy
from topic_transmitter import TopicTransmitter
from topic_receiver import TopicReceiver
# from rosbridge import Rosbridge
# from livefeed import Livefeed

def main():
    rospy.init_node("cyborg_commander")
    # livefeed = Livefeed()
    TopicTransmitter()
    TopicReceiver()
    rospy.spin()


if __name__ =="__main__":
    rospy.loginfo("Cyborg commander: Starting Program")
    main()
    rospy.loginfo("Cyborg commander: Shutting Down")