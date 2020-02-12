#!/usr/bin/env python
__author__      = "Casper Nilsen"
__copyright__   = "Copyright (C) 2019 Casper Nilsen"
__license__     = "BSD"
__version__     = "0.0.1"
__all__         = [] 


import rospy

from topic_transmitter import TopicTransmitter
# from livefeed import Livefeed

def main():
    rospy.init_node("cyborg_gui")
    # livefeed = Livefeed()
    topicTransmitter = TopicTransmitter()
    rospy.spin()


if __name__ =="__main__":
    print("Cyborg gui: Starting Program")
    main()
    print("Cyborg gui: Shutting Down")