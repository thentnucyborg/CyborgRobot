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
    rospy.init_node("cyborg_modeselector")
    # livefeed = Livefeed()
    # maneuver
    self.publisher_behaviourstate = rospy.Publisher(rospy.get_name() + "/behaviour_state", String, queue_size=10)

    topicTransmitter = TopicTransmitter()
    rospy.spin()


if __name__ =="__main__":
    print("Cyborg modeselector: Starting Program")
    main()
    print("Cyborg modeselector: Shutting Down")