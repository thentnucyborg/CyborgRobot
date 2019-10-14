#!/usr/bin/env python
__author__      = "Areg Babayan"
__copyright__   = "Copyright (C) 2019 Areg Babayan"
__license__     = "BSD"
__version__     = "0.0.3"
__all__         = [] 


import rospy

from playback import Playback
from text_to_speech import TextToSpeech

def main():
    rospy.init_node("cyborg_audio")
    playback = Playback()
    text_to_speech = TextToSpeech()
    rospy.spin()



if __name__ =="__main__":
    print("Cyborg Behavior: Starting Program")
    main()
    print("Cyborg Behavior: Shutting Down")