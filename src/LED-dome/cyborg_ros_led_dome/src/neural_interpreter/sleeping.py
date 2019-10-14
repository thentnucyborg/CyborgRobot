#!/usr/bin/env python
import rospy
import system.settings as settings

# Animation for sleeping, zzz or similar

class Sleeping():
    def __init__(self):
        self.isStatic = False