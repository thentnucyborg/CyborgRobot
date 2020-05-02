#!/usr/bin/env python
import sys
import npyscreen
import rospy
from std_msgs.msg import String
from commandform import CommandForm

"""Created by Thomas Rostrup Andersen on 11/11/2016.
Copyright (C) 2016 Thomas Rostrup Andersen. All rights reserved."""

__author__ = "Thomas Rostrup Andersen"
__copyright__ = "Created by Thomas Rostrup Andersen on 11/01/2017. Copyright (C) 2016 Thomas Rostrup Andersen. All rights reserved."
__license__ = ""
__version__ = "0.0.2"
__all__ = []

def main():
    rospy.init_node("cyborg_command")
    app = CommandApplication()
    app.run()

class CommandApplication(npyscreen.NPSAppManaged):

    STARTING_FORM = "COMMAND_FORM"
    keypress_timeout_default = 1
    
    def onStart(self):
        self.addForm("COMMAND_FORM", CommandForm)

if __name__ == '__main__':
    print("Cyborg Command: Starting Program...")

    if sys.version_info < (2,5):
        print("Cyborg Command: Running Python version " + str(sys.version_info.major) + "." + str(sys.version_info.minor) + "." + str(sys.version_info.micro) + " (Python version 2.5 or grater is required)...")
        exit()

    main()

    print("Cyborg Command: End of Program...")
