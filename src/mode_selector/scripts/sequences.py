#!/usr/bin/env python

import time
import os
import subprocess
import signal
import sys
from subprocess import Popen

#/home/aregb/Dropbox/Skole/Masteroppgave/Git/mode_selector/startup_scripts_working_version

def start_sequence(sequence):
  # SEQUENCE 1
  # NORMAL START SEQUENCE
  # Start sequence for demonstrations in Glassgarden
    if "1" in sequence:
        print("Starting sequence 1.\n")
        # Start roscore
        # /home/cyborg/start_scripts/
        Popen(["xfce4-terminal", "--command", "/home/cyborg/start_scripts/roscore_start.sh"],preexec_fn=os.setsid) #bash -c "/opt/ros/kinetic/bin/roscore"
        time.sleep(3) # Wait for ROS core to start
        # Start Arnl and controller
        Popen(["xfce4-terminal", "--command", "/home/cyborg/start_scripts/arnl_start.sh"], preexec_fn=os.setsid)
        time.sleep(15)
        Popen(["xfce4-terminal", "--command", "/home/cyborg/start_scripts/controller_start.sh"],  preexec_fn=os.setsid)
        time.sleep(1)
        return 3

    
    
    # SEQUENCE 2
    # ARIA DEMO
    # Aria Demo allows driving the robot using the joystick
    elif "2" in sequence:
        print("Starting Aria demo.\n")
        Popen(["xfce4-terminal", "--command", "/usr/local/Aria/examples/./demo"],  preexec_fn=os.setsid)
        time.sleep(4)
        return -1
    
    
    
    # SEQUENCE 3
    #
    elif "3" in sequence:
        print("Starting sequence ARNL.\n")
        Popen(["xfce4-terminal", "--command", "/home/cyborg/start_scripts/arnl_start.sh"], preexec_fn=os.setsid)
        time.sleep(4)
        return 1


    # SEQUENCE 4
    elif "4" in sequence:
        print("Starting sequence 4.\n")



    # SEQUENCE 5
    elif "5" in sequence:
        print("Starting sequence 5.\n")



    # SEQUENCE 6
    elif "6" in sequence:
        print("Starting sequence 6.\n")




    # SEQUENCE 7
    elif "7" in sequence:
        print("Starting sequence 7.\n")




    # SEQUENCE 8
    elif "8" in sequence:
        print("Executing sequence 8.\n")



#/home/cyborg/start_scripts/shutdown_aria.sh
def stop_sequence(process_count):
    
    if process_count ==-1:
        Popen(["xfce4-terminal", "--command", "/home/cyborg/start_scripts/shutdown_aria.sh"])
        print("Finished shutting down.\n")


    elif process_count >0:
        num_to_kill = process_count
        #pids_to_kill = pids
        while num_to_kill>0:
            bash_pids = subprocess.check_output(["pidof", "bash"])
            pid_to_kill = bash_pids.split(" ")[0]
            os.killpg(int(pid_to_kill), signal.SIGTERM)
            num_to_kill = num_to_kill -1
            time.sleep(1)

        print("Finished shutting down.\n")

