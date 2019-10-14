#!/usr/bin/env python
"""Created by Areg Babayan on 6/10/2018."""
__author__      = "Areg Babayan"
__license__     = "BSD"
__version__     = "0.0.6"

import threading

import rospy
import system.settings as settings
import smach
import smach_ros
from std_msgs.msg import String
import time
from neural_presenters.serial.serial_communication import SerialInterface
from neural_interpreter.support_functions.data_to_color import create_electrode_mapping
from neural_sources.file.file_server import FileServer
from neural_interpreter.eyes import Eyes
from neural_interpreter.siren import Siren
from neural_interpreter.moving_average import MovingAverage
from neural_interpreter.individual_moving_average import IndividualMovingAverage
from neural_interpreter.snake import Snake

    
class startup(smach.State):
    def __init__(self, loopfunction,update_visualization_mode):
        smach.State.__init__(self,outcomes=["nonmea","meafromfile","text"],
                               output_keys=["presenter_out","interpreter_out",
                                            "led_colors_out","current_interpreter_out"])
        self.loop = loopfunction
        self.update_visualization_mode = update_visualization_mode

    def execute(self,userdata):
        #initialize pipeline
        neuron_data = [0] * settings.NEURAL_ELECTRODES_TOTAL
        userdata.led_colors_out = bytearray([0] * (3 * settings.LEDS_TOTAL))
        userdata.interpreter_out = Eyes()
        userdata.current_interpreter_out = "eyes" 

        #execute animation
        self.loop(neuron_data)
        print("LED-dome initiated - waiting for incoming message")
        #wait for incoming message
        while not settings.CHANGE_REQUESTED and not rospy.is_shutdown(): 
            pass
            
        return self.update_visualization_mode()


#currently not implemented, included as a placeholder
class meafromserver(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=["nonmea"])

    def execute(self,userdata):
       """  rospy.loginfo("")
        global _source
        _source.loop() """
        #return "transition"

        
class meafromfile(smach.State):
    def __init__(self, loopfunction, return_interpreter,update_visualization_mode):
        smach.State.__init__(self,outcomes=["nonmea","meafromfile","text"],
                                input_keys=["current_interpreter_in","presenter","interpreter"],
                               output_keys=["interpreter","presenter"])
        self.return_interpreter = return_interpreter
        self.loop = loopfunction
        self.update_visualization_mode = update_visualization_mode
    def execute(self,userdata): 
        rospy.loginfo("executing meafromfile, interpreter: %s"%userdata.current_interpreter_in)
        #set interpreter and source
        userdata.interpreter = self.return_interpreter(userdata.current_interpreter_in)
        source= FileServer(self.loop,SerialInterface)

        #delay added for correct output after switching interpreter
        rate = rospy.Rate(10)
        rate.sleep()  
        #execute visualization and wait for message
        source.loop()
        return self.update_visualization_mode()



class nonmea(smach.State):
    def __init__(self, loopfunction, return_interpreter,update_visualization_mode):
        smach.State.__init__(self,outcomes=["nonmea","meafromfile","text"],
                                input_keys=["current_interpreter_in","interpreter_in","presenter"],
                               output_keys=["interpreter_out","presenter"])
        self.neuron_data = [0] * settings.NEURAL_ELECTRODES_TOTAL
        self.return_interpreter = return_interpreter
        self.loop = loopfunction
        self.update_visualization_mode = update_visualization_mode

    def execute(self,userdata):
        rospy.loginfo("executing nonmea, interpreter: %s" %userdata.current_interpreter_in)
        #set interpreter
        userdata.interpreter_out=self.return_interpreter(userdata.current_interpreter_in)
        #delay added for correct output after switching interpreter
        rate = rospy.Rate(10)
        rate.sleep()    

        #execute visualization and wait for message
        if userdata.interpreter_in.isStatic:
            self.loop(self.neuron_data) 
            while not settings.CHANGE_REQUESTED and not rospy.is_shutdown():
                pass
        else:
            while not settings.CHANGE_REQUESTED and not rospy.is_shutdown():
                self.loop(self.neuron_data)    
        return self.update_visualization_mode()
    
class text(smach.State):
    def __init__(self, update_visualization_mode):
        smach.State.__init__(self,outcomes=["nonmea","meafromfile"],
                                input_keys=["presenter","text"],
                                output_keys=["presenter","text"])
        self.update_visualization_mode = update_visualization_mode
    def execute(self,userdata):
        rospy.loginfo("executing text state")
        while not settings.CHANGE_REQUESTED and not rospy.is_shutdown():
            if userdata.text != None:
                if ("vertical" in userdata.text):
                    userdata.text = userdata.text[9:]
                    data = bytearray([253]) + bytearray([252]) + bytearray(userdata.text,"ascii") #utf-8
                else:
                    data = bytearray([253]) + bytearray([0]) + bytearray(userdata.text,"ascii") #utf-8   

                userdata.presenter.refresh(data)
                userdata.text = None
        return self.update_visualization_mode()


def domecontrol():
    #create a SMACH state machine
    sm = smach.StateMachine(outcomes=["error"])
    #create SMACH state machine userdata variables
    sm.userdata.sm_presenter=SerialInterface()
    sm.userdata.sm_interpreter = None 
    sm.userdata.sm_led_colors = None
    sm.userdata.sm_mode = None
    sm.userdata.sm_current_interpreter = None
    sm.userdata.sm_next_interpreter = None
    sm.userdata.sm_text = None

    def loop(data):
        sm.userdata.sm_interpreter.render(data,sm.userdata.sm_led_colors)
        for index in range(len(sm.userdata.sm_led_colors)):
            sm.userdata.sm_led_colors[index] = min(sm.userdata.sm_led_colors[index],251)
        sm.userdata.sm_presenter.refresh(sm.userdata.sm_led_colors)

    def return_interpreter(interpreter):
        if "moving-average" in interpreter:
            return MovingAverage()
        elif "individual-moving-average" in interpreter:
            return IndividualMovingAverage()
        elif "siren" in interpreter:
            return Siren()
        elif "eyes" in interpreter:
            return Eyes()
        elif "snake" in interpreter:
            return Snake()
        else:
            pass

    def update_visualization_mode():
        #flush led_colors array
        sm.userdata.sm_led_colors = bytearray([0] * (3 * settings.LEDS_TOTAL))

        #return next state
        if sm.userdata.sm_next_interpreter in ("siren","eyes","snake"):
            sm.userdata.sm_mode = "nonmea"
                      
        elif sm.userdata.sm_next_interpreter in ("moving-average","individual-moving-average"):
            sm.userdata.sm_mode = "meafromfile"

        elif sm.userdata.sm_next_interpreter == "text":
            sm.userdata.sm_mode = "text"

        sm.userdata.sm_current_interpreter = sm.userdata.sm_next_interpreter
        sm.userdata.sm_next_interpreter = None
        settings.CHANGE_REQUESTED = False
        return sm.userdata.sm_mode
        
    def set_visualization_mode_callback(data):
        if (data.data in ("siren","eyes","snake","moving-average","individual-moving-average")
            and sm.userdata.sm_current_interpreter != data.data):
            sm.userdata.sm_next_interpreter = data.data
            settings.CHANGE_REQUESTED = True
        elif ("text" in data.data):
            if (len(data.data)<=5):
                print("Received empty text")
            else:
                sm.userdata.sm_text = data.data[5:]
                if sm.userdata.sm_current_interpreter != "text":
                    sm.userdata.sm_next_interpreter = "text"
                    settings.CHANGE_REQUESTED = True

    #initialize subscriber
    set_visualization_mode_subscriber = rospy.Subscriber("cyborg_visual/domecontrol", String, set_visualization_mode_callback)


    #Open the container
    with sm:
        #add states and remap userdata for sharing between states
        smach.StateMachine.add("Startup",startup(loop,update_visualization_mode),
               transitions={"nonmea":"Nonmea",
                            "meafromfile":"MEAFromFile",
                            "text":"Text"},
                 remapping={"presenter_out":"sm_presenter",
                            "interpreter_out":"sm_interpreter",
                            "led_colors_out":"sm_led_colors",
                            "current_interpreter_out":"sm_current_interpreter",
                            "text":"sm_text"})
        
        smach.StateMachine.add("Nonmea",nonmea(loop,return_interpreter,update_visualization_mode),
                transitions={"nonmea":"Nonmea",
                             "meafromfile":"MEAFromFile",
                             "text":"Text"},
                  remapping={"current_interpreter_in":"sm_current_interpreter",
                             "interpreter_in":"sm_interpreter",
                             "interpreter_out":"sm_interpreter",
                             "presenter":"sm_presenter"})

        smach.StateMachine.add("MEAFromFile",meafromfile(loop,return_interpreter,update_visualization_mode),
                transitions={"nonmea":"Nonmea",
                             "meafromfile":"MEAFromFile",
                             "text":"Text"},
                  remapping={"current_interpreter_in":"sm_current_interpreter",
                             "presenter":"sm_presenter",
                             "interpreter":"sm_interpreter"})

        smach.StateMachine.add("MEAFromServer",meafromserver(),
                transitions={"nonmea":"Nonmea"})

        smach.StateMachine.add("Text",text(update_visualization_mode),
                transitions={"nonmea":"Nonmea",
                             "meafromfile":"MEAFromFile"},
                  remapping={"text":"sm_text",
                            "presenter":"sm_presenter"})
                
    #execute state machine
    #outcome = sm.execute()
    smach_thread = threading.Thread(target=sm.execute)
    smach_thread.daemon = True
    smach_thread.start()

if __name__=="__domecontrol__":
    domecontrol()






