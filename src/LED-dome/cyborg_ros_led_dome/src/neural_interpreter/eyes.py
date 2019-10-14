#!/usr/bin/env python
import rospy
import system.settings as settings



class Eyes():

    def __init__(self):
        self.isStatic = True
    def render(self,input_data, output_data):

        output_data[248*3+2] = 10
        output_data[249*3+2] = 55
        output_data[250*3+2] = 55
        output_data[251*3+2] = 10

        output_data[273*3+2] = 10
        output_data[274*3+2] = 55
        output_data[277*3+2] = 55
        output_data[278*3+2] = 10
        
        output_data[317*3+2] = 10
        output_data[318*3+2] = 55
        output_data[319*3+2] = 55
        output_data[320*3+2] = 10



        output_data[238*3+2] = 10
        output_data[239*3+2] = 55
        output_data[240*3+2] = 55
        output_data[241*3+2] = 10

        output_data[283*3+2] = 10
        output_data[284*3+2] = 55
        output_data[287*3+2] = 55
        output_data[288*3+2] = 10

        output_data[307*3+2] = 10
        output_data[308*3+2] = 55
        output_data[309*3+2] = 55
        output_data[310*3+2] = 10
        

        ###########################
        # backside
        output_data[0*3+2] = 30
        output_data[1*3+2] = 30
        output_data[2*3+2] = 30
        output_data[3*3+2] = 30
        output_data[4*3+2] = 30
        output_data[5*3+2] = 30
        output_data[6*3+2] = 30
        output_data[7*3+2] = 30
        # left
        output_data[23*3+2] = 30
        output_data[24*3+2] = 30
        output_data[69*3+2] = 30
        output_data[70*3+2] = 30
        output_data[127*3+2] = 30
        output_data[128*3+2] = 30

        output_data[192*3+2] = 30
        output_data[193*3+2] = 30
        output_data[262*3+2] = 30
        output_data[263*3+2] = 30
        output_data[331*3+2] = 30
        output_data[332*3+2] = 30
        output_data[399*3+2] = 30
        output_data[400*3+2] = 30
        output_data[467*3+2] = 30

        output_data[468*3+2] = 30
        output_data[531*3+2] = 30
        output_data[532*3+2] = 30
        output_data[592*3+2] = 30
        output_data[593*3+2] = 30
        output_data[649*3+2] = 30
        output_data[650*3+2] = 30
        output_data[699*3+2] = 30
        output_data[700*3+2] = 30
        output_data[741*3+2] = 30
        output_data[742*3+2] = 30
        output_data[773*3+2] = 30
        output_data[774*3+2] = 30

        # front
        output_data[783*3+2] = 30
        output_data[784*3+2] = 30
        output_data[785*3+2] = 30
        output_data[786*3+2] = 30
        output_data[787*3+2] = 30
        output_data[788*3+2] = 30
        output_data[789*3+2] = 30
        output_data[790*3+2] = 30

        #right
        output_data[8*3+2] = 30
        output_data[44*3+2] = 30
        output_data[45*3+2] = 30
        output_data[97*3+2] = 30
        output_data[98*3+2] = 30
        output_data[159*3+2] = 30
        output_data[160*3+2] = 30
        output_data[227*3+2] = 30
        output_data[228*3+2] = 30
        output_data[297*3+2] = 30
        output_data[298*3+2] = 30
        output_data[365*3+2] = 30
        output_data[366*3+2] = 30
        output_data[433*3+2] = 30
        output_data[434*3+2] = 30
        output_data[499*3+2] = 30
        output_data[500*3+2] = 30
        output_data[562*3+2] = 30
        output_data[563*3+2] = 30
        output_data[621*3+2] = 30
        output_data[622*3+2] = 30
        output_data[675*3+2] = 30
        output_data[676*3+2] = 30
        output_data[721*3+2] = 30
        output_data[722*3+2] = 30
        output_data[759*3+2] = 30
        output_data[760*3+2] = 30
        output_data[783*3+2] = 30


        











