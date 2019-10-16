#!/usr/bin/env python

import serial
import time
from sequences import start_sequence, stop_sequence


sequence = -1
process_count = 0
running = False


ser = serial.Serial('/dev/ttyUSB12')
time.sleep(3)
ser.write("C")

while True:
    while running == False:
        line = ser.readline()
        print("\nline: " + line)
        if int(line.split(" ")[2]) in range(9)[1:9]: # Check if sequence is in range 1-8
            process_count = start_sequence(line.split(" ")[2])
            if process_count != 0:
                ser.write("F")
                running = True
                #time.sleep(1)
                sequence = int(line.split(" ")[2])
                print("\n Done")
        time.sleep(1)
    
    while running == True:
        line = ser.readline()
        print("\nline: " + line)
        if int(line.split(" ")[2]) ==3: # Check if sequence is 3, shutdown current sequence
            stop_sequence(process_count)
            ser.write("C")
            running = False
        time.sleep(1)


