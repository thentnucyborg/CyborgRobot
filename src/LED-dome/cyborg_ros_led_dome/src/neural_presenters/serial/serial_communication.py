#!/usr/bin/env python
import serial
import sys
import time
import serial.tools.list_ports
from system.settings import SERIAL_BAUD_RATE, LEDS_TOTAL
import system.settings as settings


#function for conversion of array to sparse addressed array, currently not in use
def array2sparseadressedarray(led_color_array):
    indexedarray = bytearray()
    totelements = 0
    for i in range(0,settings.LEDS_TOTAL*3,3):
        if (    led_color_array[i] ==0
            and led_color_array[i +1] == 0
            and led_color_array[i +2] ==0):
            pass
        else:
            #split and append address
            indexedarray.append(((i/3)>>8)& 0xff)
            indexedarray.append((i/3) & 0xff)
            #set color-data
            indexedarray.append(led_color_array[i])
            indexedarray.append(led_color_array[i+1])
            indexedarray.append(led_color_array[i+2])
            totelements+=1
    return indexedarray


class SerialInterface:
    def __init__(self):
        port = "/dev/ttyUSB0" # ACM0 for Arduino
        # Initialize serial communication, 8 data bits, no parity 1 stop bit
        self.ser = serial.Serial(port, SERIAL_BAUD_RATE, serial.EIGHTBITS, serial.PARITY_NONE, serial.STOPBITS_ONE)
        time.sleep(2)

    # Included for completeness, not actually used in our program
    def read(self, num_bytes):
        return self.ser.read(num_bytes)

    def running(self):
        return True
    
    def shutdown(self):
        self.ser.write(bytearray([0] * (3*LEDS_TOTAL)))


    def refresh(self, led_color_array):
        #new array with start/end marker
        array = bytearray([0])
        array[0] = 254
        if (led_color_array[0]!=253):
           array.append(0)
           array.append(0)
        array +=led_color_array
        array.append(255)
        self.ser.write(array)


# This is just a testing function setting all leds to red
if __name__ == '__main__':
    testdata1 = bytearray([255, 0, 0] * 240)
    conn = SerialInterface()
    conn.refresh(testdata1)
    time.sleep(1)

