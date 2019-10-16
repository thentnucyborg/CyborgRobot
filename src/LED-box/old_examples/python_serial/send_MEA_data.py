import serial
import serial.tools.list_ports
import time 
import threading
import struct

from port_finder import *
from MEA_mapping import mappedMEAMatrix


MEAFileName = "miniMea.csv"
LEDMatrix = mappedMEAMatrix(MEAFileName)


BAUD_RATE = 250000
ARDUINO_RESTART_TIME = 3
ARDUINO_PORT = arduino_port()

ser = serial.Serial(ARDUINO_PORT, BAUD_RATE)	#This will as a side effect cause arduino to restart
time.sleep(ARDUINO_RESTART_TIME)				#Wait for arduino to restard and setup


connected = True


def readSerial():
	print("Ready to read from serial")

	recvString = ""
	while(connected):
		recvData = ser.readline()
		print("\nFrom Arduino: ", recvData.strip())

	print("Stopped reading")

####################################################################

# # Arduino serial buffer size is set to 64 byte as default (can be expanded to 256 byte)
# def sendLine(line):
# 	lineLengthCap = 63
# 	byteString = str(line).encode()
# 	#we split the bytestring into substrings of length 64, and send one at a time.
# 	subStrings = [byteString[i:i+lineLengthCap] for i in range(0, len(byteString), lineLengthCap)]
# 	for string in subStrings:
# 		ser.write(string)


# def sendMEAMatrix():
# 	for array in MEAData:
# 		sendLine(array)


# 	ser.write("|".encode())
# 	print("Done transfering")
######################################################################

def sendTimeSlice(LEDMatrix, t):
	# stringLengthCap = 63

	dataString = ""
	s = struct.pack('!{0}B'.format(len(LEDMatrix[t])), *LEDMatrix[t])
	ser.write(s)
	ser.write("\n".encode())

def sendMatrix(LEDMatrix):
	for i in range(0, len(LEDMatrix) - 1):
		sendTimeSlice(LEDMatrix, i)

readingThread = threading.Thread(target=readSerial)
readingThread.setDaemon(True)
readingThread.start()



# startTime = time.time()
sendTimeSlice(LEDMatrix, 0)
# stopTime = time.time()
	
time.sleep(2)

connected = False	#release the reading thread


# nrOfBytesSent = len(str(MEADATA))
# elapsedTime = stopTime - startTime
# byteRate = nrOfBytesSent/elapsedTime
# print("Amount of data sent = {} kB".format(int(nrOfBytesSent/1000)))
# print("Elapsed time = ", elapsedTime)
# print("byteRate = {} kB/s".format(int(byteRate/1000)))
# # print("byteRate = {} kB/s".format(newByteRate/1000))