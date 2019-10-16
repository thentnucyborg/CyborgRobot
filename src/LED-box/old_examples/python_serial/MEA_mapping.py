
def MEAFileToMatrix(fileName):


	MEAFile = open(fileName, "r")


	#Assuming the .csv file contains metadata, we don't start at line 0
	FIRST_LINE_OF_DATA = 7
	MEAData = MEAFile.readlines()[FIRST_LINE_OF_DATA:]


	MEAMatrix = []
	for line in MEAData:
		line = line.replace("\n", "")
		MEALine = []
		for nr in line.split(","):
			MEALine.append(int(nr))
		MEAMatrix.append(MEALine)
	return MEAMatrix


#just maps values to a 0x00 to 0xff range.
def mapVoltageToLed(voltage, minVoltage, maxVoltage, maxLedValue):
	return int(maxLedValue*(voltage + abs(minVoltage))/(maxVoltage - minVoltage))
	

def mapMEAMatrixToLed(MEAMatrix):
	LEDMatrix = []
	maxLedValue = 0xFF #The simples thing is to send bytes, so we use 0xff as max led value

	#find max and min voltages values
	minValue = 0
	maxValue = 0

	for line in MEAMatrix:
		for nr in line:
			minValue = min(minValue, int(nr))
			maxValue = max(maxValue, int(nr))

	valueRange = maxValue - minValue

	for line in MEAMatrix:
		LEDLine = []
		for nr in line:
			LEDLine.append(mapVoltageToLed(nr, minValue, maxValue, maxLedValue))
			LEDLine.append(mapVoltageToLed(nr, minValue, maxValue, maxLedValue))
			LEDLine.append(mapVoltageToLed(nr, minValue, maxValue, maxLedValue))
		LEDMatrix.append(LEDLine)

	return LEDMatrix


def mappedMEAMatrix(file):
	return mapMEAMatrixToLed(MEAFileToMatrix(file))
# print(mappedMEAMatrix("C:/Users/erldy/Dropbox/NTNU/9.semester/Cyborg/PythonArduinoSerial/MeaData/miniMea.csv"))