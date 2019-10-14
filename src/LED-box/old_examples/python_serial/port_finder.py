import serial.tools.list_ports

def arduino_port():
	ports = list(serial.tools.list_ports.comports())
	for p in ports:
	    if "arduino" in str(p).lower():
	    	return str(p).split(" -")[0]




