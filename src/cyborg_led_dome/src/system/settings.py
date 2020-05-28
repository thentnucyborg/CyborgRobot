#!/usr/bin/env python

import os

# Serial
SERIAL_BAUD_RATE = 1497600
# 2D plot
PLOT_COLOR_FROM = 'blue'
PLOT_COLOR_TO = 'red'

# Server
SERVER_IP = '10.22.67.23'
SERVER_PORT = '6780'
SERVER_TIMEOUT = 5

# Data flow
NEURAL_ELECTRODES_TOTAL = 60
NEURAL_PRESENTER = 'serial'
NEURAL_SOURCE = 'none'
NEURAL_INTERPRETER = 'random'
NEURAL_DATA_TYPE = 'frequency'	#possible values: intensity, frequency

# Visualization
LED_REFRESHES_PER_SECOND = 6 #18
LED_MODEL_NAME = 'large_cube'
LED_ELECTRODE_SHUFFLE = False

#1 means lowest number on the left, -1 means lowest number on the right (read from left vs right). The leds are 0-indexed
LED_ARRAY_ROWS = [	[1,	 0,	  7],
					[1,	 8,	  23],
					[-1, 24,  44],
					[1,	 45,  69],
					[-1, 70,  97],
					[1,	 98,  127],
					[-1, 128, 159],
					[1,	 160, 192],
					[-1, 193, 227],
					[1,	 228, 262],
					[-1, 263, 297],
					[1,	 298, 331],
					[-1, 332, 365],
					[1,	 366, 399],
					[-1, 400, 433],
					[1,	 434, 467],
					[-1, 468, 499],
					[1,	 500, 531],
					[-1, 532, 562],
					[1,	 563, 592],
					[-1, 593, 621],
					[1,	 622, 649],
					[-1, 650, 675],
					[1,	 676, 699],
					[-1, 700, 721],
					[1,	 722, 741],
					[-1, 742, 759],
					[1,	 760, 773],
					[-1, 774, 783],
					[1,	 784, 790]]		
LED_ROWS = 30

# Spike detection threshold
THRESHOLD = -1*10**7

### Derived variables (initialized in environment.py) ###
LEDS_TOTAL = 791
LED_MODEL = None

homedir = os.path.expanduser("~")
path = homedir + "/catkin_ws/src/cyborg_led_dome/src/neural_sources/file/data/2017-10-20_MEA2_100000rows_10sec.csv"
NEURAL_DATA_FILE = path

# Mode change
CHANGE_REQUESTED = False
