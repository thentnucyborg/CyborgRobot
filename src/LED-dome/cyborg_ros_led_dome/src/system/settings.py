#!/usr/bin/env python
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
NEURAL_DATA_TYPE = 'frequency'

# Visualization
LED_REFRESHES_PER_SECOND = 6 #18
LED_MODEL_NAME = 'large_cube'
LED_ELECTRODE_SHUFFLE = False

# Spike detection threshold
THRESHOLD = -1*10**7

### Derived variables (initialized in environment.py) ###
LEDS_TOTAL = 791
LED_MODEL = None
NEURAL_DATA_FILE = "/home/aregb/catkin_ws/src/cyborg_ros_led_dome/src/neural_sources/file/data/2017-10-20_MEA2_100000rows_10sec.csv"

# Mode change
CHANGE_REQUESTED = False
