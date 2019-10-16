import neural_interpreter.support_functions.data_to_color as d2c
import numpy as np
from collections import deque


class IndividualMovingAverage:
    def __init__(self):
        self.history  = [deque(maxlen=40) for j in range(60)]
        self.intensities = [0]*60
        self.initialized = False

    def render(self, input_data, output_data):
        if not self.initialized:
            for i in range(len(input_data)):
                for j in range(40):
                    self.history[i].append(input_data[i])
            self.initialized = True
        else:
            for i in range(len(input_data)):
                self.history[i].append(input_data[i])
        for i in range(len(input_data)):
            average = np.average(self.history[i])
            std = np.std(self.history[i])
            if std == 0:
                std = 1
            intensity = int((input_data[i] - (average - 2 * std)) / (4 * std) * 10)
            intensity = max(0, intensity)
            intensity = min(9, intensity)
            self.intensities[i] = intensity
        d2c.data_to_bytearray(self.intensities, output_data)
