import statistics
import neural_interpreter.support_functions.data_to_color as d2c
from collections import deque
import system.settings as settings

# Takes in array of size 60 with frequencies
class MovingAverage:
    def __init__(self):
        self.history_length = 40
        self.past_variances = deque([0] * self.history_length)
        self.past_averages = deque([0] * self.history_length)
        self.intensities = [0] * settings.NEURAL_ELECTRODES_TOTAL

    def render(self, input_data, output_data):
        current_variation = statistics.stdev(input_data)
        current_average = statistics.mean(input_data)
        self.past_variances.popleft()
        self.past_variances.append(current_variation)
        self.past_averages.popleft()
        self.past_averages.append(current_average)

        variation = statistics.mean(self.past_variances)
        average = statistics.mean(self.past_averages)

        low = average - variation
        high = average + variation
        if low == high:
            high += 1
            
        for index in range(len(input_data)):
            intensity = int((input_data[index]-low)/(high - low)*10)
            intensity = min(9, intensity)
            intensity = max(0, intensity)
            self.intensities[index] = intensity
        d2c.data_to_bytearray(self.intensities, output_data)
