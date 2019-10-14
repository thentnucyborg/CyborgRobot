import system.settings as settings
import sys
import neural_interpreter.support_functions.data_to_color as d2c
import numpy as np

class Intensity:
    def __init__(self):
        if settings.NEURAL_DATA_TYPE != 'intensity' and settings.NEURAL_SOURCE != 'file':
            sys.exit('This interpreter is not available in this mode. Use --file <location> and --datatype intensity')

    def render(self, input_data, output_data):
        input_data = np.array(input_data)
        max_value = input_data.min()
        for j in range(len(input_data)):
            input_data[j] -= max_value
        volt_per_group = input_data.max() / 10
        input_groups = [0] * settings.NEURAL_ELECTRODES_TOTAL
        for j in range(len(input_data)):
            input_groups[j] = d2c.color_grouping(j, input_data, volt_per_group)
        d2c.data_to_bytearray(input_groups, output_data)

if __name__ == '__main__':
    pass
