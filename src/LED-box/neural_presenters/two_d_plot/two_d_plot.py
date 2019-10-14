import time
import sys
import system.settings as settings
import matplotlib.patches as patches
import matplotlib.pyplot as plt


class TwoDPlot:

    def running(self):
        return plt.fignum_exists(self.fig.number)

    def shutdown(self):
        plt.close()

    def __init__(self):
        if settings.LED_MODEL_NAME != 'none':
            raise SyntaxError("Model not supported by current program")
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111)

    def refresh(self, led_color_array):
        self.ax.clear()
        hex_array = self.byte_to_hex(led_color_array)
        x_pos = 0.6
        y_pos = 0.6
        x_size = 0.8
        y_size = 0.8
        k = 0
        for i in range(len(hex_array)):
            p = patches.Rectangle((x_pos, y_pos), x_size, y_size, facecolor=hex_array[i], edgecolor="black",
                                  linewidth=2)
            self.ax.add_patch(p)
            if k == 9:
                y_pos += y_size + 0.2
                x_pos = 0.6
                k = 0
            else:
                x_pos += x_size + 0.2
                k += 1
        plt.ylim([0, 7])
        plt.xlim([0, 11])
        plt.draw()
        plt.pause(0.00000001)

    def byte_to_hex(self, byte_array):
        hex_array = byte_array.hex()
        output = [0] * 60
        for i in range(0, int(len(hex_array)), 6):
            hex_string = '#'
            for j in range(6):
                hex_string += hex_array[i + j]
            output[int(i / 6)] = hex_string
        return output


if __name__ == '__main__':
    test_array = [bytearray([255, 0, 0] * 60), bytearray([0, 255, 0] * 60), bytearray([0, 0, 255] * 60)]
    plot_class = TwoDPlot()
    # start loop
    for byte_array in test_array:
        plot_class.refresh(byte_array)
        time.sleep(1)
