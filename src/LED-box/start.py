import json
import time
import sys
import signal
import system.settings as settings
import system.environment as environment
from neural_presenters.virtual.virtual_led_model import VirtualLedModel
from neural_presenters.serial.serial_communication import SerialInterface
from neural_sources.file.file_server import FileServer
from neural_sources.server.client import Client
from neural_interpreter.random_mode import RandomMode
from neural_interpreter.moving_average import MovingAverage
from neural_interpreter.intensity import Intensity
from neural_presenters.two_d_plot.two_d_plot import TwoDPlot
from neural_interpreter.smiley import Smiley
from neural_interpreter.individual_moving_average import IndividualMovingAverage
from neural_interpreter.snake import Snake
from neural_interpreter.snake_white import SnakeWhite
from neural_interpreter.support_functions.data_to_color import create_electrode_mapping

_presenter = None
_source = None
_interpreter = None
_led_colors = None

def main():
    global _led_colors, _presenter, _source, _interpreter
    _led_colors = bytearray([0] * (3 * settings.LEDS_TOTAL))

    if settings.NEURAL_PRESENTER == "virtual":
        _presenter = VirtualLedModel(settings.LED_MODEL)
    elif settings.NEURAL_PRESENTER == "serial":
        _presenter = SerialInterface()
    elif settings.NEURAL_PRESENTER == "2d-plot":
        _presenter = TwoDPlot()
    else:
        raise SyntaxError("Invalid presenter!")

    if settings.LED_ELECTRODE_SHUFFLE:
        create_electrode_mapping(_presenter)

    def loop(data):
        global _interpreter, _presenter, _led_colors
        _interpreter.render(data, _led_colors)
        _presenter.refresh(_led_colors)

    if settings.NEURAL_SOURCE == "file":
        _source = FileServer(loop, _presenter)
    elif settings.NEURAL_SOURCE == "server":
        _source = Client(loop, _presenter)
    elif settings.NEURAL_SOURCE == "none":
        _source = None
    else:
        raise SyntaxError("Invalid source!")

    if settings.NEURAL_INTERPRETER == "random":
        _interpreter = RandomMode()
    elif settings.NEURAL_INTERPRETER == "moving-average":
        _interpreter = MovingAverage()
    elif settings.NEURAL_INTERPRETER == 'intensity':
        _interpreter = Intensity()
    elif settings.NEURAL_INTERPRETER == "smiley":
        _interpreter = Smiley()
    elif settings.NEURAL_INTERPRETER == "individual-moving-average":
        _interpreter = IndividualMovingAverage()
    elif settings.NEURAL_INTERPRETER == "snake":
        _interpreter = Snake()
    elif settings.NEURAL_INTERPRETER == "snake-white":
        _interpreter = SnakeWhite()
    else:
        raise SyntaxError("Invalid interpreter!")


    if _source is None:
        frame_time = 1.0/settings.LED_REFRESHES_PER_SECOND
        neuron_data = [0] * settings.NEURAL_ELECTRODES_TOTAL
        while _presenter.running():
            past_time = time.time()

            for i in range(len(neuron_data)):
                neuron_data[i] = 0
            loop(neuron_data)

            delta = time.time() - past_time
            sleep_time = frame_time - delta
            if sleep_time < 0:
                print("Can't keep up!")
                sleep_time = 0

            time.sleep(sleep_time)
    else:
        _source.loop()

if __name__ == "__main__":

    # Capture ctrl+C and exit gracefully
    def signal_handler(signal, frame):
        print("Shutting down...")
        if _presenter is not None:
            _presenter.shutdown()
        sys.exit(0)
    signal.signal(signal.SIGINT, signal_handler)

    try:
        environment.setup(sys.argv[1:])
        main()
    except SyntaxError as e:
        print("{}\nUse argument --help for more help.".format(e))
        if _presenter is not None:
            _presenter.shutdown()
        exit(-1)
