import system.settings as settings
from random import randint

class SnakeWhite:
    def __init__(self):
        self.current_led = 0
        self.n = 200

    def render(self, input_data, output_data):
        self.current_led += 1
        for k in range(self.n):
            prev = (self.current_led - k) % settings.LEDS_TOTAL
            falloff = (self.n-k - 1)/self.n
            for j in range(3):
                output_data[prev*3+j] = randint(0, int(255*falloff)) + int(255*(1-falloff))