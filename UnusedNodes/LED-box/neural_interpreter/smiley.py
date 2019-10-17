import sys

import system.settings as settings
from colour import Color
from math import pi


class Smiley:
    def __init__(self):
        if settings.LED_MODEL_NAME != 'large_cube':
            sys.exit("Model not supported by current program")
    
    loc = 0
    tot = 40

    def render(self, input_data, led_colors):
        for x in range(10):
            for y in range(5):
                top_id = settings.LED_MODEL['led-groups']['top'][y][x]
                if top_id != -1:
                    led_colors[top_id * 3] = 0
                    led_colors[top_id * 3 + 1] = 0
                    led_colors[top_id * 3 + 2] = 0



        n = self.tot
        x = self.loc
        rad = (x/39)*2*pi
        color = Color(hue=(x/39), saturation=1, luminance=0.5)
        ids = [
            [2, 0],
            [2, 1],
            [2, 2],
            [6, 0],
            [6, 1],
            [6, 2],
            [0, 3],
            [1, 4],
            [2, 4],
            [3, 4],
            [4, 4],
            [5, 4],
            [6, 4],
            [7, 4],
            [8, 3]
        ]
        for led in ids:
            top_id = settings.LED_MODEL['led-groups']['top'][led[1]][led[0]]
            if top_id != -1:
                led_colors[top_id * 3] = int(color.get_red()*255)
                led_colors[top_id * 3 + 1] = int(color.get_green()*255)
                led_colors[top_id * 3 + 2] = int(color.get_blue()*255)
        for t in range(n):
            prev = (x-t) % 40
            falloff = (n-t-1)/(n-1)
            rad = (prev/39)
            color = Color(hue=rad, saturation=1, luminance=0.5)
            for y in range(5):
                x_off = prev
                side = 'north'
                if prev < 10:
                    pass
                elif prev < 20:
                    x_off -= 10
                    side = 'west'
                elif prev < 30:
                    x_off -= 20
                    side = 'south'
                else:
                    x_off -= 30
                    side = 'east'
                led_id = settings.LED_MODEL['led-groups'][side][y][x_off]
                if led_id != -1:
                    led_colors[led_id*3] = int(color.get_red()*255*falloff)
                    led_colors[led_id*3+1] = int(color.get_green()*255*falloff)
                    led_colors[led_id*3+2] = int(color.get_blue()*255*falloff)
        self.loc = (x+1)%n