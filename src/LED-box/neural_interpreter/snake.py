import system.settings as settings

class Snake:
    def __init__(self):
        self.current_led = 0
        self.color = 0
        self.num_leds = settings.LEDS_TOTAL

    def render(self, input_data, output_data):
        for i in range(self.current_led, -1, -1):
            print(i, self.color)
            output_data[i*3 + self.color] = max(255 - 1*(self.current_led -i), 0)
        self.current_led += 1
        if self.current_led == self.num_leds:
            self.current_led = 0
            self.color += 1
            if self.color == 3:
                self.color == 0
                for i in range(self.num_leds * 3):
                    output_data[i] = 0
        if self.color == 3:
            self.color = 0