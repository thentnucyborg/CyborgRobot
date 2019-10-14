from colour import Color
import system.settings as settings
from math import sqrt
from random import randint
from random import shuffle

_led_mapping = None

def get_unpacked_model(model):
        name = model['name']
        if name == "large_cube":
            sides = ['top', 'north', 'west', 'south', 'east']
            mapping = [None] * 5
            for i in range(5):
                mapping[i] = [-1] * 50
                for j in range(5):
                    side = sides[j]
                    for k in range(10):
                        mapping[i][10*j + k] = model['led-groups'][side][i][k]
            return mapping
        else:
            return None

def create_electrode_mapping(presenter):
    global _led_mapping
    model = settings.LED_MODEL
    n_elct = settings.NEURAL_ELECTRODES_TOTAL
    n_leds = settings.LEDS_TOTAL
    
    mapping = get_unpacked_model(settings.LED_MODEL)
    if mapping is None:
        print("No dot mapping for model, neural data will be stripy")
        return
    
    leds_per_electrode = n_leds//n_elct
    leds_last_electrode = leds_per_electrode
    if n_leds % n_elct != 0:
        leds_last_electrode += 1
    
    _led_mapping = [None] * n_elct
    led_pos = [None] * n_elct
    last_index = [0,0]
    step = int(sqrt(settings.NEURAL_ELECTRODES_TOTAL*len(mapping)*len(mapping[0])))
    for i in range(n_elct):
        x = last_index[0]
        y = last_index[1]
        last_index[1] += step
        if last_index[1] >= 5:
            last_index[1] = 0
            last_index[0] += step
        
        if x >= 50 or mapping[y][x] == -1:
            x = randint(0,49)
            y = randint(0,4)
            while mapping[y][x] == -1:
                x = randint(0,49)
                y = randint(0,4)

        _led_mapping[i] = [mapping[y][x]]
        led_pos[i] = [[x,y]]
        mapping[y][x] = -1
    
    # Grow each electrode seed

    check_pattern = [[-1,0],[1,0],[0,-1],[0,1]]
    while True:
        temp = check_pattern.pop(0)
        check_pattern.append(temp)
        change = False
        for i in range(n_elct):
            done = False
            led_chosen = -1
            x = -1
            y = -1
            for j in range(len(_led_mapping[i])):
                #shuffle(check_pattern)
                for move in check_pattern:
                    x = led_pos[i][j][0] + move[0]
                    y = led_pos[i][j][1] + move[1]
                    if not (0 <= y <= 4 and 0 <= x <= 49):
                        continue
                    led_chosen = mapping[y][x]
                    if led_chosen != -1:
                        change = True
                        done = True
                        _led_mapping[i].append(led_chosen)
                        led_pos[i].append([x,y])
                        mapping[y][x] = -1
                        break
                if done:
                    break

        if not change:
            break

    """
    electrodes = leds_per_electrode
    if i == n_elct - 1:
        electrodes = leds_last_electrode
    _led_mapping[i] = [-1] * electrodes
    while mapping[last_index[1]][last_index[0]] == -1:
        last_index[1] += 1
        if last_index[1] >= 5:
            last_index[1] = 0
            last_index[0] += 1
            if last_index[0] >= 50:
                print('LEDs out of bounds when creating mapping!')
                exit(-1)
    for j in range(electrodes):
    """

def color_grouping(index, values, value_per_group):
    if values[index] <= value_per_group:
        return 0
    elif value_per_group < values[index] <= 2 * value_per_group:
        return 1
    elif 2 * value_per_group < values[index] <= 3 * value_per_group:
        return 2
    elif 3 * value_per_group < values[index] <= 4 * value_per_group:
        return 3
    elif 4 * value_per_group < values[index] <= 5 * value_per_group:
        return 4
    elif 5 * value_per_group < values[index] <= 6 * value_per_group:
        return 5
    elif 6 * value_per_group < values[index] <= 7 * value_per_group:
        return 6
    elif 7 * value_per_group < values[index] <= 8 * value_per_group:
        return 7
    elif 8 * value_per_group < values[index] <= 9 * value_per_group:
        return 8
    else:
        return 9


def generate_color_gradient():
    color1 = Color(settings.PLOT_COLOR_FROM)
    return list(color1.range_to(Color(settings.PLOT_COLOR_TO), 10))


def data_to_hex(data):
    colors = generate_color_gradient()
    hex_data = [0] * settings.NEURAL_ELECTRODES_TOTAL
    for i in range(len(data)):
        hex_data[i] = colors[int(data[i])].get_hex_l()
    return hex_data


def data_to_bytearray(input_data, output_data):
    global _led_mapping
    hex_array = data_to_hex(input_data)
    num_leds = settings.LEDS_TOTAL
    num_leds_cluster = int(num_leds / settings.NEURAL_ELECTRODES_TOTAL)

    if _led_mapping is not None:
        for i in range(len(hex_array)):
            leds = _led_mapping[i]
            for j in range(len(leds)):
                index = leds[j]*3
                output_data[index:index+3] = bytearray.fromhex(hex_array[i][1:])
    else:
        for i in range(len(hex_array)):
            for j in range(num_leds_cluster):
                index = i * 3 * num_leds_cluster + 3 * j
                output_data[index:index + 3] = bytearray.fromhex(hex_array[i][1:])

'''
def handle_large_cube(hex_array, output_data):
    electrodes_per_side = int(settings.NEURAL_ELECTRODES_TOTAL/5)
    last_side = electrodes_per_side + settings.NEURAL_ELECTRODES_TOTAL - electrodes_per_side*5
    sides = ['top', 'north', 'west', 'south', 'east']
    model = settings.LED_MODEL['led-groups']
    for i in range(5):
        side_leds = model[sides[i]]
        height = len(side_leds)
        width = len(side_leds[0])
        begin = electrodes_per_side*i
        electrodes_for_side = electrodes_per_side
        if i == 5:
            electrodes_for_side = last_side
        
        leds_per_electrode = height*width/electrodes_for_side
        led_cube_edge = sqrt(leds_per_electrode)


        for i in range(electrodes_for_side):
            electrode = hex_array[begin + i][1:]
            curr_hor = i%(width / led_cube_edge)
            curr_vert = i/(width / led_cube_edge)
            for k in range(int(led_cube_edge+0.5)):
                for l in range(int(led_cube_edge+0.5)):
                    led_ver = int(led_cube_edge*curr_vert+k)
                    if led_ver >= width:
                        led_ver = width-1
                    led_hor = int(leds_per_electrode*curr_hor+l)
                    if led_hor >= height:
                        led_hor = height-1
                    index = side_leds[led_hor][led_ver]
                    output_data[index:index+3] = bytearray.fromhex(electrode)
'''