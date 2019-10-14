![alt text](https://i.imgur.com/UzyoMpD.png)
# NTNU's Experts in Teams 2018: Cyborg

## About
The project's goal is  to humanize NTNU's cyborg. This involves the following subgoals:
- Designing an outer shell
- Processing neural data
- Visualizing neural data

# Installation
You can install the project in two ways, either by using Anaconda, or by using pip. The latter is probably the simplest.

## Anaconda
If you are using Anaconda, you can quickly get the project up and running by cloning the project, opening the Anaconda prompt and executing:
```
cd 317
conda create -n eit --file requirements
activate eit
```
This will create a virtual environment for this project. This makes it easy to remove the installed packages afterwards, considering their collective size.

Since the package websocket-client, nessesary for the server functionality, is not available in the main Anaconda package repository, you will have to install this through pip. Open the Anaconda prompt and type the following:
```
activate eit
pip install websocket-client
```
You can also use the conda-forge repository:
```
conda install -c conda-forge websocket-client
```

## Pip
Clone the project and open up a terminal (with python 3) and type:
```
cd 317
pip3 install -r requirements.txt
```

## Pycharm and Anaconda
If you are using PyCharm and Anaconda, you will have to do the following:
- File -> Settings -> Project: 317 -> Project Interpeter
- Press the cog and Add local
- Navigate to the Anaconda installation folder, then env/eit/python.exe
- Press the arrow at the top right corner and Edit configurations
- Add a new python configuration
- Choose the script you are working on (add several configurations for the different files if needed) and the default interpeter

## Visual Studio Code and Anaconda
If you are using VSCode with Anaconda, you will have to open it and do the following after opening the 317 folder:
- SHIFT + CTRL + P, type "python interpreter" and choose the first option
- Choose the eit environment (may have to wait some time, if not available, add the path ..anaconda root../env/eit/python.exe to the settings.json file)
- SHIFT + CTRL + P again, type "default build task" and choose the first option, add the following task:
```
"label": "start",
"type": "shell",
"command": "${config:python.pythonPath} start.py",
"group": {
    "kind": "build",
    "isDefault": true
}
```
- Now press SHIFT + CTRL + B to run

## Getting the virtual model to work
On Windows, the freeglut.dll needed for OpenGL is included and everything should work out-of-the-box.

If you are using linux and want to use the virtual model, you will have to execute the following command after installing the project as described above:
```
sudo apt-get install freeglut3-dev
```
The virtual model is, for the time being, not supported on Mac OS.

# Running the project
Open up a terminal with python 3, navigate to the project folder and type `python start.py --help` to get a list over the available arguments. Just running the start.py file will start the project with its default arguments, equivalent of executing the command:
```
python start.py --refresh-rate 10 --datatype frequency --led-model large_cube --interpreter individual-moving-average --colors blue red --file neural_sources/file/data/2017-10-20_MEA2_100000rows_10sec.csv --serial
```
An example for running the program with the remap-server using the virtual cube is shown the following command:
```
python start.py --server <ip_address_server> --port <port> --virtual --refresh-rate 30
```

# Project structure
The project is divided into three modules: neural sources, neural interpreters and neural presenters. Each of these modules represent a part of a three stage pipeline for visualizing neural data, and each module consists of several sub-modules that are interchangeable. The different sub-modules can be chosen by supplying the correct argument on runtime. The pipeline itself is located in the start.py file, with supplementary files located in the systems module.

## Start/system
This file marks the start of the project and holds the overall architecture.

Firstly, the input arguments are parsed by the environment.py file. This file is responsable for setting up the running environment based on the supplied input arguments. The environment is represented as a singelton holding the required arguments and pointers, located in setting.py.

An object for each stange in the pipe is then instanciated, in turn, based on the current environment. Eventually the project then enters an infinte loop contained in the neural source object. Start.py provides the neural source object with a callback function that passes the neural data from the source object, through the interpreter and at last to the presenter. The refresh rate is handled by the source object.

## Neural data sources
Marks the first step in the pipeline. This module is responsable for getting the neural data and creating a loop that passes the data on in regular intervals. The data is passed on through a callback to start.py.

A neural source has to implement a contructor, that takes a callback function and a presenter object (in case the source has to display something, eg. a connection error), and a loop function, that starts an infinite loop. This loop has to call the callback function regularly (according to the `--refresh-rate` argument).

There are currently three sources for neural data, which can be set by arguments: `--file`, `--server <ip>` and `--no-input`. The default is `--file`.

### File
This data source will read neural data from a `.csv` file depending on which data type selected (default: `frequency`):
* `--datatype frequency` outputs an array where each elements is the number of spikes detected after last refresh. The threshold value can be changed in the settings file (default: `-1e7`) **OBS: Negavtie value!**
* `--datatype intensity` outputs an array with the voltage recorded at each node.
> Tips:
> Increasing `--refresh-rate` will make file source mode more exiting, but may be limited by the presenter (in our experience, a refresh rate of 15 is the highest rate for the large LED cube)

### Server
This source will read neural data from a websocket server (https://github.com/cyborg-client/Remap-server) at the provided ip. The default port of 6780 can be overvritten by supplying the argument `--port <port>`. The LEDs on the presenter will turn red if the connection times out. Although supported by the remap-server, passing feedback signals back to the neurons is not yet supported by this program.
> Note:
> The remap-server doesn't support the intensity datatype and will fail if this datatype is supplied.

### No input
This source only contains a loop and supplies the callback with a zero-array of data. This is useful for intrepreters that don't use the neural data such as random (demo-programs)

## Data interpreters
Second step in the pipeline. This module in responsable for converting the Micro Electrode Array output to a array of RGB values appropriate for the model. We have provided a handful of different interpreters, but it is easy to create new ones. The demo-programs does not a neural source.

Interpreter classes has to implement a constructur function (and throw syntax error if the environment is invalid) and a render function. The render function takes to arguments, an input array of the neural data and an output array that is to be filled with the RBG LED values. The function returns nothing and is called before every refresh.

There are currently 7 interpreters. A interpreter can be selected by adding the argument `--interpreter <name>`. The default interpreter is `individual-moving-average`.

### Individual moving average
```
induvidual-moving-average
```
Stores all the neural data for the past n cycles. Then calculates the standard deviation and average for each neural node and sets the low cap to `average - 2*deviation` and high to `average + 2*deviation`. This causes the program to display any neural activity for each node relative to itself without the need for manual adjustment. This program was made since the neural data nodes tends to have an unevenly distributed amount of activity, so a global high and low will result in some nodes constantly being off or on. High and low color can be changed using `--colors` (default: `blue` and `red`).

### Moving average
```
moving-average
```
Calculates the standard deviation and the average for each cycle and stores them. It then takes the average of the last n values and sets the low cap to `average - deviation` and high to `average + deviation`. This causes the LED colors to represent the dynamic area of the neural data. No manual adjustment required. High and low color can be changed using `--colors` (default: `blue` and `red`).

### Intensity
```
intensity
```
Intensity interpreter uses the voltage to place each node into 10 groups based on each node output intensity on the MEA plate. The highest voltage will always be in group 9 and lowest voltage will always be in group 0. High and low color can be changed using `--colors` (default: `blue` and `red`).

### Random
```
random
```
Demo prorgam. Randomizes every LED color.

### Smiley
```
smiley
```
Demo prorgam. Only works on the `large_cube` led model. Shows a smiley on the top side, while a wave is travelling around the sides while cycling through different colors.

### Snake
```
snake
```
Demo program. Shows a snake travelling through the LED strip, changing color on each cycle.

### Snake white
```
snake-white
```
Demo program. Shows a random-colored snake moving through the LED strip on a white background.

## Visual presenters
Last step of the pipeline. Responsable for showing the interpreted neural data to the user.

Presenters should run in a separate thread. They have to implement a constructor, that sets up everything needed before displaying anything, a shutdown function, that gracefully shuts the presenter down, a running function, that reports back whether the presenter is still running, and a refresh function, for showing the LED values. The refresh funciton takes a array of RGB colors. The length of this array equals three times the number of LEDs in the active model.

There are currently three presenters. `--serial`, `--2d-plot` and `--virtual`. The default is `--serial`.

### Serial
![alt_text](https://i.imgur.com/SUMn21W.jpg)
Responsable for convaying the LED data to a physical model connected over USB. Will exit if it can't find the Arduino controlling the lights. The folder arduino_files containes the code on the arduino running the lights on the large_cube model.

### 2D plot
![alt_text](https://i.imgur.com/fXxC1Tp.png)
Simple 2D plot made using the MatPlotLib library. The output is mapped 1:1 with the source data.

### Virtual
![alt_text](https://i.imgur.com/dXQbp2c.jpg)
A 3D virtual representation of a physical LED model made in pyopengl. The model runs in a separate thread and is mainly implemented in OpenGL 3.2, however, the debug mode is implemented in OpenGL 1 for compatibility reasons. The presenter uses the json files in the led_models folder to generate the models.

Controls:
- Use the D key to switch to debug mode
- Press and hold left mouse button + move mouse to rotate the model
- Use the scroll wheel to zoom in and out

# LED model files
Holds JSON files containing information about a given physical model. The virtual presenter uses this to generate a 3D presentation of the model. Below is a commented example:
```javascript
{
  // Defines the sequential positions of the LEDs in the led strip in 3D space.
  // 1 unit = 1 meter.
  "led-strip": [
    [0, 0, -1.1], // The first LED position in [X,Y,Z].
    [1, 0, -1.1],
    [0, 1, -1.1]
  ],

  // Defines a polygonal model behind which the LEDs will sit.
  // Follow the right-hand-rule to ensure the normals of the polygons
  // points out towards the observer.
  "led-enclosure": [
    [[-0.5, -0.5, -1], [2, -0.5, -1], [-0.5, 2, -1]] // First triangular polygon, normal along negative Z.
  ],

  // Defines groups that can be used in the code to easier control single LEDs.
  // Accessible through a dictionary object in code, group name is arbitrary.
  // Use n-dimensional arrays with the ids of the LEDs (zero-indexed).
  // Use id -1 for placeholder LEDs.
  "led-groups": {
    "down-plane": [ // Group "down-plane" with a 2D matrix containing LEDs 0, 1, 2 and one placeholder.
      [0, 1],
      [2, -1]
    ]
  }
}
```
