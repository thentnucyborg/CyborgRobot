# CyborgRobot
The NTNU Cyborg Robot repo
For any more information check out the git wiki or _https://www.ntnu.no/wiki/display/cyborg/_.

## Starting with the repo
1. create a catkin workspace 
2. clone this repo into the workspace
3. go into the folder named CyborgRobot (this repo) and show any hidden files
4. move ALL files and folders, including any hidden ones, to the catkin_ws (meaning just outside of CyborgRobot)
You should now be abele to use the workspace and git without any problems.

## Installs needed
- Check out the README-files for each node to learn what dependecies and installs are needed for each.
- Also, check out [pioneerlx_setup/setup.sh](https://github.com/thentnucyborg/pioneerlx_setup/blob/master/setup.sh) for more. 

Run: 
```
find <path-to-catkin_ws/src/> -name '*.py' -exec chmod +x {} \;
```
to make all python files executable to be able to run them. Might need to do it with .sh files too (also catkin_ws/src/rosaria/cfg/RosAria.cfg)

### Installs for navigation
- sudo apt-get install ros-kinetic-navigation
- sudo apt-get install ros-kinetic-tf2-sensor-msgs
- sudo apt-get install libsdl-dev
- sudo apt-get install libsdl-image1.2-dev
- sudo apt-get install libbullet-dev
- git clone https://github.com/ros-visualization/rviz.git -b kinetic-devel
- git clone https://github.com/ros-planning/navigation.git -b kinetic-devel
