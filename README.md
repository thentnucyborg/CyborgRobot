## Last updated: 12.02.2020 ##
This branch was created for working on updating the system to ROS Melodic and Ubuntu 18.04. This because both ROS Kinetic and Ubuntu 16.04 will reach EOL in 2021. Python 2.7 reached its 01.01.2020. Though all will still work, there will be no further updates or bugfixes from those dates. 
The main problem with working with out of date software is (new) packages from for example ROS may not work with the older distros of ROS or older packages.
It might also be an idea to jump straight to ROS Noetic (to be released in 2020). The main problem with this is that not all libraries and packages are updated at once after a new release, and thus things may not work. 

There are several things that are not installed, especially for things that have to do with the new navigation stack and the gui. 
There may also be missing libraries for other packages. 

Does not finish catkin_make




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
- Also, check out [pioneerlx_setup/setup.sh](https://github.com/thentnucyborg/pioneerlx_setup/blob/master/setup.sh) for more. This setup script was already a little outdated in autumn 2019, and has now changed name to just 'setup.sh'. It has by 12.02.2020 been updates somewhat, but may still not be complete. 

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

openslam_gmapping and gmapping are used for making maps, but are not needed in the normal operation of the cyborg. 