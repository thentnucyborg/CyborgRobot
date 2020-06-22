
### This branch contains work with behaviour trees described in Morten Mjelva's thesis from 2018 and Johanne Kalland's thesis from 2020 which has not been included in the master. Because of this the branch has not been deleted as this only exsists here

# CyborgRobot
This repo contains code for running NTNU Cyborgs Robot based on the Pioneer LX mobile robot. The project utilizes ROS.

For more information, check out:
- [this repos git wiki](https://github.com/thentnucyborg/CyborgRobot/wiki)
- [overview wiki at ntnu.no](https://www.ntnu.no/wiki/display/cyborg/)

## Setup and install
The /setup/setup.sh script will run everything, including cloning from this repository and creating a catkin workspace. Meaning you can skip the steps 1-4 and the rest below. If you hav already created a workspace and cloned this repo, comment out those parts of the script if you want to use it. Everything needed should be in the script [last updated May 2020].

### Starting with the repo
1. create a catkin workspace 
2. clone this repo into the workspace
3. go into the folder named CyborgRobot (this repo) and show any hidden files
4. move ALL files and folders, including any hidden ones, to the catkin_ws (meaning just outside of CyborgRobot)
You should now be abele to use the workspace and git without any problems.

### Installs needed
- Check out the README-files for each node to learn what dependecies and installs are needed for each.
- Also, check out [setup/setup.sh](https://github.com/thentnucyborg/CyborgRobot/tree/master/setup) for more. 

Run: 
```bash
find <path-to-catkin_ws/src/> -name '*.py' -exec chmod +x {} \;
```
to make all python files executable to be able to run them. Might need to do it with .sh files too (also catkin_ws/src/rosaria/cfg/RosAria.cfg)
