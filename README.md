# CyborgRobot
The NTNU Cyborg Robot repo

## Starting with the repo ##
1. create a catkin workspace 
2. clone this repo into the workspace
3. go into the folder named CyborgRobot (this repo) and show any hidden files
4. move ALL files and folders, including any hidden ones, to the catkin_ws (meaning just outside of CyborgRobot)
You should now be abele to use the workspace and git without any problems.

## Installs needed ##
- Check out the README-files for each node to learn what dependecies and installs are needed for each.
- Also, check out "pioneerlx_setup/setup.sh" for more. 

- Run: "find <path-to-catkin_ws/src/> -name '*.py' -exec chmod +x {} \;"  to make all python files executable to be able to run them. Might need to do it with .sh files too (also catkin_ws/src/rosaria/cfg/RosAria.cfg)

