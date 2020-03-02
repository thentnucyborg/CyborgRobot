cmake_minimum_required(VERSION 2.8.3)
project(cyborg_bt)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    cyborg_msgs
)

catkin_package()

catkin_install_python(
    PROGRAMS
    src/main.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
    DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
