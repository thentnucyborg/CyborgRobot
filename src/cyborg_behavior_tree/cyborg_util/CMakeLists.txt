cmake_minimum_required(VERSION 2.8.3)
project(cyborg_util)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    geometry_msgs
)

catkin_python_setup()

catkin_package()
