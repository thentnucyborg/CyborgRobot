cmake_minimum_required(VERSION 2.8.3)
project(ros_dnn_msgs)

find_package(catkin REQUIRED COMPONENTS
    rospy
    actionlib_msgs
    std_msgs
    sensor_msgs
    message_generation
)

add_message_files(
    DIRECTORY msg
    FILES
    Prediction.msg
    Predictions.msg
)

add_action_files(
    DIRECTORY action
    FILES
    Detect.action
)

generate_messages(
    DEPENDENCIES
    actionlib_msgs
    sensor_msgs
    std_msgs
)

catkin_package()
