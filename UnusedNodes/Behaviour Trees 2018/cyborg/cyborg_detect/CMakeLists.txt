cmake_minimum_required(VERSION 2.8.3)
project(cyborg_detect)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    cv_bridge
    image_transport
    sensor_msgs
    geometry_msgs
    std_msgs
    dynamic_reconfigure
    message_generation
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_message_files(
    DIRECTORY msg
    FILES
    Prediction.msg
    Predictions.msg
)

generate_dynamic_reconfigure_options(
    cfg/Detector.cfg
)

generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
)

catkin_package()

catkin_install_python(
    PROGRAMS
    src/main.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  data
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
