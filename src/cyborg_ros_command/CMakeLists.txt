cmake_minimum_required(VERSION 2.8.3)
project(cyborg_command)

find_package( catkin REQUIRED COMPONENTS
	roscpp
	rospy
	std_msgs
	message_generation
	actionlib_msgs
)

generate_messages( DEPENDENCIES 
	std_msgs
	actionlib_msgs
)

catkin_install_python( PROGRAMS src/command.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

catkin_package( CATKIN_DEPENDS 
	message_runtime
	actionlib_msgs
	roscpp
	rospy
	std_msgs
)

include_directories( ${catkin_INCLUDE_DIRS} )
