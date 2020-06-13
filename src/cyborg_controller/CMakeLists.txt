cmake_minimum_required(VERSION 2.8.3)
project(cyborg_controller)

find_package( catkin REQUIRED COMPONENTS
	roscpp
	rospy
	std_msgs
	message_generation
	actionlib_msgs
)

add_message_files( FILES
	EmotionalFeedback.msg
	EmotionalState.msg
	SystemState.msg
)

add_service_files( FILES 
	EmotionalStateService.srv
)

add_action_files(
	DIRECTORY action
	FILES StateMachine.action
)

generate_messages( DEPENDENCIES 
	std_msgs
	actionlib_msgs
)

catkin_install_python( PROGRAMS src/controller.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

catkin_package( CATKIN_DEPENDS 
	message_runtime
	actionlib_msgs
	roscpp
	rospy
	std_msgs
)

include_directories( ${catkin_INCLUDE_DIRS} )