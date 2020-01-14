# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "cyborg_controller: 10 messages, 1 services")

set(MSG_I_FLAGS "-Icyborg_controller:/home/lassegoncz/catkin_ws/src/cyborg_ros_controller/msg;-Icyborg_controller:/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg;-Iactionlib_msgs:/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(cyborg_controller_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineFeedback.msg" NAME_WE)
add_custom_target(_cyborg_controller_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "cyborg_controller" "/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineFeedback.msg" ""
)

get_filename_component(_filename "/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineActionFeedback.msg" NAME_WE)
add_custom_target(_cyborg_controller_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "cyborg_controller" "/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineActionFeedback.msg" "actionlib_msgs/GoalID:std_msgs/Header:cyborg_controller/StateMachineFeedback:actionlib_msgs/GoalStatus"
)

get_filename_component(_filename "/home/lassegoncz/catkin_ws/src/cyborg_ros_controller/srv/EmotionalStateService.srv" NAME_WE)
add_custom_target(_cyborg_controller_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "cyborg_controller" "/home/lassegoncz/catkin_ws/src/cyborg_ros_controller/srv/EmotionalStateService.srv" ""
)

get_filename_component(_filename "/home/lassegoncz/catkin_ws/src/cyborg_ros_controller/msg/SystemState.msg" NAME_WE)
add_custom_target(_cyborg_controller_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "cyborg_controller" "/home/lassegoncz/catkin_ws/src/cyborg_ros_controller/msg/SystemState.msg" ""
)

get_filename_component(_filename "/home/lassegoncz/catkin_ws/src/cyborg_ros_controller/msg/EmotionalFeedback.msg" NAME_WE)
add_custom_target(_cyborg_controller_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "cyborg_controller" "/home/lassegoncz/catkin_ws/src/cyborg_ros_controller/msg/EmotionalFeedback.msg" ""
)

get_filename_component(_filename "/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineGoal.msg" NAME_WE)
add_custom_target(_cyborg_controller_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "cyborg_controller" "/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineGoal.msg" ""
)

get_filename_component(_filename "/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineAction.msg" NAME_WE)
add_custom_target(_cyborg_controller_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "cyborg_controller" "/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineAction.msg" "cyborg_controller/StateMachineActionResult:cyborg_controller/StateMachineGoal:actionlib_msgs/GoalStatus:cyborg_controller/StateMachineActionGoal:cyborg_controller/StateMachineActionFeedback:cyborg_controller/StateMachineResult:cyborg_controller/StateMachineFeedback:actionlib_msgs/GoalID:std_msgs/Header"
)

get_filename_component(_filename "/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineResult.msg" NAME_WE)
add_custom_target(_cyborg_controller_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "cyborg_controller" "/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineResult.msg" ""
)

get_filename_component(_filename "/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineActionResult.msg" NAME_WE)
add_custom_target(_cyborg_controller_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "cyborg_controller" "/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineActionResult.msg" "cyborg_controller/StateMachineResult:actionlib_msgs/GoalID:std_msgs/Header:actionlib_msgs/GoalStatus"
)

get_filename_component(_filename "/home/lassegoncz/catkin_ws/src/cyborg_ros_controller/msg/EmotionalState.msg" NAME_WE)
add_custom_target(_cyborg_controller_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "cyborg_controller" "/home/lassegoncz/catkin_ws/src/cyborg_ros_controller/msg/EmotionalState.msg" ""
)

get_filename_component(_filename "/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineActionGoal.msg" NAME_WE)
add_custom_target(_cyborg_controller_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "cyborg_controller" "/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineActionGoal.msg" "actionlib_msgs/GoalID:cyborg_controller/StateMachineGoal:std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(cyborg_controller
  "/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cyborg_controller
)
_generate_msg_cpp(cyborg_controller
  "/home/lassegoncz/catkin_ws/src/cyborg_ros_controller/msg/SystemState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cyborg_controller
)
_generate_msg_cpp(cyborg_controller
  "/home/lassegoncz/catkin_ws/src/cyborg_ros_controller/msg/EmotionalFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cyborg_controller
)
_generate_msg_cpp(cyborg_controller
  "/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cyborg_controller
)
_generate_msg_cpp(cyborg_controller
  "/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cyborg_controller
)
_generate_msg_cpp(cyborg_controller
  "/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineAction.msg"
  "${MSG_I_FLAGS}"
  "/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineActionResult.msg;/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineGoal.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineActionGoal.msg;/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineActionFeedback.msg;/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineResult.msg;/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineFeedback.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cyborg_controller
)
_generate_msg_cpp(cyborg_controller
  "/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineGoal.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cyborg_controller
)
_generate_msg_cpp(cyborg_controller
  "/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineActionResult.msg"
  "${MSG_I_FLAGS}"
  "/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineResult.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cyborg_controller
)
_generate_msg_cpp(cyborg_controller
  "/home/lassegoncz/catkin_ws/src/cyborg_ros_controller/msg/EmotionalState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cyborg_controller
)
_generate_msg_cpp(cyborg_controller
  "/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineFeedback.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cyborg_controller
)

### Generating Services
_generate_srv_cpp(cyborg_controller
  "/home/lassegoncz/catkin_ws/src/cyborg_ros_controller/srv/EmotionalStateService.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cyborg_controller
)

### Generating Module File
_generate_module_cpp(cyborg_controller
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cyborg_controller
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(cyborg_controller_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(cyborg_controller_generate_messages cyborg_controller_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineFeedback.msg" NAME_WE)
add_dependencies(cyborg_controller_generate_messages_cpp _cyborg_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineActionFeedback.msg" NAME_WE)
add_dependencies(cyborg_controller_generate_messages_cpp _cyborg_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lassegoncz/catkin_ws/src/cyborg_ros_controller/srv/EmotionalStateService.srv" NAME_WE)
add_dependencies(cyborg_controller_generate_messages_cpp _cyborg_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lassegoncz/catkin_ws/src/cyborg_ros_controller/msg/SystemState.msg" NAME_WE)
add_dependencies(cyborg_controller_generate_messages_cpp _cyborg_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lassegoncz/catkin_ws/src/cyborg_ros_controller/msg/EmotionalFeedback.msg" NAME_WE)
add_dependencies(cyborg_controller_generate_messages_cpp _cyborg_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineGoal.msg" NAME_WE)
add_dependencies(cyborg_controller_generate_messages_cpp _cyborg_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineAction.msg" NAME_WE)
add_dependencies(cyborg_controller_generate_messages_cpp _cyborg_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineResult.msg" NAME_WE)
add_dependencies(cyborg_controller_generate_messages_cpp _cyborg_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineActionResult.msg" NAME_WE)
add_dependencies(cyborg_controller_generate_messages_cpp _cyborg_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lassegoncz/catkin_ws/src/cyborg_ros_controller/msg/EmotionalState.msg" NAME_WE)
add_dependencies(cyborg_controller_generate_messages_cpp _cyborg_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineActionGoal.msg" NAME_WE)
add_dependencies(cyborg_controller_generate_messages_cpp _cyborg_controller_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(cyborg_controller_gencpp)
add_dependencies(cyborg_controller_gencpp cyborg_controller_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS cyborg_controller_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(cyborg_controller
  "/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/cyborg_controller
)
_generate_msg_eus(cyborg_controller
  "/home/lassegoncz/catkin_ws/src/cyborg_ros_controller/msg/SystemState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/cyborg_controller
)
_generate_msg_eus(cyborg_controller
  "/home/lassegoncz/catkin_ws/src/cyborg_ros_controller/msg/EmotionalFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/cyborg_controller
)
_generate_msg_eus(cyborg_controller
  "/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/cyborg_controller
)
_generate_msg_eus(cyborg_controller
  "/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/cyborg_controller
)
_generate_msg_eus(cyborg_controller
  "/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineAction.msg"
  "${MSG_I_FLAGS}"
  "/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineActionResult.msg;/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineGoal.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineActionGoal.msg;/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineActionFeedback.msg;/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineResult.msg;/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineFeedback.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/cyborg_controller
)
_generate_msg_eus(cyborg_controller
  "/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineGoal.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/cyborg_controller
)
_generate_msg_eus(cyborg_controller
  "/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineActionResult.msg"
  "${MSG_I_FLAGS}"
  "/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineResult.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/cyborg_controller
)
_generate_msg_eus(cyborg_controller
  "/home/lassegoncz/catkin_ws/src/cyborg_ros_controller/msg/EmotionalState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/cyborg_controller
)
_generate_msg_eus(cyborg_controller
  "/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineFeedback.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/cyborg_controller
)

### Generating Services
_generate_srv_eus(cyborg_controller
  "/home/lassegoncz/catkin_ws/src/cyborg_ros_controller/srv/EmotionalStateService.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/cyborg_controller
)

### Generating Module File
_generate_module_eus(cyborg_controller
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/cyborg_controller
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(cyborg_controller_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(cyborg_controller_generate_messages cyborg_controller_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineFeedback.msg" NAME_WE)
add_dependencies(cyborg_controller_generate_messages_eus _cyborg_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineActionFeedback.msg" NAME_WE)
add_dependencies(cyborg_controller_generate_messages_eus _cyborg_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lassegoncz/catkin_ws/src/cyborg_ros_controller/srv/EmotionalStateService.srv" NAME_WE)
add_dependencies(cyborg_controller_generate_messages_eus _cyborg_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lassegoncz/catkin_ws/src/cyborg_ros_controller/msg/SystemState.msg" NAME_WE)
add_dependencies(cyborg_controller_generate_messages_eus _cyborg_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lassegoncz/catkin_ws/src/cyborg_ros_controller/msg/EmotionalFeedback.msg" NAME_WE)
add_dependencies(cyborg_controller_generate_messages_eus _cyborg_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineGoal.msg" NAME_WE)
add_dependencies(cyborg_controller_generate_messages_eus _cyborg_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineAction.msg" NAME_WE)
add_dependencies(cyborg_controller_generate_messages_eus _cyborg_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineResult.msg" NAME_WE)
add_dependencies(cyborg_controller_generate_messages_eus _cyborg_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineActionResult.msg" NAME_WE)
add_dependencies(cyborg_controller_generate_messages_eus _cyborg_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lassegoncz/catkin_ws/src/cyborg_ros_controller/msg/EmotionalState.msg" NAME_WE)
add_dependencies(cyborg_controller_generate_messages_eus _cyborg_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineActionGoal.msg" NAME_WE)
add_dependencies(cyborg_controller_generate_messages_eus _cyborg_controller_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(cyborg_controller_geneus)
add_dependencies(cyborg_controller_geneus cyborg_controller_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS cyborg_controller_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(cyborg_controller
  "/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cyborg_controller
)
_generate_msg_lisp(cyborg_controller
  "/home/lassegoncz/catkin_ws/src/cyborg_ros_controller/msg/SystemState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cyborg_controller
)
_generate_msg_lisp(cyborg_controller
  "/home/lassegoncz/catkin_ws/src/cyborg_ros_controller/msg/EmotionalFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cyborg_controller
)
_generate_msg_lisp(cyborg_controller
  "/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cyborg_controller
)
_generate_msg_lisp(cyborg_controller
  "/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cyborg_controller
)
_generate_msg_lisp(cyborg_controller
  "/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineAction.msg"
  "${MSG_I_FLAGS}"
  "/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineActionResult.msg;/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineGoal.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineActionGoal.msg;/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineActionFeedback.msg;/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineResult.msg;/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineFeedback.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cyborg_controller
)
_generate_msg_lisp(cyborg_controller
  "/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineGoal.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cyborg_controller
)
_generate_msg_lisp(cyborg_controller
  "/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineActionResult.msg"
  "${MSG_I_FLAGS}"
  "/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineResult.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cyborg_controller
)
_generate_msg_lisp(cyborg_controller
  "/home/lassegoncz/catkin_ws/src/cyborg_ros_controller/msg/EmotionalState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cyborg_controller
)
_generate_msg_lisp(cyborg_controller
  "/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineFeedback.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cyborg_controller
)

### Generating Services
_generate_srv_lisp(cyborg_controller
  "/home/lassegoncz/catkin_ws/src/cyborg_ros_controller/srv/EmotionalStateService.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cyborg_controller
)

### Generating Module File
_generate_module_lisp(cyborg_controller
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cyborg_controller
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(cyborg_controller_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(cyborg_controller_generate_messages cyborg_controller_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineFeedback.msg" NAME_WE)
add_dependencies(cyborg_controller_generate_messages_lisp _cyborg_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineActionFeedback.msg" NAME_WE)
add_dependencies(cyborg_controller_generate_messages_lisp _cyborg_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lassegoncz/catkin_ws/src/cyborg_ros_controller/srv/EmotionalStateService.srv" NAME_WE)
add_dependencies(cyborg_controller_generate_messages_lisp _cyborg_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lassegoncz/catkin_ws/src/cyborg_ros_controller/msg/SystemState.msg" NAME_WE)
add_dependencies(cyborg_controller_generate_messages_lisp _cyborg_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lassegoncz/catkin_ws/src/cyborg_ros_controller/msg/EmotionalFeedback.msg" NAME_WE)
add_dependencies(cyborg_controller_generate_messages_lisp _cyborg_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineGoal.msg" NAME_WE)
add_dependencies(cyborg_controller_generate_messages_lisp _cyborg_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineAction.msg" NAME_WE)
add_dependencies(cyborg_controller_generate_messages_lisp _cyborg_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineResult.msg" NAME_WE)
add_dependencies(cyborg_controller_generate_messages_lisp _cyborg_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineActionResult.msg" NAME_WE)
add_dependencies(cyborg_controller_generate_messages_lisp _cyborg_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lassegoncz/catkin_ws/src/cyborg_ros_controller/msg/EmotionalState.msg" NAME_WE)
add_dependencies(cyborg_controller_generate_messages_lisp _cyborg_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineActionGoal.msg" NAME_WE)
add_dependencies(cyborg_controller_generate_messages_lisp _cyborg_controller_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(cyborg_controller_genlisp)
add_dependencies(cyborg_controller_genlisp cyborg_controller_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS cyborg_controller_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(cyborg_controller
  "/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/cyborg_controller
)
_generate_msg_nodejs(cyborg_controller
  "/home/lassegoncz/catkin_ws/src/cyborg_ros_controller/msg/SystemState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/cyborg_controller
)
_generate_msg_nodejs(cyborg_controller
  "/home/lassegoncz/catkin_ws/src/cyborg_ros_controller/msg/EmotionalFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/cyborg_controller
)
_generate_msg_nodejs(cyborg_controller
  "/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/cyborg_controller
)
_generate_msg_nodejs(cyborg_controller
  "/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/cyborg_controller
)
_generate_msg_nodejs(cyborg_controller
  "/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineAction.msg"
  "${MSG_I_FLAGS}"
  "/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineActionResult.msg;/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineGoal.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineActionGoal.msg;/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineActionFeedback.msg;/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineResult.msg;/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineFeedback.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/cyborg_controller
)
_generate_msg_nodejs(cyborg_controller
  "/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineGoal.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/cyborg_controller
)
_generate_msg_nodejs(cyborg_controller
  "/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineActionResult.msg"
  "${MSG_I_FLAGS}"
  "/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineResult.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/cyborg_controller
)
_generate_msg_nodejs(cyborg_controller
  "/home/lassegoncz/catkin_ws/src/cyborg_ros_controller/msg/EmotionalState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/cyborg_controller
)
_generate_msg_nodejs(cyborg_controller
  "/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineFeedback.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/cyborg_controller
)

### Generating Services
_generate_srv_nodejs(cyborg_controller
  "/home/lassegoncz/catkin_ws/src/cyborg_ros_controller/srv/EmotionalStateService.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/cyborg_controller
)

### Generating Module File
_generate_module_nodejs(cyborg_controller
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/cyborg_controller
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(cyborg_controller_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(cyborg_controller_generate_messages cyborg_controller_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineFeedback.msg" NAME_WE)
add_dependencies(cyborg_controller_generate_messages_nodejs _cyborg_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineActionFeedback.msg" NAME_WE)
add_dependencies(cyborg_controller_generate_messages_nodejs _cyborg_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lassegoncz/catkin_ws/src/cyborg_ros_controller/srv/EmotionalStateService.srv" NAME_WE)
add_dependencies(cyborg_controller_generate_messages_nodejs _cyborg_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lassegoncz/catkin_ws/src/cyborg_ros_controller/msg/SystemState.msg" NAME_WE)
add_dependencies(cyborg_controller_generate_messages_nodejs _cyborg_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lassegoncz/catkin_ws/src/cyborg_ros_controller/msg/EmotionalFeedback.msg" NAME_WE)
add_dependencies(cyborg_controller_generate_messages_nodejs _cyborg_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineGoal.msg" NAME_WE)
add_dependencies(cyborg_controller_generate_messages_nodejs _cyborg_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineAction.msg" NAME_WE)
add_dependencies(cyborg_controller_generate_messages_nodejs _cyborg_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineResult.msg" NAME_WE)
add_dependencies(cyborg_controller_generate_messages_nodejs _cyborg_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineActionResult.msg" NAME_WE)
add_dependencies(cyborg_controller_generate_messages_nodejs _cyborg_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lassegoncz/catkin_ws/src/cyborg_ros_controller/msg/EmotionalState.msg" NAME_WE)
add_dependencies(cyborg_controller_generate_messages_nodejs _cyborg_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineActionGoal.msg" NAME_WE)
add_dependencies(cyborg_controller_generate_messages_nodejs _cyborg_controller_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(cyborg_controller_gennodejs)
add_dependencies(cyborg_controller_gennodejs cyborg_controller_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS cyborg_controller_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(cyborg_controller
  "/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cyborg_controller
)
_generate_msg_py(cyborg_controller
  "/home/lassegoncz/catkin_ws/src/cyborg_ros_controller/msg/SystemState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cyborg_controller
)
_generate_msg_py(cyborg_controller
  "/home/lassegoncz/catkin_ws/src/cyborg_ros_controller/msg/EmotionalFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cyborg_controller
)
_generate_msg_py(cyborg_controller
  "/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cyborg_controller
)
_generate_msg_py(cyborg_controller
  "/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cyborg_controller
)
_generate_msg_py(cyborg_controller
  "/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineAction.msg"
  "${MSG_I_FLAGS}"
  "/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineActionResult.msg;/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineGoal.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineActionGoal.msg;/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineActionFeedback.msg;/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineResult.msg;/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineFeedback.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cyborg_controller
)
_generate_msg_py(cyborg_controller
  "/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineGoal.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cyborg_controller
)
_generate_msg_py(cyborg_controller
  "/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineActionResult.msg"
  "${MSG_I_FLAGS}"
  "/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineResult.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cyborg_controller
)
_generate_msg_py(cyborg_controller
  "/home/lassegoncz/catkin_ws/src/cyborg_ros_controller/msg/EmotionalState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cyborg_controller
)
_generate_msg_py(cyborg_controller
  "/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineFeedback.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cyborg_controller
)

### Generating Services
_generate_srv_py(cyborg_controller
  "/home/lassegoncz/catkin_ws/src/cyborg_ros_controller/srv/EmotionalStateService.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cyborg_controller
)

### Generating Module File
_generate_module_py(cyborg_controller
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cyborg_controller
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(cyborg_controller_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(cyborg_controller_generate_messages cyborg_controller_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineFeedback.msg" NAME_WE)
add_dependencies(cyborg_controller_generate_messages_py _cyborg_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineActionFeedback.msg" NAME_WE)
add_dependencies(cyborg_controller_generate_messages_py _cyborg_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lassegoncz/catkin_ws/src/cyborg_ros_controller/srv/EmotionalStateService.srv" NAME_WE)
add_dependencies(cyborg_controller_generate_messages_py _cyborg_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lassegoncz/catkin_ws/src/cyborg_ros_controller/msg/SystemState.msg" NAME_WE)
add_dependencies(cyborg_controller_generate_messages_py _cyborg_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lassegoncz/catkin_ws/src/cyborg_ros_controller/msg/EmotionalFeedback.msg" NAME_WE)
add_dependencies(cyborg_controller_generate_messages_py _cyborg_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineGoal.msg" NAME_WE)
add_dependencies(cyborg_controller_generate_messages_py _cyborg_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineAction.msg" NAME_WE)
add_dependencies(cyborg_controller_generate_messages_py _cyborg_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineResult.msg" NAME_WE)
add_dependencies(cyborg_controller_generate_messages_py _cyborg_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineActionResult.msg" NAME_WE)
add_dependencies(cyborg_controller_generate_messages_py _cyborg_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lassegoncz/catkin_ws/src/cyborg_ros_controller/msg/EmotionalState.msg" NAME_WE)
add_dependencies(cyborg_controller_generate_messages_py _cyborg_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineActionGoal.msg" NAME_WE)
add_dependencies(cyborg_controller_generate_messages_py _cyborg_controller_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(cyborg_controller_genpy)
add_dependencies(cyborg_controller_genpy cyborg_controller_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS cyborg_controller_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cyborg_controller)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cyborg_controller
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(cyborg_controller_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET actionlib_msgs_generate_messages_cpp)
  add_dependencies(cyborg_controller_generate_messages_cpp actionlib_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/cyborg_controller)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/cyborg_controller
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(cyborg_controller_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET actionlib_msgs_generate_messages_eus)
  add_dependencies(cyborg_controller_generate_messages_eus actionlib_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cyborg_controller)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cyborg_controller
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(cyborg_controller_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET actionlib_msgs_generate_messages_lisp)
  add_dependencies(cyborg_controller_generate_messages_lisp actionlib_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/cyborg_controller)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/cyborg_controller
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(cyborg_controller_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET actionlib_msgs_generate_messages_nodejs)
  add_dependencies(cyborg_controller_generate_messages_nodejs actionlib_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cyborg_controller)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cyborg_controller\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cyborg_controller
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(cyborg_controller_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET actionlib_msgs_generate_messages_py)
  add_dependencies(cyborg_controller_generate_messages_py actionlib_msgs_generate_messages_py)
endif()
