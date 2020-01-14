# Install script for directory: /home/lassegoncz/catkin_ws/src/cyborg_ros_controller

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/lassegoncz/catkin_ws/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cyborg_controller/msg" TYPE FILE FILES
    "/home/lassegoncz/catkin_ws/src/cyborg_ros_controller/msg/EmotionalFeedback.msg"
    "/home/lassegoncz/catkin_ws/src/cyborg_ros_controller/msg/EmotionalState.msg"
    "/home/lassegoncz/catkin_ws/src/cyborg_ros_controller/msg/SystemState.msg"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cyborg_controller/srv" TYPE FILE FILES "/home/lassegoncz/catkin_ws/src/cyborg_ros_controller/srv/EmotionalStateService.srv")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cyborg_controller/action" TYPE FILE FILES "/home/lassegoncz/catkin_ws/src/cyborg_ros_controller/action/StateMachine.action")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cyborg_controller/msg" TYPE FILE FILES
    "/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineAction.msg"
    "/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineActionGoal.msg"
    "/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineActionResult.msg"
    "/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineActionFeedback.msg"
    "/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineGoal.msg"
    "/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineResult.msg"
    "/home/lassegoncz/catkin_ws/devel/share/cyborg_controller/msg/StateMachineFeedback.msg"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cyborg_controller/cmake" TYPE FILE FILES "/home/lassegoncz/catkin_ws/build/cyborg_ros_controller/catkin_generated/installspace/cyborg_controller-msg-paths.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/lassegoncz/catkin_ws/devel/include/cyborg_controller")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/lassegoncz/catkin_ws/devel/share/roseus/ros/cyborg_controller")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/lassegoncz/catkin_ws/devel/share/common-lisp/ros/cyborg_controller")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/lassegoncz/catkin_ws/devel/share/gennodejs/ros/cyborg_controller")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  execute_process(COMMAND "/usr/bin/python" -m compileall "/home/lassegoncz/catkin_ws/devel/lib/python2.7/dist-packages/cyborg_controller")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/lassegoncz/catkin_ws/devel/lib/python2.7/dist-packages/cyborg_controller")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cyborg_controller" TYPE PROGRAM FILES "/home/lassegoncz/catkin_ws/build/cyborg_ros_controller/catkin_generated/installspace/controller.py")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/lassegoncz/catkin_ws/build/cyborg_ros_controller/catkin_generated/installspace/cyborg_controller.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cyborg_controller/cmake" TYPE FILE FILES "/home/lassegoncz/catkin_ws/build/cyborg_ros_controller/catkin_generated/installspace/cyborg_controller-msg-extras.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cyborg_controller/cmake" TYPE FILE FILES
    "/home/lassegoncz/catkin_ws/build/cyborg_ros_controller/catkin_generated/installspace/cyborg_controllerConfig.cmake"
    "/home/lassegoncz/catkin_ws/build/cyborg_ros_controller/catkin_generated/installspace/cyborg_controllerConfig-version.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cyborg_controller" TYPE FILE FILES "/home/lassegoncz/catkin_ws/src/cyborg_ros_controller/package.xml")
endif()

