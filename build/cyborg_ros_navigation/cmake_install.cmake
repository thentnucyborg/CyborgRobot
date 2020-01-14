# Install script for directory: /home/lassegoncz/catkin_ws/src/cyborg_ros_navigation

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cyborg_navigation/action" TYPE FILE FILES "/home/lassegoncz/catkin_ws/src/cyborg_ros_navigation/action/Navigation.action")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cyborg_navigation/msg" TYPE FILE FILES
    "/home/lassegoncz/catkin_ws/devel/share/cyborg_navigation/msg/NavigationAction.msg"
    "/home/lassegoncz/catkin_ws/devel/share/cyborg_navigation/msg/NavigationActionGoal.msg"
    "/home/lassegoncz/catkin_ws/devel/share/cyborg_navigation/msg/NavigationActionResult.msg"
    "/home/lassegoncz/catkin_ws/devel/share/cyborg_navigation/msg/NavigationActionFeedback.msg"
    "/home/lassegoncz/catkin_ws/devel/share/cyborg_navigation/msg/NavigationGoal.msg"
    "/home/lassegoncz/catkin_ws/devel/share/cyborg_navigation/msg/NavigationResult.msg"
    "/home/lassegoncz/catkin_ws/devel/share/cyborg_navigation/msg/NavigationFeedback.msg"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cyborg_navigation/cmake" TYPE FILE FILES "/home/lassegoncz/catkin_ws/build/cyborg_ros_navigation/catkin_generated/installspace/cyborg_navigation-msg-paths.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/lassegoncz/catkin_ws/devel/include/cyborg_navigation")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/lassegoncz/catkin_ws/devel/share/roseus/ros/cyborg_navigation")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/lassegoncz/catkin_ws/devel/share/common-lisp/ros/cyborg_navigation")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/lassegoncz/catkin_ws/devel/share/gennodejs/ros/cyborg_navigation")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  execute_process(COMMAND "/usr/bin/python" -m compileall "/home/lassegoncz/catkin_ws/devel/lib/python2.7/dist-packages/cyborg_navigation")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/lassegoncz/catkin_ws/devel/lib/python2.7/dist-packages/cyborg_navigation")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cyborg_navigation" TYPE PROGRAM FILES "/home/lassegoncz/catkin_ws/build/cyborg_ros_navigation/catkin_generated/installspace/navigation.py")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/lassegoncz/catkin_ws/build/cyborg_ros_navigation/catkin_generated/installspace/cyborg_navigation.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cyborg_navigation/cmake" TYPE FILE FILES "/home/lassegoncz/catkin_ws/build/cyborg_ros_navigation/catkin_generated/installspace/cyborg_navigation-msg-extras.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cyborg_navigation/cmake" TYPE FILE FILES
    "/home/lassegoncz/catkin_ws/build/cyborg_ros_navigation/catkin_generated/installspace/cyborg_navigationConfig.cmake"
    "/home/lassegoncz/catkin_ws/build/cyborg_ros_navigation/catkin_generated/installspace/cyborg_navigationConfig-version.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cyborg_navigation" TYPE FILE FILES "/home/lassegoncz/catkin_ws/src/cyborg_ros_navigation/package.xml")
endif()

