cmake_minimum_required(VERSION 2.8.3)
project(ros_dnn)

function(checkPackage package customMessage)
    set(varName "${package}_FOUND")
    if (NOT "${${varName}}")
        string(REPLACE "_" "-" aptPackage ${package})
        if("${customMessage}" STREQUAL "")
            message(FATAL_ERROR "\n\n ${package} is missing, please try to install it with:\n sudo apt-get install ros-$(rosversion -d)-${aptPackage}\n\n")
        else()
            message(FATAL_ERROR "\n\n ${customMessage} \n\n")
        endif()
    endif()
endfunction(checkPackage)

find_package(OpenCV 3 COMPONENTS core dnn highgui imgproc)
checkPackage("OPENCV_CORE" "OpenCV core not found")
checkPackage("OPENCV_DNN" "OpenCV dnn not found")
checkPackage("OPENCV_HIGHGUI" "OpenCV highgui not found")
checkPackage("OPENCV_IMGPROC" "OpenCV imgproc not found")

find_package(catkin REQUIRED COMPONENTS
    cv_bridge
    image_transport
    roscpp
    sensor_msgs
    dynamic_reconfigure
    nodelet
    ros_dnn_msgs
)

checkPackage("image_transport" "")
checkPackage("roscpp" "")
checkPackage("sensor_msgs" "")
checkPackage("dynamic_reconfigure" "")
checkPackage("nodelet" "")

generate_dynamic_reconfigure_options(
    cfg/ObjectDetector.cfg
)

catkin_package(
    CATKIN_DEPENDS
    cv_bridge
    roscpp
    sensor_msgs
    opencv
    image_transport
    ros_dnn_msgs
)

###############################################################################
# INCLUDES

# Specify locations of header files.
include_directories(
    include
	${catkin_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
)

link_directories(${OpenCV_LIBRARY_DIRS})

###############################################################################

###############################################################################
# EXECUTABLE

add_definitions(-std=c++11)

set(LINK_LIBRARIES
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS})

add_library(ObjectDetector src/object_detector_nodelet.cpp)
target_link_libraries(ObjectDetector ${LINK_LIBRARIES})
#add_dependencies(ObjectDetector ${PROJECT_NAME}_gencfg)

add_executable(object_detector_node src/object_detector_node.cpp)
target_link_libraries(object_detector_node ObjectDetector ${LINK_LIBRARIES})
#add_dependencies(zed_wrapper_node ${PROJECT_NAME}_gencfg)

###############################################################################
# INSTALL

install(TARGETS
  ObjectDetector
  object_detector_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(FILES
  nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY
  launch
  config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
