cmake_minimum_required(VERSION 2.8.3)
project(ros_kalman_filter)

#set C++11 support
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

## Find catkin macros and libraries
find_package( catkin REQUIRED COMPONENTS 
      roscpp
      rospy
      std_msgs)

## System dependencies are found with CMake's conventions
find_package(OpenCV 2 REQUIRED)

##Qt5 dependences dont know why.....IF NOT ERROR COMPILE
find_package(Qt5Widgets)
find_package(Qt5Test)
find_package(Qt5Concurrent)
find_package(Qt5OpenGL)

#source files
SET(SRCS 
      src/ros_kalman_filter_node.cpp
      src/ros_kalman_filter_main.cpp)

## The catkin_package macro generates cmake config files for your package
catkin_package(
      CATKIN_DEPENDS 
            roscpp 
            rospy 
            std_msgs)

## Specify additional locations of header files
include_directories(${OpenCV_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

## Declare a cpp executable for pipol_tracker_node
add_executable(ros_kalman_filter ${SRCS})
#add_dependencies(pipol_tracker_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(ros_kalman_filter ${OpenCV_LIBS} ${catkin_LIBRARIES})

#install binaries
install(TARGETS ros_kalman_filter
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#Share launch and configs
foreach(dir launch config)
    install(DIRECTORY ${dir}/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
