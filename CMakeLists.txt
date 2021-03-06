cmake_minimum_required(VERSION 2.8.3)
project(ros_img_processor)

#set C++11 support
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

## Find catkin macros and libraries
find_package( catkin REQUIRED COMPONENTS 
      roscpp
      rospy
      image_transport
      cv_bridge
      dynamic_reconfigure
      message_generation)

## System dependencies are found with CMake's conventions
find_package(OpenCV REQUIRED)

add_message_files(
  FILES
  CircleDetection.msg
  CircleDetectionStamped.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

#source files
SET(SRCS 
      src/ros_img_processor_node.cpp
      src/ros_img_processor_main.cpp)

generate_dynamic_reconfigure_options(
    cfg/CircleDetector.cfg
)

## The catkin_package macro generates cmake config files for your package
catkin_package(
      CATKIN_DEPENDS 
            roscpp 
            rospy 
            image_transport 
            cv_bridge
            dynamic_reconfigure
            std_msgs
            message_runtime)

## Specify additional locations of header files
include_directories(${OpenCV_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

## Declare a cpp executable for pipol_tracker_node
add_executable(ros_img_processor ${SRCS})
add_executable(tracker src/tracker.cpp)

#add_dependencies(ros_img_processor ${${PROJECT_NAME}_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp ${PROJECT_NAME}_gencfg)
#add_dependencies(ros_img_processor ${PROJECT_NAME}_gencfg)
target_link_libraries(ros_img_processor ${OpenCV_LIBS} ${catkin_LIBRARIES})
target_link_libraries(tracker ${OpenCV_LIBS} ${catkin_LIBRARIES})

#install binaries
install(TARGETS ros_img_processor
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#Share launch and configs
foreach(dir launch config)
    install(DIRECTORY ${dir}/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
