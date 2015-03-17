cmake_minimum_required(VERSION 2.8.3)
project(mug_detector)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  cv_bridge
  geometry_msgs
  message_generation
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  Detections.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
