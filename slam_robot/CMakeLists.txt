cmake_minimum_required(VERSION 2.8.3)
project(slam_robot)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    geometry_msgs
    message_generation
)

add_service_files(
 FILES
 Move.srv
 SpinPtu.srv
)

generate_messages(
   DEPENDENCIES
   geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
