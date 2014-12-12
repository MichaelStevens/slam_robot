cmake_minimum_required(VERSION 2.8.3)
project(exploration)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  pluginlib
  geometry_msgs
  nav_msgs
  visualization_msgs
  nav2d_navigator
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS
  roscpp
	tf
	pluginlib
	geometry_msgs
	nav_msgs
	visualization_msgs
	nav2d_navigator
)

###########
## Build ##
###########

include_directories(
  include/${PROJECT_NAME}/
  ${catkin_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
  ${tf_INCLUDE_DIRS}
)
add_executable(exploration src/Exploration.cpp src/FrontierFinder.cpp)
target_link_libraries(exploration ${catkin_LIBRARIES})
