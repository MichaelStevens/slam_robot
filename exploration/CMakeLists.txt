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
  message_generation
)

add_message_files(
  FILES
  PointList.msg
)

add_service_files(
  FILES
  GenerateGoal.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
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
  message_runtime
  message_generation
)


###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
  ${tf_INCLUDE_DIRS}
)
add_executable(frontier_finder src/FrontierFinder.cpp src/Frontier.cpp)
target_link_libraries(frontier_finder ${catkin_LIBRARIES})

add_executable(goal_generator src/goal_generator_ros.cpp src/goal_generator.cpp)
target_link_libraries(goal_generator ${catkin_LIBRARIES})
add_dependencies(goal_generator exploration_gencpp)
