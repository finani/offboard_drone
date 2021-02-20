cmake_minimum_required(VERSION 2.8.3)
project(offboard_drone)

find_package(
  catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  geometry_msgs
  tf
)

add_definitions(-std=c++11)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
  roscpp
  std_msgs
  nav_msgs
  geometry_msgs
  tf
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(
	offboard_node
  src/offboard_node.cpp
  src/px4drone.cpp
)

target_link_libraries(
	offboard_node
	${catkin_LIBRARIES}
)
