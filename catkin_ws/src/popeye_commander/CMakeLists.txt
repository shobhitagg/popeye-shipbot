cmake_minimum_required(VERSION 3.0.2)
project(popeye_commander)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  control_msgs
  genmsg
  geometry_msgs
  hebi_cpp_api
  message_generation
  nav_msgs
  roscpp
  roslib
  sensor_msgs
  std_msgs
  tf
  trajectory_msgs
  eigen_conversions
 )

## Generate added messages and services with any dependencies listed here

add_message_files(
        FILES
        gripoTarget.msg
	visionRequest.msg
	hebiCommand.msg
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  control_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  trajectory_msgs
  popeye_commander
)

#roslaunch_add_file_check(launch)

catkin_package(
	INCLUDE_DIRS include
	CATKIN_DEPENDS  
	actionlib
	actionlib_msgs
	control_msgs
	geometry_msgs
	hebi_cpp_api
	message_runtime
	nav_msgs
	roscpp
	sensor_msgs
	std_msgs
	tf
	trajectory_msgs
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}	
)

add_executable(commander0 src/main.cpp src/Commander.cpp)
add_dependencies(commander0 popeye_commander_generate_messages_cpp)
target_link_libraries(commander0 ${catkin_LIBRARIES})

add_executable(HebiCommand0 src/hebi_main.cpp src/HebiCommander.cpp)
add_dependencies(HebiCommand0 popeye_commander_generate_messages_cpp)
target_link_libraries(HebiCommand0 ${catkin_LIBRARIES})

add_executable(GripoCommand0 src/gripo_main.cpp src/GripoCommander.cpp)
add_dependencies(GripoCommand0 popeye_commander_generate_messages_cpp)
target_link_libraries(GripoCommand0 ${catkin_LIBRARIES})

