cmake_minimum_required(VERSION 3.1.3)
project(pop_hebi)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Find catkin macros and libraries
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

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

## Generate added messages and services with any dependencies listed here
 add_message_files(
	FILES
	gripoTarget.msg
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
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
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

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  .
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

# Different nodes for the arm kits
add_executable(3dof src/3dof.cpp)
add_dependencies(3dof beginner_tutorials_generate_messages_cpp)
target_link_libraries(3dof ${catkin_LIBRARIES})

add_executable(3dofPos src/3dofPos.cpp)
add_dependencies(3dofPos beginner_tutorials_generate_messages_cpp)
target_link_libraries(3dofPos ${catkin_LIBRARIES})

add_executable(3dofV2 src/3dof_v2.cpp)
add_dependencies(3dofV2 beginner_tutorials_generate_messages_cpp)
target_link_libraries(3dofV2 ${catkin_LIBRARIES})

add_executable(HebiCommand src/hebi_main.cpp src/HebiCommander.cpp)
add_dependencies(HebiCommand beginner_tutorials_generate_messages_cpp)
target_link_libraries(HebiCommand ${catkin_LIBRARIES})

add_executable(GripoCommand src/gripo_main.cpp src/GripoCommander.cpp)
add_dependencies(GripoCommand beginner_tutorials_generate_messages_cpp)
target_link_libraries(GripoCommand ${catkin_LIBRARIES})
