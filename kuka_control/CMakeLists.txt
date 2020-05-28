cmake_minimum_required(VERSION 3.0.2)
project(kuka_control)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  control_msgs
  geometry_msgs
  moveit_core
  moveit_msgs
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_visual_tools
  pcl_conversions
  pcl_msgs
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  tf
  tf2
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)



catkin_package(
  INCLUDE_DIRS include
  LIBRARIES kuka_control
  CATKIN_DEPENDS actionlib actionlib_msgs control_msgs geometry_msgs moveit_core moveit_msgs moveit_ros_planning moveit_ros_planning_interface moveit_visual_tools pcl_conversions pcl_msgs pcl_ros roscpp rospy sensor_msgs tf tf2
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

################# Add Library #############
add_library(manipulation src/manipulation.cpp)
add_dependencies(manipulation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(manipulation ${catkin_LIBRARIES})

################ Executable ###############
###########################################
add_executable(pnp_node src/main.cpp)
target_link_libraries(pnp_node  manipulation ${catkin_LIBRARIES})
