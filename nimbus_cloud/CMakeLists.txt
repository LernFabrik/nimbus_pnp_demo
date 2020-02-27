cmake_minimum_required(VERSION 2.8.3)
project(nimbus_cloud)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  pcl_conversions
  pcl_msgs
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  tf2
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)



catkin_package(
  INCLUDE_DIRS include
  LIBRARIES nimbus_cloud
  CATKIN_DEPENDS geometry_msgs pcl_conversions pcl_msgs pcl_ros roscpp rospy sensor_msgs tf2
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
  ${PCL_INCLUDE_DIRS}
)

#############################################################################
###################### !Executable!##########################################
#############################################################################
add_executable(nimbus_io_node src/nimbus_io.cpp)
target_link_libraries(nimbus_io_node ${catkin_LIBRARIES} ${PCL_INCLUDE_DIRS})