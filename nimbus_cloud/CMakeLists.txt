cmake_minimum_required(VERSION 2.8.3)
project(nimbus_cloud)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

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
find_package(Eigen3 REQUIRED)
#find_package(PCL 1.8 REQUIRED)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES nimbus_cloud 
  CATKIN_DEPENDS geometry_msgs pcl_conversions pcl_msgs pcl_ros roscpp rospy sensor_msgs tf2
  DEPENDS Boost EIGEN3 #PCL
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
link_directories(
  ${catkin_LIBRARY_DIRS}
  ${PCL_LIBRARY_DIRS}
)
add_definitions(${PCL_DEFINITIONS})

############################################################################
##################### !Add Library! ########################################
############################################################################
add_library(cloud_edit "src/cloud_edit.cpp")
add_dependencies(cloud_edit ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(cloud_edit ${catkin_LIBRARIES} ${PCL_INCLUDE_DIRS})


#############################################################################
###################### !Executable!##########################################
#############################################################################
add_executable(nimbus_io_node src/nimbus_io.cpp)
add_dependencies(nimbus_io_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(nimbus_io_node ${catkin_LIBRARIES} ${PCL_INCLUDE_DIRS})