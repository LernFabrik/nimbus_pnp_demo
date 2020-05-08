cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

project(object_recognition)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})

find_package(PCL 1.10 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Find right hdf5 file
include (CheckIncludeFile)
check_include_file ("hdf5/serial/hdf5.h" HDF5_SERIAL)
if (HDF5_SERIAL)
  set (HDF5 "hdf5_serial")
  include_directories ("/usr/include/hdf5/serial")
endif (HDF5_SERIAL)

find_package(FLANN REQUIRED)

include_directories(SYSTEM
)


find_package(FLANN REQUIRED)
include_directories(${FLANN_INCLUDE_DIRS})

include_directories(SYSTEM ${HDF5_INCLUDE_DIR})

add_executable(vfh_detection src/vfh_detection.cpp)
target_link_libraries(vfh_detection ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${FLANN_LIBRARIES} ${HDF5})

