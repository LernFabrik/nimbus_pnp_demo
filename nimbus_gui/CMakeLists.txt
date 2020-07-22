cmake_minimum_required(VERSION 3.0.2)
project(nimbus_gui)

add_definitions(-std=c++11)

find_package(Qt5Widgets REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(Qt5Gui REQUIRED)



set(${PROJECT_NAME}_HDRS
  ui/accept_pose.h
)

set(${PROJECT_NAME}_UIS
  ui/accept_pose.ui
)

add_definitions(-DQT_NO_KEYWORDS)
qt5_wrap_cpp(${PROJECT_NAME}_MOCS ${${PROJECT_NAME}_HDRS})
qt5_wrap_ui(${PROJECT_NAME}_UIS_H ${${PROJECT_NAME}_UIS})

include_directories(${CMAKE_CURRENT_BINARY_DIR} ${catkin_INCLUDE_DIRS})

## Declare a cpp library
add_library(${PROJECT_NAME}
  ${${PROJECT_NAME}_UIS_H}
  ${${PROJECT_NAME}_MOCS}
)

add_executable(test_asad src/accept_pose.cpp)

target_link_libraries(${PROJECT_NAME}
                      ${catkin_LIBRARIES}
                      Qt5::Widgets
                      Qt5::Gui
                      )

include_directories(ui)