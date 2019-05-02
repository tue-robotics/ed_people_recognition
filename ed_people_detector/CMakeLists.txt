cmake_minimum_required(VERSION 2.8.3)
project(ed_people_detector)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  ed
  roscpp
)

catkin_package(
  CATKIN_DEPENDS
    ed roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(people_detector_plugin
  src/people_detector_plugin.cpp
)

target_link_libraries(people_detector_plugin ${catkin_LIBRARIES})
