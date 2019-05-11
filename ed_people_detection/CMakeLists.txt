cmake_minimum_required(VERSION 2.8.3)
project(ed_people_detection)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  ed
  ed_people_detection_msgs
  people_detection_3d_msgs
  roscpp
)

catkin_package(
  CATKIN_DEPENDS
    ed ed_people_detection_msgs roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(ed_people_detection_plugin
  src/people_detection_plugin.cpp
)

target_link_libraries(ed_people_detection_plugin ${catkin_LIBRARIES})
add_dependencies(ed_people_detection_plugin ${catkin_EXPORTED_TARGETS})
