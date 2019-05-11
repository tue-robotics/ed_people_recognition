cmake_minimum_required(VERSION 2.8.3)
project(ed_people_recognition)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  ed
  ed_people_recognition_msgs
  people_recognition_msgs
  roscpp
)

catkin_package(
  CATKIN_DEPENDS
    ed ed_people_recognition_msgs roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(ed_people_recognition_plugin
  src/people_recognition_plugin.cpp
)

target_link_libraries(ed_people_recognition_plugin ${catkin_LIBRARIES})
add_dependencies(ed_people_recognition_plugin ${catkin_EXPORTED_TARGETS})
