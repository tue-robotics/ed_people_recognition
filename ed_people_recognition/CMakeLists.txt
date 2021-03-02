cmake_minimum_required(VERSION 3.0.2)
project(ed_people_recognition)

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
