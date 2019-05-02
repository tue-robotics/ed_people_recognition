cmake_minimum_required(VERSION 2.8.3)
project(ed_people_detector_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  message_runtime
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    message_runtime std_msgs
)
