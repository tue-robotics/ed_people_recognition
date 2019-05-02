cmake_minimum_required(VERSION 2.8.3)
project(ed_people_detector_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  ed_msgs
  message_generation
  sensor_msgs
  std_msgs
)

add_service_files(
  FILES
    EdDetectPeople.srv
)

generate_messages(
  DEPENDENCIES
    ed_msgs
    sensor_msgs
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    ed_msgs message_runtime sensor_msgs std_msgs
)
