cmake_minimum_required(VERSION 3.0.2)
project(ed_people_recognition_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  sensor_msgs
  std_msgs
)

add_service_files(
  FILES
    EdRecognizePeople.srv
)

generate_messages(
  DEPENDENCIES
    sensor_msgs
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    message_runtime sensor_msgs std_msgs
)
