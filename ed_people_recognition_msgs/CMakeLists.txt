cmake_minimum_required(VERSION 3.5)
project(ed_people_recognition_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  sensor_msgs
)

add_service_files(
  FILES
    EdRecognizePeople.srv
)

generate_messages(
  DEPENDENCIES
    sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS
    message_runtime sensor_msgs
)
