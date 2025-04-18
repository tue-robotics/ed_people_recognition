cmake_minimum_required(VERSION 3.5)
project(ed_people_recognition)

add_compile_options(-Wall -Werror=all)
add_compile_options(-Wextra -Werror=extra)

find_package(catkin REQUIRED COMPONENTS
  ed
  ${PROJECT_NAME}_msgs
  people_recognition_msgs
  roscpp
)

catkin_package(
  CATKIN_DEPENDS ed ${PROJECT_NAME}_msgs people_recognition_msgs roscpp
)

include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_plugin
  src/people_recognition_plugin.cpp
)

target_link_libraries(${PROJECT_NAME}_plugin ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_plugin ${catkin_EXPORTED_TARGETS})

# ------------------------------------------------------------------------------------------------
#                                          Install
# ------------------------------------------------------------------------------------------------

install(
  FILES plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  TARGETS ${PROJECT_NAME}_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

# ------------------------------------------------------------------------------------------------
#                                           TESTS
# ------------------------------------------------------------------------------------------------

if (CATKIN_ENABLE_TESTING)
  find_package(catkin_lint_cmake REQUIRED)
  catkin_add_catkin_lint_test("-W2 --ignore HEADER_OUTSIDE_PACKAGE_INCLUDE_PATH")
endif()
