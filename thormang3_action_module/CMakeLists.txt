cmake_minimum_required(VERSION 2.8.3)
project(thormang3_action_module)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  dynamixel_sdk
  robotis_device
  robotis_controller_msgs
  robotis_framework_common
  thormang3_action_module_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES thormang3_action_module
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(thormang3_action_module
  src/action_module.cpp
)

add_dependencies(thormang3_action_module ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


