cmake_minimum_required(VERSION 2.8.3)
project(thormang3_head_control_module)

find_package(catkin REQUIRED COMPONENTS
  dynamixel_sdk
  robotis_device
  robotis_math
  robotis_controller_msgs
  robotis_framework_common
  roscpp
  std_msgs
  cmake_modules
)

find_package(Eigen REQUIRED)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES thormang3_head_control_module
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_library(thormang3_head_control_module
  src/head_control_module.cpp
)

add_dependencies(thormang3_head_control_module ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(thormang3_head_control_module
  ${catkin_LIBRARIES}
)


