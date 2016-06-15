cmake_minimum_required(VERSION 2.8.3)
project(thormang3_base_module)

find_package(catkin REQUIRED COMPONENTS
  dynamixel_sdk
  robotis_device
  robotis_math
  robotis_controller_msgs
  robotis_framework_common
  roscpp
  std_msgs
  geometry_msgs
  cmake_modules
  thormang3_kinematics_dynamics
  #message_generation
)

find_package(Eigen REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

#generate_messages(
#  DEPENDENCIES
#  std_msgs
#  sensor_msgs
#  geometry_msgs
#)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES thormang3_base_module
#  CATKIN_DEPENDS robotis_device roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(thormang3_base_module
  src/base_module.cpp
  src/base_module_state.cpp
  #src/JointState.cpp
  #src/Transformation.cpp
  #src/Trajectory.cpp
)

add_dependencies(thormang3_base_module ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(thormang3_base_module
  yaml-cpp
  ${catkin_LIBRARIES}
)
