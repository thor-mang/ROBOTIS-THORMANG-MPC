cmake_minimum_required(VERSION 2.8.3)
project(thormang3_walking_module)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  cmake_modules
  eigen_conversions
  
  dynamixel_sdk
  robotis_device
  robotis_controller_msgs
  robotis_framework_common
  robotis_math
  thormang3_kinematics_dynamics
  thormang3_walking_module_msgs
  thormang3_balance_control
)

find_package(Eigen REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES thormang3_walking_module
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(thormang3_walking_module
  src/thormang3_online_walking.cpp
  src/thormang3_wakling_module.cpp
)

add_dependencies(thormang3_walking_module ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(thormang3_walking_module
  yaml-cpp
  ${catkin_LIBRARIES}
)

