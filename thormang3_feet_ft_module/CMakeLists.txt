cmake_minimum_required(VERSION 2.8.3)
project(thormang3_feet_ft_module)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  cmake_modules
  message_generation
  
  dynamixel_sdk
  robotis_math
  robotis_device
  robotis_framework_common
  thormang3_kinematics_dynamics
  
  thormang3_feet_ft_module_msgs
  ati_ft_sensor
)

find_package(Eigen REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES thormang3_feet_ft_module
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_library(thormang3_feet_ft_module
  src/feet_force_torque_sensor_module.cpp
)

add_dependencies(thormang3_feet_ft_module ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(thormang3_feet_ft_module
  yaml-cpp
  ${catkin_LIBRARIES}
)

