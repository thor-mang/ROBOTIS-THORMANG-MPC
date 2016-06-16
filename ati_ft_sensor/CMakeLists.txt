cmake_minimum_required(VERSION 2.8.3)
project(ati_ft_sensor)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  cmake_modules
  robotis_math
)

find_package(Eigen REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ati_ft_sensor
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_library(ati_ft_sensor
  src/ati_force_torque_sensor_twe.cpp
)

add_dependencies(ati_ft_sensor ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(ati_ft_sensor
  yaml-cpp
  ${catkin_LIBRARIES}
)

