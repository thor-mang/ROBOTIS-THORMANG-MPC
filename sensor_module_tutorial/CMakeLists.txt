cmake_minimum_required(VERSION 2.8.3)
project(sensor_module_tutorial)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  robotis_device
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES sensor_module_tutorial
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(sensor_module_tutorial
  src/${PROJECT_NAME}/sensor_module_tutorial.cpp
)

