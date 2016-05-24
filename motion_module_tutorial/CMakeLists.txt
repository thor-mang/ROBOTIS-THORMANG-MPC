cmake_minimum_required(VERSION 2.8.3)
project(motion_module_tutorial)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  robotis_device
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES motion_module_tutorial
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(motion_module_tutorial
  src/${PROJECT_NAME}/motion_module_tutorial.cpp
)
