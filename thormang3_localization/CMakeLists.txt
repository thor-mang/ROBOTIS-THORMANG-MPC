cmake_minimum_required(VERSION 2.8.3)
project(thormang3_localization)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  cmake_modules
  tf
  robotis_math
)

find_package(Eigen REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS 
  roscpp 
  std_msgs
  sensor_msgs
  geometry_msgs
  tf
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_executable(thormang3_localization_node 
  src/main.cpp
  src/thormang3_localization.cpp
)

target_link_libraries(thormang3_localization_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

