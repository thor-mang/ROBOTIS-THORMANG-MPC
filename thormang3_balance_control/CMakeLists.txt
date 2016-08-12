cmake_minimum_required(VERSION 2.8.3)
project(thormang3_balance_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cmake_modules
  robotis_math
)

find_package(Eigen REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES thormang3_balance_control
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_library(thormang3_balance_control
  src/thormang3_balance_control.cpp
)

add_dependencies(thormang3_balance_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(thormang3_balance_control
  ${catkin_LIBRARIES}
)
