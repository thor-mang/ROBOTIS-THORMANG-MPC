cmake_minimum_required(VERSION 2.8.3)
project(thormang3_wrist_ft_module)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  cmake_modules
  robotis_device
  robotis_framework_common
)

find_package(Eigen REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES thormang3_wrist_ft_module
#  CATKIN_DEPENDS geomtry_msgs robotis_device roscpp
#  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)


add_library(thormang3_wrist_ft_module
  src/ATIForceTorqueSensor.cpp
  src/ThorMang3WristForceTorqueSensorModule.cpp
)


add_dependencies(thormang3_wrist_ft_module ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(thormang3_wrist_ft_module
   yaml-cpp
   ${catkin_LIBRARIES}
)
