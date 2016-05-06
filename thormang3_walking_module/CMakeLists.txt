cmake_minimum_required(VERSION 2.8.3)
project(thormang3_walking_module)


find_package(catkin REQUIRED COMPONENTS
  dynamixel_sdk
  robotis_device
  robotis_controller_msgs
  robotis_framework_common
  robotis_math
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  cmake_modules
  eigen_conversions
  thormang3_kinematics_dynamics
  thormang3_walking_module_msgs
)

find_package(Eigen REQUIRED)

catkin_package(
  INCLUDE_DIRS include
#  CATKIN_DEPENDS walking_module_msgs
  LIBRARIES thormang3_walking_module
#  CATKIN_DEPENDS robotis_device roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(thormang3_walking_module
  src/LinearAlgebra.cpp
  src/WaklingModule.cpp
  src/PreviewControlWalking.cpp
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies(thormang3_walking_module ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(thormang3_walking_module
  yaml-cpp
  ${catkin_LIBRARIES}
)

