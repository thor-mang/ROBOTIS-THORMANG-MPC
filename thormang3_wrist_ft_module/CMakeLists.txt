cmake_minimum_required(VERSION 2.8.3)
project(thormang3_wrist_ft_module)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  cmake_modules
  robotis_math
  robotis_framework_common
  robotis_controller_msgs
  thormang3_kinematics_dynamics
  thormang3_feet_ft_module_msgs
  ati_ft_sensor
)

find_package(Eigen REQUIRED)



###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES thormang3_wrist_ft_module
  CATKIN_DEPENDS roscpp robotis_math robotis_framework_common robotis_controller_msgs thormang3_kinematics_dynamics thormang3_feet_ft_module_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

set(HEADERS
  include/${PROJECT_NAME}/wrist_force_torque_sensor_module.h
)

set(SOURCES
  src/wrist_force_torque_sensor_module.cpp
)


## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

# Declare a C++ library
add_library(thormang3_wrist_ft_module ${SOURCES} ${HEADERS})
set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

