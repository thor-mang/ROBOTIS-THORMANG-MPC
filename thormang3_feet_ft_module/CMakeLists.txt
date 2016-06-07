cmake_minimum_required(VERSION 2.8.3)
project(thormang3_feet_ft_module)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
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
  LIBRARIES thormang3_feet_ft_module
#  CATKIN_DEPENDS roscpp
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

# Declare a C++ library
add_library(thormang3_feet_ft_module
  src/thormang3_feet_force_torque_sensor_module.cpp
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies(thormang3_feet_ft_module ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


## Specify libraries to link a library or executable target against
target_link_libraries(thormang3_feet_ft_module
  yaml-cpp
  ${catkin_LIBRARIES}
)

