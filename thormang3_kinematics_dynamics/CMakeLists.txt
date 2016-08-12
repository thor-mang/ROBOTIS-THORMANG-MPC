cmake_minimum_required(VERSION 2.8.3)
project(thormang3_kinematics_dynamics)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  robotis_math
  cmake_modules
)

find_package(Eigen REQUIRED)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES thormang3_kinematics_dynamics
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_library(thormang3_kinematics_dynamics
  src/link_data.cpp
  src/kinematics_dynamics.cpp
)

add_dependencies(thormang3_kinematics_dynamics ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(thormang3_kinematics_dynamics
  ${catkin_LIBRARIES}
)

