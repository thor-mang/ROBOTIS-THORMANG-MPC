cmake_minimum_required(VERSION 2.8.3)
project(thormang3_manager)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  dynamixel_sdk
  robotis_framework_common
  robotis_device
  robotis_controller
  robotis_controller_msgs
  robotis_math
  
  cmake_modules
  ati_ft_sensor
  thormang3_kinematics_dynamics
  
  thormang3_feet_ft_module
  thormang3_balance_control
  
  thormang3_action_module
  thormang3_head_control_module
  thormang3_manipulation_module
  thormang3_walking_module
  thormang3_base_module
  thormang3_wholebody_module
)

find_package(Eigen REQUIRED)

catkin_package(
#   INCLUDE_DIRS include
#  LIBRARIES thormang3_manager
#  CATKIN_DEPENDS robotis_controller roscpp
#  DEPENDS system_lib
)

include_directories(
  include
  ${Eigen_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(thormang3_manager src/thormang3_manager.cpp)

target_link_libraries(thormang3_manager
  ${catkin_LIBRARIES}
)
