cmake_minimum_required(VERSION 2.8.3)
project(thormang3_action_module_msgs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
)


## Generate services in the 'srv' folder
add_service_files(
  FILES
  IsRunning.srv
)


## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)



catkin_package(

)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


