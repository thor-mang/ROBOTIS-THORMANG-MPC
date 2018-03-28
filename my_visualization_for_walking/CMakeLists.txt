cmake_minimum_required(VERSION 2.8.3)
project(my_visualization_for_walking)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  thormang3_walking_module_msgs
  robotis_controller_msgs
  thormang3_foot_step_generator
  robotis_math
)


catkin_package(
#  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs
)

set(HEADERS
  include/my_visualization_for_walking/data_storage_tool.h
)

set(SOURCES
  src/data_storage_tool.cpp
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  ${HEADERS}
  ${SOURCES}
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(data_storage_tool_node src/data_storage_tool_node.cpp)

target_link_libraries(data_storage_tool_node ${PROJECT_NAME})

