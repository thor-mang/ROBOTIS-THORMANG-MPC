#cmake_minimum_required(VERSION 2.8.3)
project(thormang3_walking_module_msgs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  message_generation
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  PoseXYZRPY.msg
  PoseZRPY.msg
  StepTimeData.msg
  StepPositionData.msg
  StepData.msg
  StepDataArray.msg
  BalanceParam.msg
  RobotPose.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  IsRunning.srv
  SetBalanceParam.srv
  GetReferenceStepData.srv
  AddStepDataArray.srv
  WalkingStart.srv
  RemoveExistingStepData.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
