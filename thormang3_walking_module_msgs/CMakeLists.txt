
project(thormang3_walking_module_msgs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  message_generation
)

add_message_files(
  FILES
  PoseXYZRPY.msg
  PoseZRPY.msg
  StepTimeData.msg
  StepPositionData.msg
  StepData.msg
  BalanceParam.msg
  RobotPose.msg
)

add_service_files(
  FILES
  IsRunning.srv
  SetBalanceParam.srv
  GetReferenceStepData.srv
  AddStepDataArray.srv
  StartWalking.srv
  RemoveExistingStepData.srv
)

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
