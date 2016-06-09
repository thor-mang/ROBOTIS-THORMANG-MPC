/*
 * TestMotionModule.cpp
 *
 *  Created on: 2016. 1. 25.
 *      Author: zerom
 */

#include <stdio.h>
#include <eigen_conversions/eigen_msg.h>

#include "thormang3_walking_module/walking_module.h"

using namespace thormang3;

class WalkingStatusMSG {
public:
  static const std::string FAILED_TO_ADD_STEP_DATA_MSG;
  static const std::string BALANCE_PARAM_SETTING_START_MSG;
  static const std::string BALANCE_PARAM_SETTING_FINISH_MSG;
  static const std::string WALKING_MODULE_IS_ENABLED_MSG;
  static const std::string WALKING_MODULE_IS_DISABLED_MSG;
  static const std::string WALKING_START_MSG;
  static const std::string WALKING_FINISH_MSG;
};

const std::string WalkingStatusMSG::FAILED_TO_ADD_STEP_DATA_MSG = "Failed_to_add_Step_Data";
const std::string WalkingStatusMSG::BALANCE_PARAM_SETTING_START_MSG = "Balance_Param_Setting_Started";
const std::string WalkingStatusMSG::BALANCE_PARAM_SETTING_FINISH_MSG = "Balance_Param_Setting_Finished";
const std::string WalkingStatusMSG::WALKING_MODULE_IS_ENABLED_MSG = "Walking_Module_is_enabled";
const std::string WalkingStatusMSG::WALKING_MODULE_IS_DISABLED_MSG = "Walking_Module_is_enabled";
const std::string WalkingStatusMSG::WALKING_START_MSG = "Walking_Started";
const std::string WalkingStatusMSG::WALKING_FINISH_MSG = "Walking_Finished";


WalkingMotionModule::WalkingMotionModule()
: control_cycle_msec_(8)
{
  enable_          = false;
  module_name_     = "walking_module";
  control_mode_    = robotis_framework::PositionControl;
  result_["r_leg_hip_y"] = new robotis_framework::DynamixelState();
  result_["r_leg_hip_r"] = new robotis_framework::DynamixelState();
  result_["r_leg_hip_p"] = new robotis_framework::DynamixelState();
  result_["r_leg_kn_p" ] = new robotis_framework::DynamixelState();
  result_["r_leg_an_p" ] = new robotis_framework::DynamixelState();
  result_["r_leg_an_r" ] = new robotis_framework::DynamixelState();

  result_["l_leg_hip_y"] = new robotis_framework::DynamixelState();
  result_["l_leg_hip_r"] = new robotis_framework::DynamixelState();
  result_["l_leg_hip_p"] = new robotis_framework::DynamixelState();
  result_["l_leg_kn_p" ] = new robotis_framework::DynamixelState();
  result_["l_leg_an_p" ] = new robotis_framework::DynamixelState();
  result_["l_leg_an_r" ] = new robotis_framework::DynamixelState();

  previous_running_    = present_running    = false;

  gyro_roll_ =  gyro_pitch_ = 0;
  orientation_roll_ = orientation_pitch_ = 0;
  r_foot_fx_N_  = r_foot_fy_N_  = r_foot_fz_N_  = 0;
  r_foot_Tx_Nm_ = r_foot_Ty_Nm_ = r_foot_Tz_Nm_ = 0;
  l_foot_fx_N_  = l_foot_fy_N_  = l_foot_fz_N_  = 0;
  l_foot_Tx_Nm_ = l_foot_Ty_Nm_ = l_foot_Tz_Nm_ = 0;


  r_foot_ft_publish_checker_ = -1;
  l_foot_ft_publish_checker_ = -1;

  desired_matrix_g_to_cob_   = Eigen::MatrixXd::Identity(4,4);
  desired_matrix_g_to_rfoot_ = Eigen::MatrixXd::Identity(4,4);
  desired_matrix_g_to_lfoot_ = Eigen::MatrixXd::Identity(4,4);


  balance_update_with_loop_ = false;
  balance_update_duration_ = 1.0;
  balance_update_sys_time_ = 1.0;
  balance_update_polynomial_coeff_.resize(6, 1);

  double tf = balance_update_duration_;
  Eigen::MatrixXd A(6,6), B(6, 1);
  A <<  0.0,     0.0,     0.0,    0.0,    0.0, 1.0,
      0.0,   0.0,    0.0,    0.0,       1.0, 0.0,
      0.0,   0.0,    0.0,    2.0,       0.0, 0.0,
      tf*tf*tf*tf*tf,     tf*tf*tf*tf,      tf*tf*tf,        tf*tf,     tf, 1.0,
      5.0*tf*tf*tf*tf,    4.0*tf*tf*tf,    3.0*tf*tf,    2.0*tf,        1.0, 0.0,
      20.0*tf*tf*tf,      12.0*tf*tf,       6.0*tf,        2.0,        0.0, 0.0;

  B << 0, 0, 0, 1.0, 0, 0;

  balance_update_polynomial_coeff_ = A.inverse() * B;

  rot_x_pi_3d_.resize(3,3);
  rot_x_pi_3d_ << 1,  0,  0,
                  0, -1,  0,
                  0,  0, -1;
  rot_z_pi_3d_.resize(3,3);
  rot_z_pi_3d_ << -1,  0, 0,
                   0, -1, 0,
                   0,  0, 1;
}

WalkingMotionModule::~WalkingMotionModule()
{
  queue_thread_.join();
}

void WalkingMotionModule::initialize(const int control_cycle_msec, robotis_framework::Robot *robot)
{
  queue_thread_ = boost::thread(boost::bind(&WalkingMotionModule::queueThread, this));
  control_cycle_msec_ = control_cycle_msec;

  RobotisOnlineWalking *online_walking = RobotisOnlineWalking::getInstance();
  online_walking->SetInitialPose(0, -0.093, -0.63, 0, 0, 0,
                                 0,  0.093, -0.63, 0, 0, 0,
                                 0,      0,     0, 0, 0, 0);

  online_walking->SetFTScaleFactor(1.0, 1.0);
  online_walking->SetInitForceTorque(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);

  online_walking->WALK_STABILIZER_GAIN_RATIO = 0;
  online_walking->IMU_GYRO_GAIN_RATIO = 0.0;//7.31*0.01;
  online_walking->FORCE_MOMENT_DISTRIBUTION_RATIO = 0.0;

  online_walking->BALANCE_X_GAIN     = +1.0*20.30*0.625*(online_walking->FORCE_MOMENT_DISTRIBUTION_RATIO)*online_walking->WALK_STABILIZER_GAIN_RATIO;
  online_walking->BALANCE_Y_GAIN     = -1.0*20.30*0.75*(online_walking->FORCE_MOMENT_DISTRIBUTION_RATIO)*online_walking->WALK_STABILIZER_GAIN_RATIO;
  online_walking->BALANCE_Z_GAIN     =    0.0*2.0*(online_walking->FORCE_MOMENT_DISTRIBUTION_RATIO)*online_walking->WALK_STABILIZER_GAIN_RATIO;

  online_walking->BALANCE_PITCH_GAIN = -1.0*0.10*0.5 *(1.0 - online_walking->FORCE_MOMENT_DISTRIBUTION_RATIO)*online_walking->WALK_STABILIZER_GAIN_RATIO;
  online_walking->BALANCE_ROLL_GAIN  = -1.0*0.10*0.75*(1.0 - online_walking->FORCE_MOMENT_DISTRIBUTION_RATIO)*online_walking->WALK_STABILIZER_GAIN_RATIO;

  online_walking->HIP_ROLL_FEEDFORWARD_ANGLE_RAD = 0.0*M_PI/180.0;
  ////////// Damping Controller
  online_walking->BALANCE_ANKLE_ROLL_GAIN_BY_IMU = 0;
  online_walking->BALANCE_ANKLE_PITCH_GAIN_BY_IMU = 0;

  online_walking->BALANCE_X_GAIN_BY_FT = 0;
  online_walking->BALANCE_Y_GAIN_BY_FT = 0;
  online_walking->BALANCE_Z_GAIN_BY_FT = 0;


  online_walking->BALANCE_RIGHT_ROLL_GAIN_BY_FT = 0;
  online_walking->BALANCE_RIGHT_PITCH_GAIN_BY_FT = 0;

  online_walking->BALANCE_LEFT_ROLL_GAIN_BY_FT = 0;
  online_walking->BALANCE_LEFT_PITCH_GAIN_BY_FT = 0;

  //the below gain must be zero.
  online_walking->BALANCE_HIP_ROLL_GAIN = 0;
  online_walking->BALANCE_HIP_PITCH_GAIN = 0;
  online_walking->AXIS_CONTROLLER_GAIN = 0;
  online_walking->LANDING_CONTROLLER_GAIN = 0;

  //time constant
  online_walking->BALANCE_ANKLE_ROLL_TIME_CONSTANT_BY_IMU    = 0.2;
  online_walking->BALANCE_ANKLE_PITCH_TIME_CONSTANT_BY_IMU    = 0.2;

  online_walking->BALANCE_X_TIME_CONSTANT                    = 0.1;
  online_walking->BALANCE_Y_TIME_CONSTANT                    = 0.1;
  online_walking->BALANCE_Z_TIME_CONSTANT                    = 0.1;
  online_walking->BALANCE_RIGHT_ANKLE_ROLL_TIME_CONSTANT    = 0.1;
  online_walking->BALANCE_RIGHT_ANKLE_PITCH_TIME_CONSTANT    = 0.1;
  online_walking->BALANCE_LEFT_ANKLE_ROLL_TIME_CONSTANT         = 0.1;
  online_walking->BALANCE_LEFT_ANKLE_PITCH_TIME_CONSTANT    = 0.1;


  online_walking->COB_X_MANUAL_ADJUSTMENT_M    = -10.0*0.001;

  online_walking->initialize();

  online_walking->BALANCE_ENABLE = true;

  publish_mutex_.lock();
  desired_matrix_g_to_cob_   = online_walking->matGtoCOB;
  desired_matrix_g_to_rfoot_ = online_walking->matGtoRF;
  desired_matrix_g_to_lfoot_ = online_walking->matGtoLF;
  publish_mutex_.unlock();

  result_["r_leg_hip_y"]->goal_position_ = online_walking->m_OutAngleRad[0];
  result_["r_leg_hip_r"]->goal_position_ = online_walking->m_OutAngleRad[1];
  result_["r_leg_hip_p"]->goal_position_ = online_walking->m_OutAngleRad[2];
  result_["r_leg_kn_p"]->goal_position_  = online_walking->m_OutAngleRad[3];
  result_["r_leg_an_p"]->goal_position_  = online_walking->m_OutAngleRad[4];
  result_["r_leg_an_r"]->goal_position_  = online_walking->m_OutAngleRad[5];

  result_["l_leg_hip_y"]->goal_position_ = online_walking->m_OutAngleRad[6];
  result_["l_leg_hip_r"]->goal_position_ = online_walking->m_OutAngleRad[7];
  result_["l_leg_hip_p"]->goal_position_ = online_walking->m_OutAngleRad[8];
  result_["l_leg_kn_p" ]->goal_position_ = online_walking->m_OutAngleRad[9];
  result_["l_leg_an_p" ]->goal_position_ = online_walking->m_OutAngleRad[10];
  result_["l_leg_an_r" ]->goal_position_ = online_walking->m_OutAngleRad[11];

  online_walking->start();
  online_walking->process();

  previous_running_ = isRunning();
}

void    WalkingMotionModule::queueThread()
{
  ros::NodeHandle     ros_node;
  ros::CallbackQueue  callback_queue;

  ros_node.setCallbackQueue(&callback_queue);

  /* publish topics */
  robot_pose_pub_ = ros_node.advertise<thormang3_walking_module_msgs::RobotPose>("/robotis/walking/robot_pose", 1);
  //status_msg_pub_    = _ros_node.advertise<std_msgs::String>("/robotis/walking/status_message", 1);
  status_msg_pub_ = ros_node.advertise<robotis_controller_msgs::StatusMsg>("robotis/status", 1);


  /* ROS Service Callback Functions */
  ros::ServiceServer get_ref_step_data_server  = ros_node.advertiseService("/robotis/walking/get_reference_step_data",   &WalkingMotionModule::getReferenceStepDataServiceCallback,   this);
  ros::ServiceServer add_step_data_array_sever = ros_node.advertiseService("/robotis/walking/add_step_data",             &WalkingMotionModule::addStepDataServiceCallback,            this);
  ros::ServiceServer walking_start_server      = ros_node.advertiseService("/robotis/walking/walking_start",             &WalkingMotionModule::startWalkingServiceCallback,           this);
  ros::ServiceServer is_running_server         = ros_node.advertiseService("/robotis/walking/is_running",                &WalkingMotionModule::IsRunningServiceCallback,              this);
  ros::ServiceServer set_balance_param_server  = ros_node.advertiseService("/robotis/walking/set_balance_param",         &WalkingMotionModule::setBalanceParamServiceCallback,        this);
  ros::ServiceServer remove_existing_step_data = ros_node.advertiseService("/robotis/walking/remove_existing_step_data", &WalkingMotionModule::removeExistingStepDataServiceCallback, this);

  /* sensor topic subscribe */
  ros::Subscriber imu_data_sub    = ros_node.subscribe("/robotis/sensor/imu/imu",    3, &WalkingMotionModule::imuDataOutputCallback,        this);

  while(ros_node.ok())
  {
    callback_queue.callAvailable();
    usleep(100);
  }
}

void WalkingMotionModule::publishRobotPose(void)
{
  publish_mutex_.lock();
  robot_pose_msg_.global_to_center_of_body.position.x = desired_matrix_g_to_cob_.coeff(0, 3);
  robot_pose_msg_.global_to_center_of_body.position.y = desired_matrix_g_to_cob_.coeff(1, 3);
  robot_pose_msg_.global_to_center_of_body.position.z = desired_matrix_g_to_cob_.coeff(2, 3);
  Eigen::Quaterniond quaterniond_g_to_cob(desired_matrix_g_to_cob_.block<3, 3>(0, 0));


  robot_pose_msg_.global_to_right_foot.position.x = desired_matrix_g_to_rfoot_.coeff(0, 3);
  robot_pose_msg_.global_to_right_foot.position.y = desired_matrix_g_to_rfoot_.coeff(1, 3);
  robot_pose_msg_.global_to_right_foot.position.z = desired_matrix_g_to_rfoot_.coeff(2, 3);
  Eigen::Quaterniond quaterniond_g_to_rf(desired_matrix_g_to_rfoot_.block<3, 3>(0, 0));

  robot_pose_msg_.global_to_left_foot.position.x = desired_matrix_g_to_lfoot_.coeff(0, 3);
  robot_pose_msg_.global_to_left_foot.position.y = desired_matrix_g_to_lfoot_.coeff(1, 3);
  robot_pose_msg_.global_to_left_foot.position.z = desired_matrix_g_to_lfoot_.coeff(2, 3);
  Eigen::Quaterniond quaterniond_g_to_lf(desired_matrix_g_to_lfoot_.block<3, 3>(0, 0));
  publish_mutex_.unlock();

  tf::quaternionEigenToMsg(quaterniond_g_to_cob, robot_pose_msg_.global_to_center_of_body.orientation);
  tf::quaternionEigenToMsg(quaterniond_g_to_rf,  robot_pose_msg_.global_to_right_foot.orientation);
  tf::quaternionEigenToMsg(quaterniond_g_to_lf,  robot_pose_msg_.global_to_left_foot.orientation);

  robot_pose_pub_.publish(robot_pose_msg_);

}

void WalkingMotionModule::publishStatusMsg(unsigned int type, std::string msg)
{
  robotis_controller_msgs::StatusMsg status_msg;
  status_msg.header.stamp = ros::Time::now();
  status_msg.type = type;
  status_msg.module_name = "Walking";
  status_msg.status_msg = msg;

  status_msg_pub_.publish(status_msg);
}

int WalkingMotionModule::convertStepDataMsgToStepData(thormang3_walking_module_msgs::StepData& src, StepData& des)
{
  int copy_result = thormang3_walking_module_msgs::AddStepDataArray::Response::NO_ERROR;
  des.TimeData.bWalkingState           = src.time_data.walking_state;
  des.TimeData.dAbsStepTime            = src.time_data.abs_step_time;
  des.TimeData.dDSPratio               = src.time_data.dsp_ratio;
  des.TimeData.sigmoid_ratio_x         = 1.0;//msg->step_data[i].TimeData.front_pause_ratio_x;
  des.TimeData.sigmoid_ratio_y         = 1.0;//msg->step_data[i].TimeData.front_pause_ratio_y;
  des.TimeData.sigmoid_ratio_z         = 1.0;//msg->step_data[i].TimeData.front_pause_ratio_z;
  des.TimeData.sigmoid_ratio_roll      = 1.0;//msg->step_data[i].TimeData.front_pause_ratio_roll;
  des.TimeData.sigmoid_ratio_pitch     = 1.0;//msg->step_data[i].TimeData.front_pause_ratio_pitch;
  des.TimeData.sigmoid_ratio_yaw       = 1.0;//msg->step_data[i].TimeData.front_pause_ratio_yaw;

  des.TimeData.sigmoid_distortion_x     = 1.0;//msg->step_data[i].TimeData.back_pause_ratio_x;
  des.TimeData.sigmoid_distortion_y     = 1.0;//msg->step_data[i].TimeData.back_pause_ratio_y;
  des.TimeData.sigmoid_distortion_z     = 1.0;//msg->step_data[i].TimeData.back_pause_ratio_z;
  des.TimeData.sigmoid_distortion_roll  = 1.0;//msg->step_data[i].TimeData.back_pause_ratio_roll;
  des.TimeData.sigmoid_distortion_pitch = 1.0;//msg->step_data[i].TimeData.back_pause_ratio_pitch;
  des.TimeData.sigmoid_distortion_yaw   = 1.0;//msg->step_data[i].TimeData.back_pause_ratio_yaw;


  des.PositionData.bMovingFoot        = src.position_data.moving_foot;
  des.PositionData.dShoulderSwingGain = 0;
  des.PositionData.dElbowSwingGain    = 0;
  des.PositionData.dFootHeight        = src.position_data.foot_z_swap;
  des.PositionData.dWaistPitchAngle   = 0;
  des.PositionData.dWaistYawAngle     = src.position_data.torso_yaw_angle_rad;
  des.PositionData.dZ_Swap_Amplitude  = src.position_data.body_z_swap;

  des.PositionData.stBodyPosition.z          = src.position_data.body_pose.z;
  des.PositionData.stBodyPosition.roll       = src.position_data.body_pose.roll;
  des.PositionData.stBodyPosition.pitch      = src.position_data.body_pose.pitch;
  des.PositionData.stBodyPosition.yaw        = src.position_data.body_pose.yaw;
  des.PositionData.stRightFootPosition.x     = src.position_data.right_foot_pose.x;
  des.PositionData.stRightFootPosition.y     = src.position_data.right_foot_pose.y;
  des.PositionData.stRightFootPosition.z     = src.position_data.right_foot_pose.z;
  des.PositionData.stRightFootPosition.roll  = src.position_data.right_foot_pose.roll;
  des.PositionData.stRightFootPosition.pitch = src.position_data.right_foot_pose.pitch;
  des.PositionData.stRightFootPosition.yaw   = src.position_data.right_foot_pose.yaw;
  des.PositionData.stLeftFootPosition.x      = src.position_data.left_foot_pose.x;
  des.PositionData.stLeftFootPosition.y      = src.position_data.left_foot_pose.y;
  des.PositionData.stLeftFootPosition.z      = src.position_data.left_foot_pose.z;
  des.PositionData.stLeftFootPosition.roll   = src.position_data.left_foot_pose.roll;
  des.PositionData.stLeftFootPosition.pitch  = src.position_data.left_foot_pose.pitch;
  des.PositionData.stLeftFootPosition.yaw    = src.position_data.left_foot_pose.yaw;

  if((src.time_data.walking_state != thormang3_walking_module_msgs::StepTimeData::IN_WALKING_STARTING)
      && (src.time_data.walking_state != thormang3_walking_module_msgs::StepTimeData::IN_WALKING)
      && (src.time_data.walking_state != thormang3_walking_module_msgs::StepTimeData::IN_WALKING_ENDING) )
    copy_result |= thormang3_walking_module_msgs::AddStepDataArray::Response::PROBLEM_IN_TIME_DATA;

  if((src.time_data.start_time_delay_ratio_x < 0)
      || (src.time_data.start_time_delay_ratio_y < 0)
      || (src.time_data.start_time_delay_ratio_z < 0)
      || (src.time_data.start_time_delay_ratio_roll < 0)
      || (src.time_data.start_time_delay_ratio_pitch < 0)
      || (src.time_data.start_time_delay_ratio_yaw < 0) )
    copy_result |= thormang3_walking_module_msgs::AddStepDataArray::Response::PROBLEM_IN_TIME_DATA;

  if((src.time_data.finish_time_advance_ratio_x < 0)
      || (src.time_data.finish_time_advance_ratio_y < 0)
      || (src.time_data.finish_time_advance_ratio_z < 0)
      || (src.time_data.finish_time_advance_ratio_roll < 0)
      || (src.time_data.finish_time_advance_ratio_pitch < 0)
      || (src.time_data.finish_time_advance_ratio_yaw < 0) )
    copy_result |= thormang3_walking_module_msgs::AddStepDataArray::Response::PROBLEM_IN_TIME_DATA;

  if(((src.time_data.start_time_delay_ratio_x + src.time_data.finish_time_advance_ratio_x) > 1.0)
      || ((src.time_data.start_time_delay_ratio_y        + src.time_data.finish_time_advance_ratio_y        ) > 1.0)
      || ((src.time_data.start_time_delay_ratio_z        + src.time_data.finish_time_advance_ratio_z        ) > 1.0)
      || ((src.time_data.start_time_delay_ratio_roll    + src.time_data.finish_time_advance_ratio_roll    ) > 1.0)
      || ((src.time_data.start_time_delay_ratio_pitch    + src.time_data.finish_time_advance_ratio_pitch    ) > 1.0)
      || ((src.time_data.start_time_delay_ratio_yaw    + src.time_data.finish_time_advance_ratio_yaw    ) > 1.0) )
    copy_result |= thormang3_walking_module_msgs::AddStepDataArray::Response::PROBLEM_IN_TIME_DATA;

  if((src.position_data.moving_foot != thormang3_walking_module_msgs::StepPositionData::STANDING)
      && (src.position_data.moving_foot != thormang3_walking_module_msgs::StepPositionData::RIGHT_FOOT_SWING)
      && (src.position_data.moving_foot != thormang3_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING))
    copy_result |= thormang3_walking_module_msgs::AddStepDataArray::Response::PROBLEM_IN_POSITION_DATA;

  if(src.position_data.foot_z_swap < 0)
    copy_result |= thormang3_walking_module_msgs::AddStepDataArray::Response::PROBLEM_IN_POSITION_DATA;

  return copy_result;
}

int WalkingMotionModule::convertStepDataToStepDataMsg(StepData& src, thormang3_walking_module_msgs::StepData& des)
{
  des.time_data.walking_state   = src.TimeData.bWalkingState;
  des.time_data.abs_step_time   = src.TimeData.dAbsStepTime;
  des.time_data.dsp_ratio       = src.TimeData.dDSPratio;

  des.time_data.start_time_delay_ratio_x     = des.time_data.finish_time_advance_ratio_x     = 0;
  des.time_data.start_time_delay_ratio_y     = des.time_data.finish_time_advance_ratio_y     = 0;
  des.time_data.start_time_delay_ratio_z     = des.time_data.finish_time_advance_ratio_z     = 0;
  des.time_data.start_time_delay_ratio_roll  = des.time_data.finish_time_advance_ratio_roll  = 0;
  des.time_data.start_time_delay_ratio_pitch = des.time_data.finish_time_advance_ratio_pitch = 0;
  des.time_data.start_time_delay_ratio_yaw   = des.time_data.finish_time_advance_ratio_yaw   = 0;

  des.position_data.moving_foot         = src.PositionData.bMovingFoot;
  des.position_data.foot_z_swap         = src.PositionData.dFootHeight;
  des.position_data.torso_yaw_angle_rad = src.PositionData.dWaistYawAngle;
  des.position_data.body_z_swap         = src.PositionData.dZ_Swap_Amplitude;

  des.position_data.body_pose.z           = src.PositionData.stBodyPosition.z;
  des.position_data.body_pose.roll        = src.PositionData.stBodyPosition.roll;
  des.position_data.body_pose.pitch       = src.PositionData.stBodyPosition.pitch;
  des.position_data.body_pose.yaw         = src.PositionData.stBodyPosition.yaw;
  des.position_data.right_foot_pose.x     = src.PositionData.stRightFootPosition.x;
  des.position_data.right_foot_pose.y     = src.PositionData.stRightFootPosition.y;
  des.position_data.right_foot_pose.z     = src.PositionData.stRightFootPosition.z;
  des.position_data.right_foot_pose.roll  = src.PositionData.stRightFootPosition.roll;
  des.position_data.right_foot_pose.pitch = src.PositionData.stRightFootPosition.pitch;
  des.position_data.right_foot_pose.yaw   = src.PositionData.stRightFootPosition.yaw;
  des.position_data.left_foot_pose.x      = src.PositionData.stLeftFootPosition.x;
  des.position_data.left_foot_pose.y      = src.PositionData.stLeftFootPosition.y;
  des.position_data.left_foot_pose.z      = src.PositionData.stLeftFootPosition.z;
  des.position_data.left_foot_pose.roll   = src.PositionData.stLeftFootPosition.roll;
  des.position_data.left_foot_pose.pitch  = src.PositionData.stLeftFootPosition.pitch;
  des.position_data.left_foot_pose.yaw    = src.PositionData.stLeftFootPosition.yaw;

  return 0;
}

bool WalkingMotionModule::getReferenceStepDataServiceCallback(thormang3_walking_module_msgs::GetReferenceStepData::Request &req,
    thormang3_walking_module_msgs::GetReferenceStepData::Response &res)
{
  RobotisOnlineWalking *online_walking = RobotisOnlineWalking::getInstance();

  StepData refStepData;

  online_walking->GetReferenceStepDatafotAddition(&refStepData);

  convertStepDataToStepDataMsg(refStepData, res.reference_step_data);

  return true;
}

bool WalkingMotionModule::addStepDataServiceCallback(thormang3_walking_module_msgs::AddStepDataArray::Request &req,
    thormang3_walking_module_msgs::AddStepDataArray::Response &res)
{
  RobotisOnlineWalking *online_walking = RobotisOnlineWalking::getInstance();
  res.result = thormang3_walking_module_msgs::AddStepDataArray::Response::NO_ERROR;

  if(enable_ == false) {
    res.result |= thormang3_walking_module_msgs::AddStepDataArray::Response::NOT_ENABLED_WALKING_MODULE;
    std::string status_msg = WalkingStatusMSG::FAILED_TO_ADD_STEP_DATA_MSG;
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, status_msg);
    return true;
  }

  if(online_walking->isRunning() == true) {
    res.result |= thormang3_walking_module_msgs::AddStepDataArray::Response::ROBOT_IS_WALKING_NOW;
    std::string status_msg  = WalkingStatusMSG::FAILED_TO_ADD_STEP_DATA_MSG;
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, status_msg);
    return true;
  }

  StepData step_data, ref_step_data;
  std::vector<StepData> req_step_data_array;

  online_walking->GetReferenceStepDatafotAddition(&ref_step_data);

  for(int i = 0; i < req.step_data_array.size(); i++)
  {
    res.result |= convertStepDataMsgToStepData(req.step_data_array[i], step_data);

    if(step_data.TimeData.dAbsStepTime <= 0) {
      res.result |= thormang3_walking_module_msgs::AddStepDataArray::Response::PROBLEM_IN_TIME_DATA;
    }

    if(i != 0) {
      if(step_data.TimeData.dAbsStepTime <= req_step_data_array[req_step_data_array.size() - 1].TimeData.dAbsStepTime) {
        res.result |= thormang3_walking_module_msgs::AddStepDataArray::Response::PROBLEM_IN_TIME_DATA;
      }
    }
    else {
      if(step_data.TimeData.dAbsStepTime <= ref_step_data.TimeData.dAbsStepTime) {
        res.result |= thormang3_walking_module_msgs::AddStepDataArray::Response::PROBLEM_IN_TIME_DATA;
      }
    }

    if(res.result != thormang3_walking_module_msgs::AddStepDataArray::Response::NO_ERROR) {
      std::string status_msg = WalkingStatusMSG::FAILED_TO_ADD_STEP_DATA_MSG;
      publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, status_msg);
      return true;
    }

    req_step_data_array.push_back(step_data);
  }


  if(req.remove_existing_step_data == true) {
    int exist_num_of_step_data = online_walking->GetNumofRemainingUnreservedStepData();
    if(exist_num_of_step_data != 0)
      for(int remove_count  = 0; remove_count < exist_num_of_step_data; remove_count++)
        online_walking->EraseLastStepData();
  }

  for(unsigned int i = 0; i < req_step_data_array.size() ; i++)
    online_walking->AddStepData(req_step_data_array[i]);

  if( req.auto_start == true)    {
    online_walking->start();
  }

  return true;
}

bool WalkingMotionModule::startWalkingServiceCallback(thormang3_walking_module_msgs::StartWalking::Request &req,
    thormang3_walking_module_msgs::StartWalking::Response &res)
{
  RobotisOnlineWalking *prev_walking = RobotisOnlineWalking::getInstance();
  res.result = thormang3_walking_module_msgs::StartWalking::Response::NO_ERROR;

  if(enable_ == false) {
    res.result |= thormang3_walking_module_msgs::StartWalking::Response::NOT_ENABLED_WALKING_MODULE;
  }

  if(prev_walking->isRunning() == true){
    res.result |= thormang3_walking_module_msgs::StartWalking::Response::ROBOT_IS_WALKING_NOW;
  }

  if(prev_walking->GetNumofRemainingUnreservedStepData() == 0){
    res.result |= thormang3_walking_module_msgs::StartWalking::Response::NO_STEP_DATA;
  }

  if(res.result == thormang3_walking_module_msgs::StartWalking::Response::NO_ERROR) {
    prev_walking->start();
  }

  return true;
}

bool WalkingMotionModule::IsRunningServiceCallback(thormang3_walking_module_msgs::IsRunning::Request &req,
    thormang3_walking_module_msgs::IsRunning::Response &res)
{
  bool is_running = isRunning();
  res.is_running = is_running;

  return true;
}

bool WalkingMotionModule::isRunning()
{
  return RobotisOnlineWalking::getInstance()->isRunning();
}

bool WalkingMotionModule::setBalanceParamServiceCallback(thormang3_walking_module_msgs::SetBalanceParam::Request  &req,
    thormang3_walking_module_msgs::SetBalanceParam::Response &res)
{
  RobotisOnlineWalking *online_walking = RobotisOnlineWalking::getInstance();
  res.result = thormang3_walking_module_msgs::SetBalanceParam::Response::NO_ERROR;

  if( enable_ == false)
    res.result |= thormang3_walking_module_msgs::SetBalanceParam::Response::NOT_ENABLED_WALKING_MODULE;

  if( balance_update_with_loop_ == true)    {
    res.result |= thormang3_walking_module_msgs::SetBalanceParam::Response::PREV_REQUEST_IS_NOT_FINISHED;
  }

  if( ( req.balance_param.foot_roll_angle_time_constant  <= 0.0 )
      || ( req.balance_param.foot_pitch_angle_time_constant  <= 0.0 )
      || ( req.balance_param.foot_x_force_time_constant      <= 0.0 )
      || ( req.balance_param.foot_y_force_time_constant      <= 0.0 )
      || ( req.balance_param.foot_z_force_time_constant      <= 0.0 )
      || ( req.balance_param.foot_roll_torque_time_constant  <= 0.0 )
      || ( req.balance_param.foot_pitch_torque_time_constant <= 0.0 ) )
  {
    res.result |= thormang3_walking_module_msgs::SetBalanceParam::Response::TIME_CONST_IS_ZERO_OR_NEGATIVE;
  }

  if(res.result == thormang3_walking_module_msgs::SetBalanceParam::Response::NO_ERROR) {
    if( req.updating_duration < 0.0 )
    {
      // under 8ms apply immediately
      setBalanceParam(req.balance_param);
      std::string status_msg = WalkingStatusMSG::BALANCE_PARAM_SETTING_FINISH_MSG;
      publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, status_msg);
      return true;
    }
    else {
      balance_update_duration_ = req.updating_duration;
    }

    balance_update_sys_time_ = 0.0;
    balance_update_polynomial_coeff_.resize(6, 1);

    double tf = balance_update_duration_;
    Eigen::MatrixXd A(6,6), B(6, 1);
    A <<  0.0,     0.0,     0.0,    0.0,    0.0, 1.0,
        0.0,   0.0,    0.0,    0.0,       1.0, 0.0,
        0.0,   0.0,    0.0,    2.0,       0.0, 0.0,
        tf*tf*tf*tf*tf,     tf*tf*tf*tf,      tf*tf*tf,        tf*tf,     tf, 1.0,
        5.0*tf*tf*tf*tf,    4.0*tf*tf*tf,    3.0*tf*tf,    2.0*tf,        1.0, 0.0,
        20.0*tf*tf*tf,      12.0*tf*tf,       6.0*tf,        2.0,        0.0, 0.0;

    B << 0, 0, 0, 1.0, 0, 0;
    balance_update_polynomial_coeff_ = A.inverse() * B;

    desired_balance_param_ = req.balance_param;

    previous_balance_param_.cob_x_offset_m                  = online_walking->COB_X_MANUAL_ADJUSTMENT_M;
    previous_balance_param_.cob_y_offset_m                  = online_walking->COB_Y_MANUAL_ADJUSTMENT_M;

    previous_balance_param_.hip_roll_swap_angle_rad         = online_walking->HIP_ROLL_FEEDFORWARD_ANGLE_RAD;

    previous_balance_param_.gyro_gain                       = online_walking->WALK_STABILIZER_GAIN_RATIO;
    previous_balance_param_.foot_roll_angle_gain            = online_walking->BALANCE_ANKLE_ROLL_GAIN_BY_IMU;
    previous_balance_param_.foot_pitch_angle_gain           = online_walking->BALANCE_ANKLE_PITCH_GAIN_BY_IMU;

    previous_balance_param_.foot_x_force_gain               = online_walking->BALANCE_X_GAIN_BY_FT;
    previous_balance_param_.foot_y_force_gain               = online_walking->BALANCE_Y_GAIN_BY_FT;
    previous_balance_param_.foot_z_force_gain               = online_walking->BALANCE_Z_GAIN_BY_FT;
    previous_balance_param_.foot_roll_torque_gain           = online_walking->BALANCE_RIGHT_ROLL_GAIN_BY_FT;
    previous_balance_param_.foot_pitch_torque_gain          = online_walking->BALANCE_RIGHT_PITCH_GAIN_BY_FT;


    previous_balance_param_.foot_roll_angle_time_constant   = online_walking->BALANCE_ANKLE_ROLL_TIME_CONSTANT_BY_IMU;
    previous_balance_param_.foot_pitch_angle_time_constant  = online_walking->BALANCE_ANKLE_PITCH_TIME_CONSTANT_BY_IMU;

    previous_balance_param_.foot_x_force_time_constant      = online_walking->BALANCE_X_TIME_CONSTANT;
    previous_balance_param_.foot_y_force_time_constant      = online_walking->BALANCE_Y_TIME_CONSTANT;
    previous_balance_param_.foot_z_force_time_constant      = online_walking->BALANCE_Z_TIME_CONSTANT;
    previous_balance_param_.foot_roll_torque_time_constant  = online_walking->BALANCE_RIGHT_ANKLE_ROLL_TIME_CONSTANT;
    previous_balance_param_.foot_pitch_torque_time_constant = online_walking->BALANCE_RIGHT_ANKLE_PITCH_TIME_CONSTANT;

    balance_update_with_loop_ = true;

    std::string status_msg = WalkingStatusMSG::BALANCE_PARAM_SETTING_START_MSG;
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, status_msg);
  }

  return true;
}

void WalkingMotionModule::setBalanceParam(thormang3_walking_module_msgs::BalanceParam& balance_param_msg)
{
  RobotisOnlineWalking *online_walking = RobotisOnlineWalking::getInstance();

  online_walking->COB_X_MANUAL_ADJUSTMENT_M = balance_param_msg.cob_x_offset_m;
  online_walking->COB_Y_MANUAL_ADJUSTMENT_M = balance_param_msg.cob_y_offset_m;
  online_walking->HIP_ROLL_FEEDFORWARD_ANGLE_RAD = balance_param_msg.hip_roll_swap_angle_rad;

  online_walking->WALK_STABILIZER_GAIN_RATIO = balance_param_msg.gyro_gain;
  online_walking->BALANCE_X_GAIN     = +1.0*20.30*0.625*(online_walking->FORCE_MOMENT_DISTRIBUTION_RATIO)*online_walking->WALK_STABILIZER_GAIN_RATIO;
  online_walking->BALANCE_Y_GAIN     = -1.0*20.30*0.75*(online_walking->FORCE_MOMENT_DISTRIBUTION_RATIO)*online_walking->WALK_STABILIZER_GAIN_RATIO;
  online_walking->BALANCE_Z_GAIN     =    0.0*2.0*(online_walking->FORCE_MOMENT_DISTRIBUTION_RATIO)*online_walking->WALK_STABILIZER_GAIN_RATIO;

  online_walking->BALANCE_PITCH_GAIN = -1.0*0.10*0.5 *(1.0 - online_walking->FORCE_MOMENT_DISTRIBUTION_RATIO)*online_walking->WALK_STABILIZER_GAIN_RATIO;
  online_walking->BALANCE_ROLL_GAIN  = -1.0*0.10*0.75*(1.0 - online_walking->FORCE_MOMENT_DISTRIBUTION_RATIO)*online_walking->WALK_STABILIZER_GAIN_RATIO;

  online_walking->BALANCE_ANKLE_ROLL_GAIN_BY_IMU  = balance_param_msg.foot_roll_angle_gain;
  online_walking->BALANCE_ANKLE_PITCH_GAIN_BY_IMU = balance_param_msg.foot_pitch_angle_gain;

  online_walking->BALANCE_X_GAIN_BY_FT                = balance_param_msg.foot_x_force_gain;
  online_walking->BALANCE_Y_GAIN_BY_FT                = balance_param_msg.foot_y_force_gain;
  online_walking->BALANCE_Z_GAIN_BY_FT                = balance_param_msg.foot_z_force_gain;
  online_walking->BALANCE_RIGHT_ROLL_GAIN_BY_FT        = balance_param_msg.foot_roll_torque_gain;
  online_walking->BALANCE_LEFT_ROLL_GAIN_BY_FT        = balance_param_msg.foot_roll_torque_gain;
  online_walking->BALANCE_RIGHT_PITCH_GAIN_BY_FT    = balance_param_msg.foot_pitch_torque_gain;
  online_walking->BALANCE_LEFT_PITCH_GAIN_BY_FT        = balance_param_msg.foot_pitch_torque_gain;

  online_walking->BALANCE_ANKLE_ROLL_TIME_CONSTANT_BY_IMU  = balance_param_msg.foot_roll_angle_time_constant;
  online_walking->BALANCE_ANKLE_PITCH_TIME_CONSTANT_BY_IMU = balance_param_msg.foot_pitch_angle_time_constant;

  online_walking->BALANCE_X_TIME_CONSTANT                    = balance_param_msg.foot_x_force_time_constant;
  online_walking->BALANCE_Y_TIME_CONSTANT                    = balance_param_msg.foot_y_force_time_constant;
  online_walking->BALANCE_Z_TIME_CONSTANT                    = balance_param_msg.foot_z_force_time_constant;
  online_walking->BALANCE_RIGHT_ANKLE_ROLL_TIME_CONSTANT    = balance_param_msg.foot_roll_torque_time_constant;
  online_walking->BALANCE_LEFT_ANKLE_ROLL_TIME_CONSTANT        = balance_param_msg.foot_roll_torque_time_constant;
  online_walking->BALANCE_RIGHT_ANKLE_PITCH_TIME_CONSTANT    = balance_param_msg.foot_pitch_torque_time_constant;
  online_walking->BALANCE_LEFT_ANKLE_PITCH_TIME_CONSTANT    = balance_param_msg.foot_pitch_torque_time_constant;
}

bool WalkingMotionModule::removeExistingStepDataServiceCallback(thormang3_walking_module_msgs::RemoveExistingStepData::Request  &req,
    thormang3_walking_module_msgs::RemoveExistingStepData::Response &res)
{
  RobotisOnlineWalking *online_walking = RobotisOnlineWalking::getInstance();

  res.result = thormang3_walking_module_msgs::RemoveExistingStepData::Response::NO_ERROR;

  if(isRunning())
    res.result |= thormang3_walking_module_msgs::RemoveExistingStepData::Response::ROBOT_IS_WALKING_NOW;
  else {
    int exist_num_of_step_data = online_walking->GetNumofRemainingUnreservedStepData();
    if(exist_num_of_step_data != 0)
      for(int remove_count  = 0; remove_count < exist_num_of_step_data; remove_count++)
        online_walking->EraseLastStepData();
  }
  return true;
}


void WalkingMotionModule::imuDataOutputCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
  RobotisOnlineWalking::getInstance()->current_gyro_roll_rad_per_sec  = -1.0*(msg->angular_velocity.x);
  RobotisOnlineWalking::getInstance()->current_gyro_pitch_rad_per_sec = -1.0*(msg->angular_velocity.y);


  Eigen::Quaterniond imu_quat;
  tf::quaternionMsgToEigen(msg->orientation, imu_quat);

  Eigen::MatrixXd imu_mat = (rot_x_pi_3d_*(imu_quat.toRotationMatrix()))*rot_z_pi_3d_;

  double roll  = atan2( imu_mat.coeff(2,1), imu_mat.coeff(2,2));
  double pitch = atan2(-imu_mat.coeff(2,0), sqrt(robotis_framework::powDI(imu_mat.coeff(2,1), 2) + robotis_framework::powDI(imu_mat.coeff(2,2), 2)));
  double yaw   = atan2( imu_mat.coeff(1,0), imu_mat.coeff(0,0));

  RobotisOnlineWalking::getInstance()->current_imu_roll_rad = roll;
  RobotisOnlineWalking::getInstance()->current_imu_pitch_rad = pitch;
}

void WalkingMotionModule::onModuleEnable()
{
  std::string status_msg = WalkingStatusMSG::WALKING_MODULE_IS_ENABLED_MSG;
  publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, status_msg);
}

void WalkingMotionModule::onModuleDisable()
{
  previous_running_ = present_running = false;

  RobotisOnlineWalking *prev_walking = RobotisOnlineWalking::getInstance();
  std::string status_msg = WalkingStatusMSG::WALKING_MODULE_IS_DISABLED_MSG;
  balance_update_with_loop_ = false;
  prev_walking->HIP_ROLL_FEEDFORWARD_ANGLE_RAD = 0.0;

  prev_walking->WALK_STABILIZER_GAIN_RATIO = 0.0;
  prev_walking->BALANCE_X_GAIN     = +1.0*20.30*0.625*(prev_walking->FORCE_MOMENT_DISTRIBUTION_RATIO)*prev_walking->WALK_STABILIZER_GAIN_RATIO;
  prev_walking->BALANCE_Y_GAIN     = -1.0*20.30*0.75*(prev_walking->FORCE_MOMENT_DISTRIBUTION_RATIO)*prev_walking->WALK_STABILIZER_GAIN_RATIO;
  prev_walking->BALANCE_Z_GAIN     =    0.0*2.0*(prev_walking->FORCE_MOMENT_DISTRIBUTION_RATIO)*prev_walking->WALK_STABILIZER_GAIN_RATIO;

  prev_walking->BALANCE_PITCH_GAIN = -1.0*0.10*0.5 *(1.0 - prev_walking->FORCE_MOMENT_DISTRIBUTION_RATIO)*prev_walking->WALK_STABILIZER_GAIN_RATIO;
  prev_walking->BALANCE_ROLL_GAIN  = -1.0*0.10*0.75*(1.0 - prev_walking->FORCE_MOMENT_DISTRIBUTION_RATIO)*prev_walking->WALK_STABILIZER_GAIN_RATIO;

  prev_walking->BALANCE_ANKLE_ROLL_GAIN_BY_IMU  = 0.0;
  prev_walking->BALANCE_ANKLE_PITCH_GAIN_BY_IMU = 0.0;

  prev_walking->BALANCE_X_GAIN_BY_FT              = 0.0;
  prev_walking->BALANCE_Y_GAIN_BY_FT              = 0.0;
  prev_walking->BALANCE_Z_GAIN_BY_FT              = 0.0;
  prev_walking->BALANCE_RIGHT_ROLL_GAIN_BY_FT     = 0.0;
  prev_walking->BALANCE_LEFT_ROLL_GAIN_BY_FT      = 0.0;
  prev_walking->BALANCE_RIGHT_PITCH_GAIN_BY_FT    = 0.0;
  prev_walking->BALANCE_LEFT_PITCH_GAIN_BY_FT     = 0.0;
  publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, status_msg);
}

void WalkingMotionModule::process(std::map<std::string, robotis_framework::Dynamixel *> dxls, std::map<std::string, double> sensors)
{
  if(enable_ == false)
    return;

  RobotisOnlineWalking *online_walking = RobotisOnlineWalking::getInstance();

  r_foot_fx_N_  = sensors["r_foot_fx_scaled_N"];
  r_foot_fy_N_  = sensors["r_foot_fy_scaled_N"];
  r_foot_fz_N_  = sensors["r_foot_fz_scaled_N"];
  r_foot_Tx_Nm_ = sensors["r_foot_tx_scaled_Nm"];
  r_foot_Ty_Nm_ = sensors["r_foot_ty_scaled_Nm"];
  r_foot_Tz_Nm_ = sensors["r_foot_tz_scaled_Nm"];

  l_foot_fx_N_  = sensors["l_foot_fx_scaled_N"];
  l_foot_fy_N_  = sensors["l_foot_fy_scaled_N"];
  l_foot_fz_N_  = sensors["l_foot_fz_scaled_N"];
  l_foot_Tx_Nm_ = sensors["l_foot_tx_scaled_Nm"];
  l_foot_Ty_Nm_ = sensors["l_foot_ty_scaled_Nm"];
  l_foot_Tz_Nm_ = sensors["l_foot_tz_scaled_Nm"];


  r_foot_fx_N_ = robotis_framework::sign(r_foot_fx_N_) * fmin( fabs(r_foot_fx_N_), 2000.0);
  r_foot_fy_N_ = robotis_framework::sign(r_foot_fy_N_) * fmin( fabs(r_foot_fy_N_), 2000.0);
  r_foot_fz_N_ = robotis_framework::sign(r_foot_fz_N_) * fmin( fabs(r_foot_fz_N_), 2000.0);
  r_foot_Tx_Nm_ = robotis_framework::sign(r_foot_Tx_Nm_) *fmin(fabs(r_foot_Tx_Nm_), 300.0);
  r_foot_Ty_Nm_ = robotis_framework::sign(r_foot_Ty_Nm_) *fmin(fabs(r_foot_Ty_Nm_), 300.0);
  r_foot_Tz_Nm_ = robotis_framework::sign(r_foot_Tz_Nm_) *fmin(fabs(r_foot_Tz_Nm_), 300.0);

  l_foot_fx_N_ = robotis_framework::sign(l_foot_fx_N_) * fmin( fabs(l_foot_fx_N_), 2000.0);
  l_foot_fy_N_ = robotis_framework::sign(l_foot_fy_N_) * fmin( fabs(l_foot_fy_N_), 2000.0);
  l_foot_fz_N_ = robotis_framework::sign(l_foot_fz_N_) * fmin( fabs(l_foot_fz_N_), 2000.0);
  l_foot_Tx_Nm_ = robotis_framework::sign(l_foot_Tx_Nm_) *fmin(fabs(l_foot_Tx_Nm_), 300.0);
  l_foot_Ty_Nm_ = robotis_framework::sign(l_foot_Ty_Nm_) *fmin(fabs(l_foot_Ty_Nm_), 300.0);
  l_foot_Tz_Nm_ = robotis_framework::sign(l_foot_Tz_Nm_) *fmin(fabs(l_foot_Tz_Nm_), 300.0);



  if(balance_update_with_loop_ == true)
  {
    balance_update_sys_time_ += control_cycle_msec_ * 0.001;
    if(balance_update_sys_time_ >= balance_update_duration_ ) {
      balance_update_sys_time_ = balance_update_duration_;
      balance_update_with_loop_ = false;
      setBalanceParam(desired_balance_param_);
      std::string _status_msg = WalkingStatusMSG::BALANCE_PARAM_SETTING_FINISH_MSG;
      publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, _status_msg);
    }
    else {
      double current_update_gain =  balance_update_polynomial_coeff_.coeff(0,0) * robotis_framework::powDI(balance_update_sys_time_ , 5)
      + balance_update_polynomial_coeff_.coeff(1,0) * robotis_framework::powDI(balance_update_sys_time_ , 4)
      + balance_update_polynomial_coeff_.coeff(2,0) * robotis_framework::powDI(balance_update_sys_time_ , 3)
      + balance_update_polynomial_coeff_.coeff(3,0) * robotis_framework::powDI(balance_update_sys_time_ , 2)
      + balance_update_polynomial_coeff_.coeff(4,0) * robotis_framework::powDI(balance_update_sys_time_ , 1)
      + balance_update_polynomial_coeff_.coeff(5,0) ;

      current_balance_param_.cob_x_offset_m                  = current_update_gain*(desired_balance_param_.cob_x_offset_m                   - previous_balance_param_.cob_x_offset_m                  ) + previous_balance_param_.cob_x_offset_m;
      current_balance_param_.cob_y_offset_m                  = current_update_gain*(desired_balance_param_.cob_y_offset_m                   - previous_balance_param_.cob_y_offset_m                  ) + previous_balance_param_.cob_y_offset_m;

      current_balance_param_.hip_roll_swap_angle_rad           = current_update_gain*(desired_balance_param_.hip_roll_swap_angle_rad         - previous_balance_param_.hip_roll_swap_angle_rad        ) + previous_balance_param_.hip_roll_swap_angle_rad;

      current_balance_param_.gyro_gain                       = current_update_gain*(desired_balance_param_.gyro_gain                         - previous_balance_param_.gyro_gain                      ) + previous_balance_param_.gyro_gain;
      current_balance_param_.foot_roll_angle_gain            = current_update_gain*(desired_balance_param_.foot_roll_angle_gain            - previous_balance_param_.foot_roll_angle_gain           ) + previous_balance_param_.foot_roll_angle_gain;
      current_balance_param_.foot_pitch_angle_gain           = current_update_gain*(desired_balance_param_.foot_pitch_angle_gain           - previous_balance_param_.foot_pitch_angle_gain          ) + previous_balance_param_.foot_pitch_angle_gain;

      current_balance_param_.foot_x_force_gain               = current_update_gain*(desired_balance_param_.foot_x_force_gain               - previous_balance_param_.foot_x_force_gain              ) + previous_balance_param_.foot_x_force_gain;
      current_balance_param_.foot_y_force_gain               = current_update_gain*(desired_balance_param_.foot_y_force_gain               - previous_balance_param_.foot_y_force_gain              ) + previous_balance_param_.foot_y_force_gain;
      current_balance_param_.foot_z_force_gain               = current_update_gain*(desired_balance_param_.foot_z_force_gain               - previous_balance_param_.foot_z_force_gain              ) + previous_balance_param_.foot_z_force_gain;
      current_balance_param_.foot_roll_torque_gain           = current_update_gain*(desired_balance_param_.foot_roll_torque_gain           - previous_balance_param_.foot_roll_torque_gain          ) + previous_balance_param_.foot_roll_torque_gain;
      current_balance_param_.foot_pitch_torque_gain          = current_update_gain*(desired_balance_param_.foot_pitch_torque_gain          - previous_balance_param_.foot_pitch_torque_gain         ) + previous_balance_param_.foot_pitch_torque_gain;


      current_balance_param_.foot_roll_angle_time_constant   = current_update_gain*(desired_balance_param_.foot_roll_angle_time_constant   - previous_balance_param_.foot_roll_angle_time_constant  ) + previous_balance_param_.foot_roll_angle_time_constant;
      current_balance_param_.foot_pitch_angle_time_constant  = current_update_gain*(desired_balance_param_.foot_pitch_angle_time_constant  - previous_balance_param_.foot_pitch_angle_time_constant ) + previous_balance_param_.foot_pitch_angle_time_constant;

      current_balance_param_.foot_x_force_time_constant      = current_update_gain*(desired_balance_param_.foot_x_force_time_constant      - previous_balance_param_.foot_x_force_time_constant     ) + previous_balance_param_.foot_x_force_time_constant;
      current_balance_param_.foot_y_force_time_constant      = current_update_gain*(desired_balance_param_.foot_y_force_time_constant      - previous_balance_param_.foot_y_force_time_constant     ) + previous_balance_param_.foot_y_force_time_constant;
      current_balance_param_.foot_z_force_time_constant      = current_update_gain*(desired_balance_param_.foot_z_force_time_constant      - previous_balance_param_.foot_z_force_time_constant     ) + previous_balance_param_.foot_z_force_time_constant;
      current_balance_param_.foot_roll_torque_time_constant  = current_update_gain*(desired_balance_param_.foot_roll_torque_time_constant  - previous_balance_param_.foot_roll_torque_time_constant ) + previous_balance_param_.foot_roll_torque_time_constant;
      current_balance_param_.foot_pitch_torque_time_constant = current_update_gain*(desired_balance_param_.foot_pitch_torque_time_constant - previous_balance_param_.foot_pitch_torque_time_constant) + previous_balance_param_.foot_pitch_torque_time_constant;


      setBalanceParam(current_balance_param_);
    }
  }

  online_walking->current_right_fx_N  = r_foot_fx_N_;
  online_walking->current_right_fy_N  = r_foot_fy_N_;
  online_walking->current_right_fz_N  = r_foot_fz_N_;
  online_walking->current_right_Tx_Nm = r_foot_Tx_Nm_;
  online_walking->current_right_Ty_Nm = r_foot_Ty_Nm_;
  online_walking->current_right_Tz_Nm = r_foot_Tz_Nm_;

  online_walking->current_left_fx_N  = l_foot_fx_N_;
  online_walking->current_left_fy_N  = l_foot_fy_N_;
  online_walking->current_left_fz_N  = l_foot_fz_N_;
  online_walking->current_left_Tx_Nm = l_foot_Tx_Nm_;
  online_walking->current_left_Ty_Nm = l_foot_Ty_Nm_;
  online_walking->current_left_Tz_Nm = l_foot_Tz_Nm_;


  online_walking->process();

  publish_mutex_.lock();
  desired_matrix_g_to_cob_   = online_walking->matGtoCOB;
  desired_matrix_g_to_rfoot_ = online_walking->matGtoRF;
  desired_matrix_g_to_lfoot_ = online_walking->matGtoLF;
  publish_mutex_.unlock();

  publishRobotPose();

  result_["r_leg_hip_y"]->goal_position_ = online_walking->m_OutAngleRad[0];
  result_["r_leg_hip_r"]->goal_position_ = online_walking->m_OutAngleRad[1];
  result_["r_leg_hip_p"]->goal_position_ = online_walking->m_OutAngleRad[2];
  result_["r_leg_kn_p" ]->goal_position_ = online_walking->m_OutAngleRad[3];
  result_["r_leg_an_p" ]->goal_position_ = online_walking->m_OutAngleRad[4];
  result_["r_leg_an_r" ]->goal_position_ = online_walking->m_OutAngleRad[5];

  result_["l_leg_hip_y"]->goal_position_ = online_walking->m_OutAngleRad[6];
  result_["l_leg_hip_r"]->goal_position_ = online_walking->m_OutAngleRad[7];
  result_["l_leg_hip_p"]->goal_position_ = online_walking->m_OutAngleRad[8];
  result_["l_leg_kn_p" ]->goal_position_ = online_walking->m_OutAngleRad[9];
  result_["l_leg_an_p" ]->goal_position_ = online_walking->m_OutAngleRad[10];
  result_["l_leg_an_r" ]->goal_position_ = online_walking->m_OutAngleRad[11];

  present_running = isRunning();
  if(previous_running_ != present_running) {
    if(present_running == true) {
      std::string _status_msg = WalkingStatusMSG::WALKING_START_MSG;
      publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, _status_msg);
    }
    else {
      std::string _status_msg = WalkingStatusMSG::WALKING_FINISH_MSG;
      publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, _status_msg);
    }
  }
  previous_running_ = present_running;
}

void WalkingMotionModule::stop()
{
  return;
}

