/*******************************************************************************
 * Copyright (c) 2016, ROBOTIS CO., LTD.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of ROBOTIS nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

/*
 * wholebody_module.cpp
 *
 *  Created on: Aug 10, 2016
 *      Author: sch
 */

#include <stdio.h>
#include "thormang3_wholebody_module/wholebody_module.h"

using namespace thormang3;

WholebodyModule::WholebodyModule()
  : control_cycle_sec_(0.008),
    gazebo_(false),
    is_moving_(false),
    wb_ik_solving_(false),
    wb_l_arm_planning_(false),
    wb_r_arm_planning_(false),
    is_balancing_(false),
    on_balance_gain_(false),
    off_balance_gain_(false),
    is_wheel_pose_(false),
    is_gain_updating_(false),
    p_gain_(32)
{
  enable_       = false;
  module_name_  = "wholebody_module";
  control_mode_ = robotis_framework::PositionControl;

  /* arm */
  result_["r_arm_sh_p1"]   = new robotis_framework::DynamixelState();
  result_["l_arm_sh_p1"]   = new robotis_framework::DynamixelState();
  result_["r_arm_sh_r"]    = new robotis_framework::DynamixelState();
  result_["l_arm_sh_r"]    = new robotis_framework::DynamixelState();
  result_["r_arm_sh_p2"]   = new robotis_framework::DynamixelState();
  result_["l_arm_sh_p2"]   = new robotis_framework::DynamixelState();
  result_["r_arm_el_y"]    = new robotis_framework::DynamixelState();
  result_["l_arm_el_y"]    = new robotis_framework::DynamixelState();
  result_["r_arm_wr_r"]    = new robotis_framework::DynamixelState();
  result_["l_arm_wr_r"]    = new robotis_framework::DynamixelState();
  result_["r_arm_wr_y"]    = new robotis_framework::DynamixelState();
  result_["l_arm_wr_y"]    = new robotis_framework::DynamixelState();
  result_["r_arm_wr_p"]    = new robotis_framework::DynamixelState();
  result_["l_arm_wr_p"]    = new robotis_framework::DynamixelState();

  /* leg */
  result_["r_leg_hip_y"]   = new robotis_framework::DynamixelState();
  result_["r_leg_hip_r"]   = new robotis_framework::DynamixelState();
  result_["r_leg_hip_p"]   = new robotis_framework::DynamixelState();
  result_["r_leg_kn_p"]    = new robotis_framework::DynamixelState();
  result_["r_leg_an_p"]    = new robotis_framework::DynamixelState();
  result_["r_leg_an_r"]    = new robotis_framework::DynamixelState();
  result_["l_leg_hip_y"]   = new robotis_framework::DynamixelState();
  result_["l_leg_hip_r"]   = new robotis_framework::DynamixelState();
  result_["l_leg_hip_p"]   = new robotis_framework::DynamixelState();
  result_["l_leg_kn_p"]    = new robotis_framework::DynamixelState();
  result_["l_leg_an_p"]    = new robotis_framework::DynamixelState();
  result_["l_leg_an_r"]    = new robotis_framework::DynamixelState();

  /* body */
  result_["torso_y"]       = new robotis_framework::DynamixelState();

  /* gripper */
//  result_["r_arm_grip"]    = new robotis_framework::DynamixelState();
//  result_["l_arm_grip"]    = new robotis_framework::DynamixelState();

  /* arm */
  joint_name_to_id_["r_arm_sh_p1"]  = 1;
  joint_name_to_id_["l_arm_sh_p1"]  = 2;
  joint_name_to_id_["r_arm_sh_r"]   = 3;
  joint_name_to_id_["l_arm_sh_r"]   = 4;
  joint_name_to_id_["r_arm_sh_p2"]  = 5;
  joint_name_to_id_["l_arm_sh_p2"]  = 6;
  joint_name_to_id_["r_arm_el_y"]   = 7;
  joint_name_to_id_["l_arm_el_y"]   = 8;
  joint_name_to_id_["r_arm_wr_r"]   = 9;
  joint_name_to_id_["l_arm_wr_r"]   = 10;
  joint_name_to_id_["r_arm_wr_y"]   = 11;
  joint_name_to_id_["l_arm_wr_y"]   = 12;
  joint_name_to_id_["r_arm_wr_p"]   = 13;
  joint_name_to_id_["l_arm_wr_p"]   = 14;

  /* leg */
  joint_name_to_id_["r_leg_hip_y"]  = 15;
  joint_name_to_id_["l_leg_hip_y"]  = 16;
  joint_name_to_id_["r_leg_hip_r"]  = 17;
  joint_name_to_id_["l_leg_hip_r"]  = 18;
  joint_name_to_id_["r_leg_hip_p"]  = 19;
  joint_name_to_id_["l_leg_hip_p"]  = 20;
  joint_name_to_id_["r_leg_kn_p"]   = 21;
  joint_name_to_id_["l_leg_kn_p"]   = 22;
  joint_name_to_id_["r_leg_an_p"]   = 23;
  joint_name_to_id_["l_leg_an_p"]   = 24;
  joint_name_to_id_["r_leg_an_r"]   = 25;
  joint_name_to_id_["l_leg_an_r"]   = 26;

  /* body */
  joint_name_to_id_["torso_y"]      = 27;

  /* gripper */
//  joint_name_to_id_["r_arm_grip"]   = 31;
//  joint_name_to_id_["l_arm_grip"]   = 30;

  /* end effector */
  joint_name_to_id_["r_arm_end"]    = 35;
  joint_name_to_id_["l_arm_end"]    = 34;

  /* ----- parameter initialization ----- */
  present_joint_position_  = Eigen::VectorXd::Zero(MAX_JOINT_ID+1);
  present_joint_velocity_  = Eigen::VectorXd::Zero(MAX_JOINT_ID+1);
  goal_joint_position_ = Eigen::VectorXd::Zero(MAX_JOINT_ID+1);

  via_num_ = 1;
  via_time_ = Eigen::MatrixXd::Zero(via_num_, 1);

  joint_ini_pose_ = Eigen::VectorXd::Zero(MAX_JOINT_ID+1);
  joint_ini_via_pose_ = Eigen::MatrixXd::Zero(via_num_,MAX_JOINT_ID+1);
  joint_ini_via_d_pose_ = Eigen::MatrixXd::Zero(via_num_, MAX_JOINT_ID + 1);
  joint_ini_via_dd_pose_ = Eigen::MatrixXd::Zero(via_num_, MAX_JOINT_ID + 1);

  wheel_joint_diff_pose_ = Eigen::VectorXd::Zero(MAX_JOINT_ID+1);

  wheel_ini_pose_ = Eigen::VectorXd::Zero(3);
  wheel_ini_via_pose_ = Eigen::MatrixXd::Zero(via_num_,3);
  wheel_ini_via_d_pose_ = Eigen::MatrixXd::Zero(via_num_,3);
  wheel_ini_via_dd_pose_ = Eigen::MatrixXd::Zero(via_num_,3);

  ik_target_position_ = Eigen::MatrixXd::Zero(3,1);
  wb_pelvis_target_position_ = Eigen::MatrixXd::Zero(3,1);
  wb_l_foot_target_position_ = Eigen::MatrixXd::Zero(3,1);
  wb_r_foot_target_position_ = Eigen::MatrixXd::Zero(3,1);
  wb_l_arm_target_position_ = Eigen::MatrixXd::Zero(3,1);
  wb_r_arm_target_position_ = Eigen::MatrixXd::Zero(3,1);

  wb_arm_diff_position_ = Eigen::VectorXd::Zero(3);

  wb_l_foot_default_position_ = Eigen::MatrixXd::Zero(3,1);
  wb_l_foot_default_position_.coeffRef(0,0) = 0.01;
  wb_l_foot_default_position_.coeffRef(1,0) = 0.093;

  wb_r_foot_default_position_ = Eigen::MatrixXd::Zero(3,1);
  wb_r_foot_default_position_.coeffRef(0,0) = 0.01;
  wb_r_foot_default_position_.coeffRef(1,0) = -0.093;

  wb_l_foot_default_quaternion_ = robotis_framework::convertRPYToQuaternion(0.0, 0.0, 0.0);
  wb_r_foot_default_quaternion_ = robotis_framework::convertRPYToQuaternion(0.0, 0.0, 0.0);

  ik_weight_ = Eigen::MatrixXd::Zero(ALL_JOINT_ID+1,1);

  // robot tree
  robotis_ = new KinematicsDynamics(WholeBody);

  balance_control_.setGyroBalanceEnable(false); // Gyro
  balance_control_.setOrientationBalanceEnable(false); // IMU
  balance_control_.setForceTorqueBalanceEnable(false); // FT

  double balance_gain_mov_time_ = 1.0;
  balance_gain_time_steps_ = int(balance_gain_mov_time_/control_cycle_sec_) + 1;

  on_balance_gain_tra_ = robotis_framework::calcMinimumJerkTra(0.0, 0.0, 0.0,
                                                               1.0, 0.0, 0.0,
                                                               control_cycle_sec_, balance_gain_mov_time_);
  off_balance_gain_tra_ = robotis_framework::calcMinimumJerkTra(1.0, 0.0, 0.0,
                                                                0.0, 0.0, 0.0,
                                                                control_cycle_sec_, balance_gain_mov_time_);
  // balance defalut gain
  gyro_gain_ = 0.5;
  foot_roll_angle_gain_ = -1.0;
  foot_pitch_angle_gain_ = -1.0;
  foot_roll_angle_time_constant_ = 0.2;
  foot_pitch_angle_time_constant_ = 0.2;

  left_foot_force_x_gain_ = 0.1;
  left_foot_force_y_gain_ = 0.1;
  left_foot_force_x_time_constant_ = 0.2;
  left_foot_force_x_time_constant_ = 0.2;

  right_foot_force_x_gain_ = 0.1;
  right_foot_force_y_gain_ = 0.1;
  right_foot_force_x_time_constant_ = 0.2;
  right_foot_force_x_time_constant_ = 0.2;

  foot_force_z_gain_ = -0.02;
  foot_force_z_time_constant_ = 0.2;

  left_foot_torque_roll_gain_ = 0.002;
  left_foot_torque_pitch_gain_ = 0.0015;
  left_foot_torque_roll_time_constant_ = 0.2;
  left_foot_torque_pitch_time_constant_ = 0.2;

  right_foot_torque_roll_gain_ = 0.002;
  right_foot_torque_pitch_gain_ = 0.0015;
  right_foot_torque_roll_time_constant_ = 0.2;
  right_foot_torque_pitch_time_constant_ = 0.2;

  wb_pelvis_diff_x_constant_ = -0.075;
  wb_pelvis_diff_y_constant_ = -0.075;
  wb_pelvis_diff_z_constant_ = 0.1;

  balance_goal_l_foot_ft_ = -210.0;
  balance_goal_r_foot_ft_ = -210.0;

  default_center_of_mass_ = Eigen::MatrixXd::Zero(3,1);
  default_center_of_mass_.coeffRef(0,0) = 0.024;
  default_center_of_mass_.coeffRef(1,0) = -0.004;
  default_center_of_mass_.coeffRef(2,0) = 0.65;
}

WholebodyModule::~WholebodyModule()
{
  queue_thread_.join();
}

void WholebodyModule::initialize(const int control_cycle_msec, robotis_framework::Robot *robot)
{
  ros::NodeHandle ros_node;
  ros_node.getParam("gazebo", gazebo_);

  control_cycle_sec_ = control_cycle_msec * 0.001;
  queue_thread_ = boost::thread(boost::bind(&WholebodyModule::queueThread, this));
}

void WholebodyModule::queueThread()
{
  ros::NodeHandle    ros_node;
  ros::CallbackQueue callback_queue;

  ros_node.setCallbackQueue(&callback_queue);

  /* publish topics */
  status_msg_pub_ = ros_node.advertise<robotis_controller_msgs::StatusMsg>("/robotis/status", 1);
  set_ctrl_module_pub_ = ros_node.advertise<std_msgs::String>("/robotis/enable_ctrl_module", 1);
  goal_torque_limit_pub_ = ros_node.advertise<robotis_controller_msgs::SyncWriteItem>("/robotis/sync_write_item", 1);

  /* subscribe topics */
  ros::Subscriber set_mode_msg_sub = ros_node.subscribe("/robotis/wholebody/set_mode_msg", 5,
                                                        &WholebodyModule::setModeMsgCallback, this);
  ros::Subscriber ini_pose_msg_sub = ros_node.subscribe("/robotis/wholebody/ini_pose_msg", 5,
                                                        &WholebodyModule::setIniPoseMsgCallback, this);
  ros::Subscriber joint_pose_msg_sub = ros_node.subscribe("/robotis/wholebody/joint_pose_msg", 5,
                                                          &WholebodyModule::setJointPoseMsgCallback, this);
  ros::Subscriber kinematics_pose_msg_sub = ros_node.subscribe("/robotis/wholebody/kinematics_pose_msg", 5,
                                                               &WholebodyModule::setKinematicsPoseMsgCallback, this);
  ros::Subscriber wholebody_balance_msg_sub = ros_node.subscribe("/robotis/wholebody/wholebody_balance_msg", 5,
                                                                 &WholebodyModule::setWholebodyBalanceMsgCallback, this);
  ros::Subscriber imu_data_sub = ros_node.subscribe("/robotis/sensor/imu/imu", 5,
                                                   &WholebodyModule::imuDataCallback, this);

  /* service */
  ros::ServiceServer get_joint_pose_server = ros_node.advertiseService("/robotis/wholebody/get_joint_pose",
                                                                       &WholebodyModule::getJointPoseCallback, this);
  ros::ServiceServer get_kinematics_pose_server = ros_node.advertiseService("/robotis/wholebody/get_kinematics_pose",
                                                                            &WholebodyModule::getKinematicsPoseCallback, this);



  std::string ik_weight_path = ros::package::getPath("thormang3_wholebody_module") + "/config/ik_weight.yaml";
  parseInverseKinematicsWeightData(ik_weight_path);

  std::string balance_gain_path = ros::package::getPath("thormang3_wholebody_module") + "/config/balance_gain.yaml";
  parseBalanceGainData(balance_gain_path);

  while (ros_node.ok())
  {
    callback_queue.callAvailable();
    usleep(1000);
  }
}

void WholebodyModule::setModeMsgCallback(const std_msgs::String::ConstPtr& msg)
{
  //  ROS_INFO("--- Set Torque Control Mode ---");
  std_msgs::String str_msg;
  str_msg.data = "wholebody_module";
  set_ctrl_module_pub_.publish(str_msg);
  return;
}

void WholebodyModule::parseInverseKinematicsWeightData(const std::string &path)
{
  YAML::Node doc;
  try
  {
    // load yaml
    doc = YAML::LoadFile( path.c_str() );
  }
  catch(const std::exception& e)
  {
    ROS_ERROR("Fail to load yaml file.");
    return;
  }

  YAML::Node inverse_kinematics_weight_node = doc["weight_value"];
  for(YAML::iterator it = inverse_kinematics_weight_node.begin(); it != inverse_kinematics_weight_node.end() ; ++it)
  {
    int id = it->first.as<int>();
    double value = it->second.as<double>();

    ik_weight_.coeffRef(id,0) = value;
  }
}

void WholebodyModule::parseIniPoseData(const std::string &path)
{
  YAML::Node doc;
  try
  {
    // load yaml
    doc = YAML::LoadFile(path.c_str());
  } catch (const std::exception& e)
  {
    ROS_ERROR("Fail to load yaml file.");
    return;
  }

  // parse movement time
  mov_time_ = doc["mov_time"].as<double>();

  // parse via-point number
  via_num_ = doc["via_num"].as<int>();

  // parse via-point time
  std::vector<double> via_time;
  via_time = doc["via_time"].as<std::vector<double> >();

  via_time_.resize(via_num_, 1);
  for (int num = 0; num < via_num_; num++)
    via_time_.coeffRef(num, 0) = via_time[num];

  // parse via-point pose
  joint_ini_via_pose_.resize(via_num_, MAX_JOINT_ID + 1);
  joint_ini_via_d_pose_.resize(via_num_, MAX_JOINT_ID + 1);
  joint_ini_via_dd_pose_.resize(via_num_, MAX_JOINT_ID + 1);

  joint_ini_via_pose_.fill(0.0);
  joint_ini_via_d_pose_.fill(0.0);
  joint_ini_via_dd_pose_.fill(0.0);

  YAML::Node via_pose_node = doc["via_pose"];
  for (YAML::iterator it = via_pose_node.begin(); it != via_pose_node.end(); ++it)
  {
    int id;
    std::vector<double> value;

    id = it->first.as<int>();
    value = it->second.as<std::vector<double> >();

    for (int num = 0; num < via_num_; num++)
      joint_ini_via_pose_.coeffRef(num, id) = value[num] * DEGREE2RADIAN;
  }

  // parse target pose
  YAML::Node tar_pose_node = doc["tar_pose"];
  for (YAML::iterator it = tar_pose_node.begin(); it != tar_pose_node.end(); ++it)
  {
    int id;
    double value;

    id = it->first.as<int>();
    value = it->second.as<double>();

    joint_ini_pose_(id) = value * DEGREE2RADIAN;
  }

  all_time_steps_ = int(mov_time_ / control_cycle_sec_) + 1;
  goal_joint_tra_.resize(all_time_steps_, MAX_JOINT_ID + 1);
}

void WholebodyModule::parseWheelPoseData(const std::string &path)
{
  YAML::Node doc;
  try
  {
    // load yaml
    doc = YAML::LoadFile(path.c_str());
  } catch (const std::exception& e)
  {
    ROS_ERROR("Fail to load yaml file.");
    return;
  }

  // parse movement time
  mov_time_ = doc["mov_time"].as<double>();

  // parse via-point number
  via_num_ = doc["via_num"].as<int>();

  // parse via-point time
  std::vector<double> via_time;
  via_time = doc["via_time"].as<std::vector<double> >();

  via_time_.resize(via_num_, 1);
  for (int num = 0; num < via_num_; num++)
    via_time_.coeffRef(num, 0) = via_time[num];

  wheel_ini_via_pose_.resize(via_num_,3);
  wheel_ini_via_d_pose_.resize(via_num_,3);
  wheel_ini_via_dd_pose_.resize(via_num_,3);

  wheel_ini_via_pose_.fill(0.0);
  wheel_ini_via_d_pose_.fill(0.0);
  wheel_ini_via_dd_pose_.fill(0.0);

  YAML::Node via_pose_node = doc["via_pose"];
  for (YAML::iterator it = via_pose_node.begin(); it != via_pose_node.end(); ++it)
  {
    int dim;
    std::vector<double> value;

    dim = it->first.as<int>();
    value = it->second.as<std::vector<double> >();

    for (int num = 0; num < via_num_; num++)
      wheel_ini_via_pose_.coeffRef(num, dim) = value[num];
  }

  // parse target pose
  YAML::Node tar_pose_node = doc["tar_pose"];
  for (YAML::iterator it = tar_pose_node.begin(); it != tar_pose_node.end(); ++it)
  {
    int dim;
    double value;

    dim = it->first.as<int>();
    value = it->second.as<double>();

    wheel_ini_pose_(dim) = value;
  }
}

void WholebodyModule::parseWheelJointPoseData(const std::string &path)
{
  YAML::Node doc;
  try
  {
    // load yaml
    doc = YAML::LoadFile(path.c_str());
  } catch (const std::exception& e)
  {
    ROS_ERROR("Fail to load yaml file.");
    return;
  }

  // parse movement time
  mov_time_ = doc["mov_time"].as<double>();

  // parse target pose
  YAML::Node tar_pose_node = doc["tar_pose"];
  for (YAML::iterator it = tar_pose_node.begin(); it != tar_pose_node.end(); ++it)
  {
    int id;
    double value;

    id = it->first.as<int>();
    value = it->second.as<double>();

    wheel_joint_diff_pose_(id) = value * DEGREE2RADIAN - goal_joint_position_(id);
  }

  all_time_steps_ = int(mov_time_ / control_cycle_sec_) + 1;
  goal_joint_tra_.resize(all_time_steps_, MAX_JOINT_ID + 1);
}

void WholebodyModule::parseBalanceGainData(const std::string &path)
{
  YAML::Node doc;
  try
  {
    // load yaml
    doc = YAML::LoadFile(path.c_str());
  }
  catch (const std::exception& e)
  {
    ROS_ERROR("Fail to load yaml file.");
    return;
  }

  ROS_INFO("Parse Balance Gain Data");

  gyro_gain_ = doc["gyro_gain"].as<double>();

  foot_roll_angle_gain_ = doc["foot_roll_angle_gain"].as<double>();
  foot_pitch_angle_gain_ = doc["foot_pitch_angle_gain"].as<double>();
  foot_roll_angle_time_constant_ = doc["foot_roll_angle_time_constant"].as<double>();
  foot_pitch_angle_time_constant_ = doc["foot_pitch_angle_time_constant"].as<double>();

  left_foot_force_x_gain_ = doc["left_foot_force_x_gain"].as<double>();
  left_foot_force_y_gain_ = doc["left_foot_force_y_gain"].as<double>();
  left_foot_force_x_time_constant_ = doc["left_foot_force_x_time_constant"].as<double>();
  left_foot_force_y_time_constant_ = doc["left_foot_force_y_time_constant"].as<double>();

  right_foot_force_x_gain_ = doc["right_foot_force_x_gain"].as<double>();
  right_foot_force_y_gain_ = doc["right_foot_force_y_gain"].as<double>();
  right_foot_force_x_time_constant_ = doc["right_foot_force_x_time_constant"].as<double>();
  right_foot_force_y_time_constant_ = doc["right_foot_force_y_time_constant"].as<double>();

  foot_force_z_gain_ = doc["foot_force_z_gain"].as<double>();
  foot_force_z_time_constant_ = doc["foot_force_z_time_constant"].as<double>();

  left_foot_torque_roll_gain_ = doc["left_foot_torque_roll_gain"].as<double>();
  left_foot_torque_pitch_gain_ = doc["left_foot_torque_pitch_gain"].as<double>();
  left_foot_torque_roll_time_constant_ = doc["left_foot_torque_roll_time_constant"].as<double>();
  left_foot_torque_pitch_time_constant_ = doc["left_foot_torque_pitch_time_constant"].as<double>();

  right_foot_torque_roll_gain_ = doc["right_foot_torque_roll_gain"].as<double>();
  right_foot_torque_pitch_gain_ = doc["right_foot_torque_pitch_gain"].as<double>();
  right_foot_torque_roll_time_constant_ = doc["right_foot_torque_roll_time_constant"].as<double>();
  right_foot_torque_pitch_time_constant_ = doc["right_foot_torque_pitch_time_constant"].as<double>();

  wb_pelvis_diff_x_constant_ = doc["wb_pelvis_diff_x_constant"].as<double>();
  wb_pelvis_diff_y_constant_ = doc["wb_pelvis_diff_y_constant"].as<double>();
  wb_pelvis_diff_z_constant_ = doc["wb_pelvis_diff_z_constant"].as<double>();
}

void WholebodyModule::setWholebodyBalanceMsgCallback(const std_msgs::String::ConstPtr& msg)
{
  if (msg->data == "balance_on")
  {
    ROS_INFO("balance on");

    is_balancing_ = true;
    on_balance_gain_ = true;
    balance_gain_cnt_ = 0;

    is_moving_ = false;
    wb_ik_solving_ = false;
  }
  else if(msg->data == "balance_off")
  {
    ROS_INFO("balance off");

    on_balance_gain_ = false;
    balance_gain_cnt_ = 0;

    is_moving_ = false;
    wb_ik_solving_ = false;
  }
}

void WholebodyModule::imuDataCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
  imu_data_msg_ = *msg;

  imu_data_msg_.angular_velocity.x *= -1.0;
  imu_data_msg_.angular_velocity.y *= -1.0;
}

void WholebodyModule::setIniPoseMsgCallback(const std_msgs::String::ConstPtr& msg)
{
  if(enable_ == false)
    return;

  if (is_moving_ == false)
  {
    if (msg->data == "ini_pose" || msg->data == "base_pose")
    {
      // parse initial pose
      std::string ini_pose_path = ros::package::getPath("thormang3_wholebody_module") + "/config/" + msg->data +".yaml";
      parseIniPoseData(ini_pose_path);

      tra_gene_tread_ = new boost::thread(boost::bind(&WholebodyModule::traGeneProcIniPose, this));
      delete tra_gene_tread_;
    }
    else if (msg->data == "wheel_sit_down" || msg->data == "wheel_stand_up")
    {
      if (is_balancing_ == true)
      {
        std::string wheel_pose_path = ros::package::getPath("thormang3_wholebody_module") + "/config/" + msg->data +".yaml";
        parseWheelPoseData(wheel_pose_path);

        tra_gene_tread_ = new boost::thread(boost::bind(&WholebodyModule::traGeneProcWheelPose, this));
        delete tra_gene_tread_;
      }
      else
        ROS_INFO("balance is off");
    }
    else if (msg->data == "wheel_on_pose" || msg->data == "wheel_off_pose")
    {
      if (msg->data == "wheel_on_pose")
      {
        is_wheel_pose_ = true;
        std::string wheel_joint_pose_path = ros::package::getPath("thormang3_wholebody_module") + "/config/" + "wheel_joint_pose.yaml";
        parseWheelJointPoseData(wheel_joint_pose_path);
      }
      else if (msg->data == "wheel_off_pose")
        is_wheel_pose_ = false;

      tra_gene_tread_ = new boost::thread(boost::bind(&WholebodyModule::traGeneProcWheelJointPose, this));
      delete tra_gene_tread_;
    }
    else if (msg->data == "knee_p_gain_up" || msg->data == "knee_p_gain_down")
    {
      robotis_controller_msgs::SyncWriteItem sync_write_msg;
      sync_write_msg.item_name = "goal_torque";

      if (msg->data == "knee_p_gain_down")
      {
        sync_write_msg.joint_name.push_back("r_leg_kn_p");
        sync_write_msg.value.push_back(105);
        sync_write_msg.joint_name.push_back("l_leg_kn_p");
        sync_write_msg.value.push_back(105);
      }
      else if (msg->data == "knee_p_gain_up")
      {
        sync_write_msg.joint_name.push_back("r_leg_kn_p");
        sync_write_msg.value.push_back(1240);
        sync_write_msg.joint_name.push_back("l_leg_kn_p");
        sync_write_msg.value.push_back(1240);
      }

      goal_torque_limit_pub_.publish(sync_write_msg);
    }
  }
  else
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_WARN, "Previous task is alive");

  return;
}

void WholebodyModule::setJointPoseMsgCallback(const thormang3_wholebody_module_msgs::JointPose::ConstPtr& msg)
{
  if(enable_ == false)
    return;

  goal_joint_pose_msg_ = *msg;

  if (is_moving_ == false)
  {
    tra_gene_tread_ = new boost::thread(boost::bind(&WholebodyModule::traGeneProcJointSpace, this));
    delete tra_gene_tread_;
  }
  else
    ROS_INFO("previous task is alive");

  return;
}

void WholebodyModule::setKinematicsPoseMsgCallback(const thormang3_wholebody_module_msgs::KinematicsPose::ConstPtr& msg)
{
  if(enable_ == false)
    return;

  goal_kinematics_pose_msg_ = *msg;

  if (is_balancing_ == true)
  {
    if (is_moving_ == false)
    {
//      if (goal_kinematics_pose_msg_.name == "left_arm" || goal_kinematics_pose_msg_.name == "left_arm_with_torso")
//      {
//        ik_id_start_ = ID_L_ARM_START;
//        if (goal_kinematics_pose_msg_.name == "left_arm_with_torso")
//          ik_id_start_ = ID_TORSO;

//        ik_id_end_ = ID_L_ARM_END;

//        tra_gene_tread_ = new boost::thread(boost::bind(&WholebodyModule::traGeneProcForTaskSpace, this));
//        delete tra_gene_tread_;
//      }
//      else if (goal_kinematics_pose_msg_.name == "right_arm" || goal_kinematics_pose_msg_.name == "right_arm_with_torso")
//      {
//        ik_id_start_ = ID_R_ARM_START;
//        if (goal_kinematics_pose_msg_.name == "right_arm_with_torso")
//          ik_id_start_ = ID_TORSO;

//        ik_id_end_ = ID_R_ARM_END;

//        tra_gene_tread_ = new boost::thread(boost::bind(&WholebodyModule::traGeneProcForTaskSpace, this));
//        delete tra_gene_tread_;
//      }
      if (goal_kinematics_pose_msg_.name == "pelvis")
      {
//        ik_id_start_ = ID_PELVIS;
//        ik_id_end_ = ID_PELVIS;

        tra_gene_tread_ = new boost::thread(boost::bind(&WholebodyModule::traGeneProcPelvis, this));
        delete tra_gene_tread_;
      }
      else if (goal_kinematics_pose_msg_.name == "left_arm_wholebody")
      {
//        ik_id_start_ = ID_BASE;
//        ik_id_end_ = ID_L_ARM_END;

        wb_l_arm_planning_ = true;
        wb_r_arm_planning_ = false;

        tra_gene_tread_ = new boost::thread(boost::bind(&WholebodyModule::traGeneProcWholebody, this));
        delete tra_gene_tread_;
      }
      else if (goal_kinematics_pose_msg_.name == "right_arm_wholebody")
      {
//        ik_id_start_ = ID_BASE;
//        ik_id_end_ = ID_R_ARM_END;

        wb_l_arm_planning_ = false;
        wb_r_arm_planning_ = true;

        tra_gene_tread_ = new boost::thread(boost::bind(&WholebodyModule::traGeneProcWholebody, this));
        delete tra_gene_tread_;
      }
//      else if (goal_kinematics_pose_msg_.name == "stand_wheel_pose" || goal_kinematics_pose_msg_.name == "stand_wheel_pose_back")
//      {
//        ik_id_start_ = ID_PELVIS;
//        ik_id_end_ = ID_PELVIS;

//        tra_gene_tread_ = new boost::thread(boost::bind(&WholebodyModule::traGeneProcForStandWheelPose, this));
//        delete tra_gene_tread_;
//      }
    }
    else
      ROS_INFO("previous task is alive");
  }
  else
    ROS_INFO("balance is off");

  return;
}

void WholebodyModule::traGeneProcIniPose()
{
  for (int id = 1; id <= MAX_JOINT_ID; id++)
  {
    double ini_value = goal_joint_position_(id);
    double tar_value = joint_ini_pose_(id);

    Eigen::MatrixXd tra;

    if (via_num_ == 0)
    {
      tra = robotis_framework::calcMinimumJerkTra(ini_value, 0.0, 0.0,
                                                  tar_value, 0.0, 0.0,
                                                  control_cycle_sec_, mov_time_);
    }
    else
    {
      Eigen::MatrixXd via_value = joint_ini_via_pose_.col(id);
      Eigen::MatrixXd d_via_value = joint_ini_via_d_pose_.col(id);
      Eigen::MatrixXd dd_via_value = joint_ini_via_dd_pose_.col(id);

      tra = robotis_framework::calcMinimumJerkTraWithViaPoints(via_num_,
                                                               ini_value, 0.0, 0.0,
                                                               via_value, d_via_value, dd_via_value,
                                                               tar_value, 0.0, 0.0,
                                                               control_cycle_sec_, via_time_, mov_time_);
    }

    goal_joint_tra_.block(0, id, all_time_steps_, 1) = tra;
  }

  is_moving_ = true;
  is_balancing_ = false;
  cnt_ = 0;

  robotis_->thormang3_link_data_[ID_PELVIS_POS_X]->relative_position_.coeffRef(0,0) = 0.0;
  robotis_->thormang3_link_data_[ID_PELVIS_POS_Y]->relative_position_.coeffRef(1,0) = 0.0;
  robotis_->thormang3_link_data_[ID_PELVIS_POS_Z]->relative_position_.coeffRef(2,0) = 0.723;

  robotis_->thormang3_link_data_[ID_PELVIS_ROT_X]->joint_angle_ = 0.0;
  robotis_->thormang3_link_data_[ID_PELVIS_ROT_Y]->joint_angle_ = 0.0;
  robotis_->thormang3_link_data_[ID_PELVIS_ROT_Z]->joint_angle_ = 0.0;

  ROS_INFO("[start] send trajectory");
}

void WholebodyModule::traGeneProcWheelPose()
{
  all_time_steps_ = int(mov_time_ / control_cycle_sec_) + 1;
  goal_pelvis_tra_.resize(all_time_steps_, 3);
  goal_l_foot_tra_.resize(all_time_steps_, 3);
  goal_r_foot_tra_.resize(all_time_steps_, 3);

  for (int dim = 0; dim < 3; dim++)
  {
    double ini_value = robotis_->thormang3_link_data_[ID_PELVIS]->position_.coeff(dim,0);
    double tar_value = wheel_ini_pose_(dim);

    Eigen::MatrixXd tra;
    if (via_num_ == 0)
    {
      tra = robotis_framework::calcMinimumJerkTra(ini_value, 0.0, 0.0,
                                                  tar_value, 0.0, 0.0,
                                                  control_cycle_sec_, mov_time_);
    }
    else
    {
      Eigen::MatrixXd via_value = wheel_ini_via_pose_.col(dim);
      Eigen::MatrixXd d_via_value = wheel_ini_via_d_pose_.col(dim);
      Eigen::MatrixXd dd_via_value = wheel_ini_via_dd_pose_.col(dim);

      tra = robotis_framework::calcMinimumJerkTraWithViaPoints(via_num_,
                                                               ini_value, 0.0, 0.0,
                                                               via_value, d_via_value, dd_via_value,
                                                               tar_value, 0.0, 0.0,
                                                               control_cycle_sec_, via_time_, mov_time_);
    }
    goal_pelvis_tra_.block(0, dim, all_time_steps_, 1) = tra;
  }

  /* target quaternion */
  wb_pelvis_goal_quaternion_ =
      robotis_framework::convertRotationToQuaternion(robotis_->thormang3_link_data_[ID_PELVIS]->orientation_);

  calcGoalTraLeg();

  cnt_ = 0;
  is_moving_ = true;
  wb_ik_solving_ = true;

  ROS_INFO("[start] send trajectory");
}

void WholebodyModule::traGeneProcWheelJointPose()
{
  double constant;

  if (is_wheel_pose_==true)
    constant = 1.0;
  else
    constant = -1.0;

  for (int id = 1; id <= MAX_JOINT_ID; id++)
  {
    double ini_value = goal_joint_position_(id);
    double tar_value = goal_joint_position_(id) + constant * wheel_joint_diff_pose_(id) ;

    Eigen::MatrixXd  tra = robotis_framework::calcMinimumJerkTra(ini_value, 0.0, 0.0,
                                                                 tar_value, 0.0, 0.0,
                                                                 control_cycle_sec_, mov_time_);

    goal_joint_tra_.block(0, id, all_time_steps_, 1) = tra;
  }

  is_moving_ = true;
  is_balancing_ = false;
  cnt_ = 0;
}

//void WholebodyModule::traGeneProcForStandWheelPose()
//{
//  mov_time_ = 3.0;
//  all_time_steps_ = int(mov_time_/control_cycle_msec_) + 1;

//  goal_pevlis_tra_.resize(all_time_steps_, 3);
//  goal_l_foot_tra_.resize(all_time_steps_, 3);
//  goal_r_foot_tra_.resize(all_time_steps_, 3);

//  Eigen::MatrixXd pelvis_tar_vector = Eigen::MatrixXd::Zero(3,1);
//  pelvis_tar_vector.coeffRef(2,0) = 0.0;

//  if ( goal_kinematics_pose_msg_.name == "stand_wheel_pose_back" )
//  {
//    pelvis_tar_vector.coeffRef(2,0) = 0.0;
//  }

//  /* calculate trajectory */
//  for (int dim=0; dim<3; dim++)
//  {
//    double ini_value = robotis_->thormang3_link_data_[ID_PELVIS]->position_.coeff(dim,0);
//    double tar_value = ini_value + pelvis_tar_vector.coeff(dim,0);

//    Eigen::MatrixXd tra = robotis_framework::calcMinimumJerkTra(ini_value, 0.0, 0.0,
//                                                                tar_value, 0.0, 0.0,
//                                                                control_cycle_msec_, mov_time_);

//    goal_pevlis_tra_.block(0, dim, all_time_steps_, 1) = tra;
//  }

//  for (int dim=0; dim<3; dim++)
//  {
//    double ini_value = robotis_->thormang3_link_data_[ID_L_LEG_END]->position_.coeff(dim,0);
//    double tar_value = robotis_->thormang3_link_data_[ID_L_LEG_END]->position_.coeff(dim,0);

//    Eigen::MatrixXd tra = robotis_framework::calcMinimumJerkTra(ini_value, 0.0, 0.0,
//                                                                tar_value, 0.0, 0.0,
//                                                                control_cycle_msec_, mov_time_);

//    goal_l_foot_tra_.block(0, dim, all_time_steps_, 1) = tra;
//  }

//  for (int dim=0; dim<3; dim++)
//  {
//    double ini_value = robotis_->thormang3_link_data_[ID_R_LEG_END]->position_.coeff(dim,0);
//    double tar_value = robotis_->thormang3_link_data_[ID_R_LEG_END]->position_.coeff(dim,0);

//    Eigen::MatrixXd tra = robotis_framework::calcMinimumJerkTra(ini_value, 0.0, 0.0,
//                                                                tar_value, 0.0, 0.0,
//                                                                control_cycle_msec_, mov_time_);

//    goal_r_foot_tra_.block(0, dim, all_time_steps_, 1) = tra;
//  }

//  /* target quaternion */
//  Eigen::Quaterniond pelvis_target_quaternion =
//      robotis_framework::convertRotationToQuaternion(robotis_->thormang3_link_data_[ID_PELVIS]->orientation_);

//  wb_pelvis_goal_quaternion_ = pelvis_target_quaternion;

//  double l_foot_target_roll = -14.0 * DEGREE2RADIAN;
//  double r_foot_target_roll = 14.0 * DEGREE2RADIAN;

//  if ( goal_kinematics_pose_msg_.name == "stand_wheel_pose_back" )
//  {
//    l_foot_target_roll = 0.0;
//    r_foot_target_roll = 0.0;
//  }

//  Eigen::Quaterniond l_foot_target_quaternion =
//      robotis_framework::convertRPYToQuaternion(l_foot_target_roll, 0.0, 0.0);

//  wb_l_foot_goal_quaternion_ = l_foot_target_quaternion;

//  Eigen::Quaterniond r_foot_target_quaternion =
//      robotis_framework::convertRPYToQuaternion(r_foot_target_roll, 0.0, 0.0);

//  wb_r_foot_goal_quaternion_ = r_foot_target_quaternion;

//  cnt_ = 0;
//  is_moving_ = true;
//  wb_ik_solving_ = true;

//  ROS_INFO("[start] send trajectory");
//}

void WholebodyModule::traGeneProcJointSpace()
{
  ROS_INFO("%s", goal_joint_pose_msg_.name.c_str() );
  ROS_INFO("%f", goal_joint_pose_msg_.value );

  if (goal_joint_pose_msg_.time <= 0.0)
  {
    /* set movement time */
    double tol = 30 * DEGREE2RADIAN; // rad per sec
    double mov_time = 1.5;

    int joint_id = joint_name_to_id_[goal_joint_pose_msg_.name];

    double ini_value = goal_joint_position_(joint_id);
    double tar_value = goal_joint_pose_msg_.value;
    double diff = fabs(tar_value - ini_value);

    mov_time_ =  diff / tol;
    int all_time_steps = int(floor((mov_time_ / control_cycle_sec_) + 1));
    mov_time_ = double (all_time_steps-1) * control_cycle_sec_;

    if (mov_time_ < mov_time)
      mov_time_= mov_time;
  }
  else
  {
    mov_time_ = goal_joint_pose_msg_.time;
    int all_time_steps = int(floor((mov_time_/control_cycle_sec_) + 1 ));
    mov_time_ = double (all_time_steps - 1) * control_cycle_sec_;
  }

  all_time_steps_ = int(mov_time_ / control_cycle_sec_) + 1;
  goal_joint_tra_.resize(all_time_steps_, MAX_JOINT_ID + 1);

  /* calculate joint trajectory */
  for (int id = 1; id <= MAX_JOINT_ID; id++)
  {
    double ini_value = goal_joint_position_(id);
    double tar_value = goal_joint_position_(id);

    if(robotis_->thormang3_link_data_[id]->name_ == goal_joint_pose_msg_.name)
      tar_value = goal_joint_pose_msg_.value;

    Eigen::MatrixXd tra =
        robotis_framework::calcMinimumJerkTra(ini_value, 0.0, 0.0,
                                              tar_value , 0.0 , 0.0 ,
                                              control_cycle_sec_, mov_time_);

    goal_joint_tra_.block(0, id, all_time_steps_, 1) = tra;
  }

  cnt_ = 0;
  is_balancing_ = false;
  is_moving_ = true;

  ROS_INFO("[start] send trajectory");
}

//void WholebodyModule::traGeneProcForTaskSpace()
//{
//  if (goal_kinematics_pose_msg_.time <= 0.0)
//  {
//    /* set movement time */
//    double tol = 0.1; // m per sec
//    double mov_time = 2.0;

//    double diff = sqrt(pow(robotis_->thormang3_link_data_[ik_id_end_]->position_.coeff(0,0) - goal_kinematics_pose_msg_.pelvis_pose.position.x, 2) +
//                       pow(robotis_->thormang3_link_data_[ik_id_end_]->position_.coeff(1,0) - goal_kinematics_pose_msg_.pelvis_pose.position.y, 2) +
//                       pow(robotis_->thormang3_link_data_[ik_id_end_]->position_.coeff(2,0) - goal_kinematics_pose_msg_.pelvis_pose.position.z, 2));

//    mov_time_ = diff / tol;
//    int all_time_steps = int(floor((mov_time_/control_cycle_msec_) + 1 ));
//    mov_time_ = double (all_time_steps - 1) * control_cycle_msec_;

//    if ( mov_time_ < mov_time )
//      mov_time_ = mov_time;
//  }
//  else
//  {
//    mov_time_ = goal_kinematics_pose_msg_.time;
//    int all_time_steps = int(floor((mov_time_/control_cycle_msec_) + 1 ));
//    mov_time_ = double (all_time_steps - 1) * control_cycle_msec_;
//  }

//  all_time_steps_ = int(mov_time_/control_cycle_msec_) + 1;
//  goal_task_tra_.resize(all_time_steps_, 3);

//  /* calculate trajectory */
//  for ( int dim = 0; dim < 3; dim++ )
//  {
//    double ini_value = robotis_->thormang3_link_data_[ik_id_end_]->position_.coeff(dim,0);
//    double tar_value;
//    if ( dim == 0 )
//      tar_value = goal_kinematics_pose_msg_.pelvis_pose.position.x ;
//    else if ( dim == 1 )
//      tar_value = goal_kinematics_pose_msg_.pelvis_pose.position.y ;
//    else if ( dim == 2 )
//      tar_value = goal_kinematics_pose_msg_.pelvis_pose.position.z ;

//    Eigen::MatrixXd tra = robotis_framework::calcMinimumJerkTra(ini_value, 0.0, 0.0,
//                                                                tar_value, 0.0, 0.0,
//                                                                control_cycle_msec_, mov_time_);

//    goal_task_tra_.block(0, dim, all_time_steps_, 1) = tra;
//  }

//  /* target quaternion */
//  Eigen::Quaterniond target_quaternion(goal_kinematics_pose_msg_.pelvis_pose.orientation.w,
//                                       goal_kinematics_pose_msg_.pelvis_pose.orientation.x,
//                                       goal_kinematics_pose_msg_.pelvis_pose.orientation.y,
//                                       goal_kinematics_pose_msg_.pelvis_pose.orientation.z);

//  ik_goal_quaternion_ = target_quaternion;

//  cnt_ = 0;
//  is_moving_ = true;
//  ik_solving_ = true;

//  ROS_INFO("[start] send trajectory");
//}

void WholebodyModule::traGeneProcPelvis()
{
  mov_time_ = goal_kinematics_pose_msg_.mov_time;
  int all_time_steps = int(floor((mov_time_/control_cycle_sec_) + 1 ));
  mov_time_ = double (all_time_steps - 1) * control_cycle_sec_;

  all_time_steps_ = int(mov_time_/control_cycle_sec_) + 1;
  goal_pelvis_tra_.resize(all_time_steps_, 3);
  goal_l_foot_tra_.resize(all_time_steps_, 3);
  goal_r_foot_tra_.resize(all_time_steps_, 3);

  Eigen::MatrixXd pelvis_tar_value = Eigen::MatrixXd::Zero(3,1);
  pelvis_tar_value.coeffRef(0,0) = goal_kinematics_pose_msg_.pelvis_pose.position.x;
  pelvis_tar_value.coeffRef(1,0) = goal_kinematics_pose_msg_.pelvis_pose.position.y;
  pelvis_tar_value.coeffRef(2,0) = goal_kinematics_pose_msg_.pelvis_pose.position.z;

  /* calculate trajectory */
  for (int dim=0; dim<3; dim++)
  {
    double ini_value = robotis_->thormang3_link_data_[ID_PELVIS]->position_.coeff(dim,0);
    double tar_value = pelvis_tar_value.coeff(dim,0);

    Eigen::MatrixXd tra = robotis_framework::calcMinimumJerkTra(ini_value, 0.0, 0.0,
                                                                tar_value, 0.0, 0.0,
                                                                control_cycle_sec_, mov_time_);

    goal_pelvis_tra_.block(0, dim, all_time_steps_, 1) = tra;
  }

  /* target quaternion */
  Eigen::Quaterniond pelvis_target_quaternion(goal_kinematics_pose_msg_.pelvis_pose.orientation.w,
                                              goal_kinematics_pose_msg_.pelvis_pose.orientation.x,
                                              goal_kinematics_pose_msg_.pelvis_pose.orientation.y,
                                              goal_kinematics_pose_msg_.pelvis_pose.orientation.z);

  wb_pelvis_goal_quaternion_ = pelvis_target_quaternion;

  calcGoalTraLeg();

  cnt_ = 0;
  is_moving_ = true;
  wb_ik_solving_ = true;

  ROS_INFO("[start] send trajectory");
}

void WholebodyModule::traGeneProcWholebody()
{
  mov_time_ = goal_kinematics_pose_msg_.mov_time;
  int all_time_steps = int(floor((mov_time_/control_cycle_sec_) + 1 ));
  mov_time_ = double (all_time_steps - 1) * control_cycle_sec_;

  all_time_steps_ = int(mov_time_/control_cycle_sec_) + 1;
  goal_pelvis_tra_.resize(all_time_steps_, 3);
  goal_l_foot_tra_.resize(all_time_steps_, 3);
  goal_r_foot_tra_.resize(all_time_steps_, 3);
  goal_l_arm_tra_.resize(all_time_steps_, 3);
  goal_r_arm_tra_.resize(all_time_steps_, 3);

  if (wb_l_arm_planning_ == true)
  {
    Eigen::MatrixXd l_arm_tar_value = Eigen::MatrixXd::Zero(3,1);
    l_arm_tar_value.coeffRef(0,0) = goal_kinematics_pose_msg_.l_arm_pose.position.x;
    l_arm_tar_value.coeffRef(1,0) = goal_kinematics_pose_msg_.l_arm_pose.position.y;
    l_arm_tar_value.coeffRef(2,0) = goal_kinematics_pose_msg_.l_arm_pose.position.z;

    /* calculate trajectory */
    for (int dim=0; dim<3; dim++)
    {
      double ini_value = robotis_->thormang3_link_data_[ID_L_ARM_END]->position_.coeff(dim,0);
      double tar_value = l_arm_tar_value.coeff(dim,0);

      wb_arm_diff_position_(dim) = tar_value - ini_value;

      Eigen::MatrixXd tra = robotis_framework::calcMinimumJerkTra(ini_value, 0.0, 0.0,
                                                                  tar_value, 0.0, 0.0,
                                                                  control_cycle_sec_, mov_time_);

      goal_l_arm_tra_.block(0, dim, all_time_steps_, 1) = tra;
    }

    /* target quaternion */
    Eigen::Quaterniond l_arm_target_quaternion(goal_kinematics_pose_msg_.l_arm_pose.orientation.w,
                                               goal_kinematics_pose_msg_.l_arm_pose.orientation.x,
                                               goal_kinematics_pose_msg_.l_arm_pose.orientation.y,
                                               goal_kinematics_pose_msg_.l_arm_pose.orientation.z);

    wb_l_arm_goal_quaternion_ = l_arm_target_quaternion;
  }
  else if (wb_r_arm_planning_ == true)
  {
    Eigen::MatrixXd r_arm_tar_value = Eigen::MatrixXd::Zero(3,1);
    r_arm_tar_value.coeffRef(0,0) = goal_kinematics_pose_msg_.r_arm_pose.position.x;
    r_arm_tar_value.coeffRef(1,0) = goal_kinematics_pose_msg_.r_arm_pose.position.y;
    r_arm_tar_value.coeffRef(2,0) = goal_kinematics_pose_msg_.r_arm_pose.position.z;

    /* calculate trajectory */
    for (int dim=0; dim<3; dim++)
    {
      double ini_value = robotis_->thormang3_link_data_[ID_R_ARM_END]->position_.coeff(dim,0);
      double tar_value = r_arm_tar_value.coeff(dim,0);

      wb_arm_diff_position_(dim) = tar_value - ini_value;

      Eigen::MatrixXd tra = robotis_framework::calcMinimumJerkTra(ini_value, 0.0, 0.0,
                                                                  tar_value, 0.0, 0.0,
                                                                  control_cycle_sec_, mov_time_);

      goal_r_arm_tra_.block(0, dim, all_time_steps_, 1) = tra;
    }

    /* target quaternion */
    Eigen::Quaterniond r_arm_target_quaternion(goal_kinematics_pose_msg_.r_arm_pose.orientation.w,
                                               goal_kinematics_pose_msg_.r_arm_pose.orientation.x,
                                               goal_kinematics_pose_msg_.r_arm_pose.orientation.y,
                                               goal_kinematics_pose_msg_.r_arm_pose.orientation.z);

    wb_r_arm_goal_quaternion_ = r_arm_target_quaternion;
  }

  calcGoalTraPelvis();
  calcGoalTraLeg();

  cnt_ = 0;
  is_moving_ = true;
  wb_ik_solving_ = true;

  ROS_INFO("[start] send trajectory");
}

void WholebodyModule::calcGoalTraPelvis()
{
  Eigen::MatrixXd wb_pelvis_diff_constant = Eigen::MatrixXd::Zero(3,1);
  wb_pelvis_diff_constant.coeffRef(0,0) = wb_pelvis_diff_x_constant_;
  wb_pelvis_diff_constant.coeffRef(1,0) = wb_pelvis_diff_y_constant_;
  wb_pelvis_diff_constant.coeffRef(2,0) = wb_pelvis_diff_z_constant_;

  Eigen::MatrixXd wb_pelvis_limit = Eigen::MatrixXd::Zero(3,2);
  wb_pelvis_limit.coeffRef(0,0) = 0.05;
  wb_pelvis_limit.coeffRef(0,1) = -0.05;
  wb_pelvis_limit.coeffRef(1,0) = 0.05;
  wb_pelvis_limit.coeffRef(1,1) = -0.05;
  wb_pelvis_limit.coeffRef(2,0) = 0.75;
  wb_pelvis_limit.coeffRef(2,1) = 0.3;

  Eigen::MatrixXd wb_pelvis_tar_value = Eigen::MatrixXd::Zero(3,1);
  for (int dim=0; dim <3; dim++)
  {
    wb_pelvis_tar_value.coeffRef(dim,0) =
        robotis_->thormang3_link_data_[ID_PELVIS]->position_.coeff(dim,0) + wb_pelvis_diff_constant.coeff(dim,0) * wb_arm_diff_position_(dim);

    if (wb_pelvis_tar_value.coeff(dim,0) > wb_pelvis_limit.coeff(dim,0))
      wb_pelvis_tar_value.coeffRef(dim,0) = wb_pelvis_limit.coeff(dim,0);
    else if (wb_pelvis_tar_value.coeff(dim,0) < wb_pelvis_limit.coeff(dim,1))
      wb_pelvis_tar_value.coeffRef(dim,0) = wb_pelvis_limit.coeff(dim,1);
  }

  for (int dim=0; dim<3; dim++)
  {
    double ini_value = robotis_->thormang3_link_data_[ID_PELVIS]->position_.coeff(dim,0);
    double tar_value = wb_pelvis_tar_value.coeff(dim,0);

    Eigen::MatrixXd tra = robotis_framework::calcMinimumJerkTra(ini_value, 0.0, 0.0,
                                                                tar_value, 0.0, 0.0,
                                                                control_cycle_sec_, mov_time_);

    goal_pelvis_tra_.block(0, dim, all_time_steps_, 1) = tra;
  }

//  Eigen::Quaterniond pelvis_target_quaternion =
//      robotis_framework::convertRotationToQuaternion(robotis_->thormang3_link_data_[ID_PELVIS]->orientation_);

//  wb_pelvis_goal_quaternion_ = pelvis_target_quaternion;
}

void WholebodyModule::calcGoalTraLeg()
{
  for (int dim=0; dim<3; dim++)
  {
    double ini_value = wb_l_foot_default_position_.coeff(dim,0);
    double tar_value = wb_l_foot_default_position_.coeff(dim,0);

    Eigen::MatrixXd tra = robotis_framework::calcMinimumJerkTra(ini_value, 0.0, 0.0,
                                                                tar_value, 0.0, 0.0,
                                                                control_cycle_sec_, mov_time_);

    goal_l_foot_tra_.block(0, dim, all_time_steps_, 1) = tra;
  }

  for (int dim=0; dim<3; dim++)
  {
    double ini_value = wb_r_foot_default_position_.coeff(dim,0);
    double tar_value = wb_r_foot_default_position_.coeff(dim,0);

    Eigen::MatrixXd tra = robotis_framework::calcMinimumJerkTra(ini_value, 0.0, 0.0,
                                                                tar_value, 0.0, 0.0,
                                                                control_cycle_sec_, mov_time_);

    goal_r_foot_tra_.block(0, dim, all_time_steps_, 1) = tra;
  }

  wb_l_foot_goal_quaternion_ = wb_l_foot_default_quaternion_;
  wb_r_foot_goal_quaternion_ = wb_r_foot_default_quaternion_;
}

void WholebodyModule::calcGoalFT()
{
  double l_foot_distance = fabs(0.093 - center_of_mass_.coeff(1,0)) + default_center_of_mass_.coeff(1,0);
  double r_foot_distance = fabs(-0.093 - center_of_mass_.coeff(1,0)) - default_center_of_mass_.coeff(1,0);

  balance_goal_l_foot_ft_ = -420.0 * r_foot_distance / 0.186;
  balance_goal_r_foot_ft_ = -420.0 * l_foot_distance / 0.186;

//  ROS_INFO("goal_l_foot_ft : %f", balance_goal_l_foot_ft_);
//  ROS_INFO("goal_r_foot_ft : %f", balance_goal_r_foot_ft_);
}

bool WholebodyModule::getJointPoseCallback(thormang3_wholebody_module_msgs::GetJointPose::Request &req,
                                           thormang3_wholebody_module_msgs::GetJointPose::Response &res)
{
  if(enable_ == false)
    return false;

  for (int id=1; id<=MAX_JOINT_ID; id++)
  {
    if(robotis_->thormang3_link_data_[id]->name_ == req.joint_name)
    {
      res.joint_value = goal_joint_position_(id);
      return true;
    }
  }

  return false;
}

bool WholebodyModule::getKinematicsPoseCallback(thormang3_wholebody_module_msgs::GetKinematicsPose::Request &req,
                                                thormang3_wholebody_module_msgs::GetKinematicsPose::Response &res)
{
  if(enable_ == false)
    return false;

  int end_index;

  if (req.group_name == "left_arm_wholebody")
    end_index = ID_L_ARM_END;
  else if (req.group_name == "right_arm_wholebody")
    end_index = ID_R_ARM_END;
  else if (req.group_name == "pelvis")
    end_index = ID_PELVIS;
  else
    return false;

  res.group_pose.position.x = robotis_->thormang3_link_data_[end_index ]->position_.coeff(0,0);
  res.group_pose.position.y = robotis_->thormang3_link_data_[end_index ]->position_.coeff(1,0);
  res.group_pose.position.z = robotis_->thormang3_link_data_[end_index ]->position_.coeff(2,0);

  Eigen::Quaterniond quaternion = robotis_framework::convertRotationToQuaternion(robotis_->thormang3_link_data_[end_index]->orientation_);

  res.group_pose.orientation.w = quaternion.w();
  res.group_pose.orientation.x = quaternion.x();
  res.group_pose.orientation.y = quaternion.y();
  res.group_pose.orientation.z = quaternion.z();

  return true;
}

//void WholebodyModule::setInverseKinematics(int cnt)
//{
//  for (int dim=0; dim<3; dim++)
//    ik_target_position_.coeffRef(dim, 0) = goal_task_tra_.coeff(cnt, dim);

//  double time_step = ( double ) cnt / ( double ) all_time_steps_;
//  Eigen::Quaterniond quaternion = ik_start_quaternion_.slerp(time_step, ik_goal_quaternion_);

//  ik_target_rotation_ = robotis_framework::convertQuaternionToRotation(quaternion);
//}

void WholebodyModule::setPelvisPose(int cnt)
{
  for ( int dim = 0; dim < 3; dim++ )
    wb_pelvis_target_position_.coeffRef(dim, 0) = goal_pelvis_tra_.coeff(cnt, dim);

  double time_step = ( double ) cnt / ( double ) all_time_steps_;
  Eigen::Quaterniond quaternion = wb_pelvis_start_quaternion_.slerp(time_step, wb_pelvis_goal_quaternion_);

  wb_pelvis_target_rotation_ = robotis_framework::convertQuaternionToRotation(quaternion);
}

void WholebodyModule::setInverseKinematicsLeftFoot(int cnt)
{
  for (int dim=0; dim<3; dim++)
    wb_l_foot_target_position_.coeffRef(dim, 0) = goal_l_foot_tra_.coeff(cnt, dim);

  double time_step = ( double ) cnt / ( double ) all_time_steps_;
  Eigen::Quaterniond quaternion = wb_l_foot_start_quaternion_.slerp(time_step, wb_l_foot_goal_quaternion_);

  wb_l_foot_target_rotation_ = robotis_framework::convertQuaternionToRotation(quaternion);
}

void WholebodyModule::setInverseKinematicsRightFoot(int cnt)
{
  for (int dim=0; dim<3; dim++)
    wb_r_foot_target_position_.coeffRef(dim, 0) = goal_r_foot_tra_.coeff(cnt, dim);

  double time_step = ( double ) cnt / ( double ) all_time_steps_;
  Eigen::Quaterniond quaternion = wb_r_foot_start_quaternion_.slerp(time_step, wb_r_foot_goal_quaternion_);

  wb_r_foot_target_rotation_ = robotis_framework::convertQuaternionToRotation(quaternion);
}

void WholebodyModule::setInverseKinematicsLeftArm(int cnt)
{
  for (int dim=0; dim<3; dim++)
    wb_l_arm_target_position_.coeffRef(dim, 0) = goal_l_arm_tra_.coeff(cnt, dim);

  double time_step = ( double ) cnt / ( double ) all_time_steps_;
  Eigen::Quaterniond quaternion = wb_l_arm_start_quaternion_.slerp(time_step, wb_l_arm_goal_quaternion_);

  wb_l_arm_target_rotation_ = robotis_framework::convertQuaternionToRotation(quaternion);
}

void WholebodyModule::setInverseKinematicsRightArm(int cnt)
{
  for (int dim=0; dim<3; dim++)
    wb_r_arm_target_position_.coeffRef(dim, 0) = goal_r_arm_tra_.coeff(cnt, dim);

  double time_step = ( double ) cnt / ( double ) all_time_steps_;
  Eigen::Quaterniond quaternion = wb_r_arm_start_quaternion_.slerp(time_step, wb_r_arm_goal_quaternion_);

  wb_r_arm_target_rotation_ = robotis_framework::convertQuaternionToRotation(quaternion);
}

void WholebodyModule::setStartTrajectory()
{
  publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Start Trajectory");

//  if (ik_solving_ == true)
//  {
//    Eigen::MatrixXd ik_start_rotation = robotis_->thormang3_link_data_[ik_id_end_]->orientation_;
//    ik_start_quaternion_ = robotis_framework::convertRotationToQuaternion(ik_start_rotation);
//  }
//  else
  if (wb_ik_solving_ == true)
  {
    Eigen::MatrixXd wb_start_rotation = robotis_->thormang3_link_data_[ID_PELVIS]->orientation_;
    wb_pelvis_start_quaternion_ = robotis_framework::convertRotationToQuaternion(wb_start_rotation);

    wb_l_foot_start_quaternion_ = wb_l_foot_default_quaternion_;
    wb_r_foot_start_quaternion_ = wb_r_foot_default_quaternion_;

    Eigen::MatrixXd wb_l_arm_start_rotation = robotis_->thormang3_link_data_[ID_L_ARM_END]->orientation_;
    wb_l_arm_start_quaternion_ = robotis_framework::convertRotationToQuaternion(wb_l_arm_start_rotation);

    Eigen::MatrixXd wb_r_arm_start_rotation = robotis_->thormang3_link_data_[ID_R_ARM_END]->orientation_;
    wb_r_arm_start_quaternion_ = robotis_framework::convertRotationToQuaternion(wb_r_arm_start_rotation);
  }
}

void WholebodyModule::setEndTrajectory()
{
  if (is_moving_ == true)
  {
    if (cnt_ >= all_time_steps_)
    {
      ROS_INFO("[end] send trajectory");

      publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "End Trajectory");

      is_moving_ = false;

//      ik_solving_ = false;
      wb_ik_solving_ = false;

      wb_l_arm_planning_ = false;
      wb_r_arm_planning_ = false;

      cnt_ = 0;

      wb_pelvis_target_position_ = robotis_->thormang3_link_data_[ID_PELVIS]->position_;
      wb_pelvis_target_rotation_ = robotis_->thormang3_link_data_[ID_PELVIS]->orientation_;

      wb_l_foot_target_position_ = wb_l_foot_default_position_;
      wb_l_foot_target_rotation_ = robotis_framework::convertQuaternionToRotation(wb_l_foot_default_quaternion_);
      wb_r_foot_target_position_ = wb_r_foot_default_position_;
      wb_r_foot_target_rotation_ = robotis_framework::convertQuaternionToRotation(wb_r_foot_default_quaternion_);
    }
  }
}

void WholebodyModule::setBalanceControlGain(int cnt)
{
  double gain_ratio;
  double max_pelvis = 0.723;
  double min_pelvis = 0.3;

  if (robotis_->thormang3_link_data_[ID_PELVIS]->position_.coeff(2,0) > max_pelvis)
    gain_ratio = 1.0;
  else if (robotis_->thormang3_link_data_[ID_PELVIS]->position_.coeff(2,0) < min_pelvis)
    gain_ratio = 0.0;
  else
    gain_ratio = (robotis_->thormang3_link_data_[ID_PELVIS]->position_.coeff(2,0) - min_pelvis) / (max_pelvis - min_pelvis);

  double constant;
  if (gazebo_ == true)
    constant = 0.0;
  else
    constant = 1.0;

  double gyro_gain = gyro_gain_ * gain_ratio * constant;

  balance_control_.setGyroBalanceGainRatio(gyro_gain * constant);

  balance_control_.foot_roll_angle_ctrl_.gain_ = foot_roll_angle_gain_ * gain_ratio * constant;
  balance_control_.foot_pitch_angle_ctrl_.gain_ = foot_pitch_angle_gain_ * gain_ratio * constant;

  balance_control_.left_foot_force_x_ctrl_.gain_ = left_foot_force_x_gain_ * gain_ratio * constant;
  balance_control_.left_foot_force_y_ctrl_.gain_ = left_foot_force_y_gain_ * gain_ratio * constant;

  balance_control_.right_foot_force_x_ctrl_.gain_ = right_foot_force_x_gain_ * gain_ratio * constant;
  balance_control_.right_foot_force_y_ctrl_.gain_ = right_foot_force_y_gain_ * gain_ratio * constant;

  balance_control_.foot_force_z_diff_ctrl_.gain_ = foot_force_z_gain_ * gain_ratio * constant;

  balance_control_.right_foot_torque_roll_ctrl_.gain_ = left_foot_torque_roll_gain_ * gain_ratio * constant;
  balance_control_.right_foot_torque_pitch_ctrl_.gain_ = left_foot_torque_pitch_gain_ * gain_ratio * constant;

  balance_control_.left_foot_torque_roll_ctrl_.gain_ = right_foot_torque_roll_gain_ * gain_ratio * constant;
  balance_control_.left_foot_torque_pitch_ctrl_.gain_ = right_foot_torque_pitch_gain_ * gain_ratio * constant;

  balance_control_.foot_roll_angle_ctrl_.time_constant_sec_ = foot_roll_angle_time_constant_;
  balance_control_.foot_pitch_angle_ctrl_.time_constant_sec_ = foot_pitch_angle_time_constant_;

  balance_control_.left_foot_force_x_ctrl_.time_constant_sec_ = left_foot_force_x_time_constant_;
  balance_control_.left_foot_force_y_ctrl_.time_constant_sec_ = left_foot_force_y_time_constant_;

  balance_control_.right_foot_force_x_ctrl_.time_constant_sec_ = right_foot_force_x_time_constant_;
  balance_control_.right_foot_force_y_ctrl_.time_constant_sec_ = right_foot_force_y_time_constant_;

  balance_control_.foot_force_z_diff_ctrl_.time_constant_sec_ = foot_force_z_time_constant_;

  balance_control_.right_foot_torque_roll_ctrl_.time_constant_sec_ = left_foot_torque_roll_time_constant_;
  balance_control_.right_foot_torque_pitch_ctrl_.time_constant_sec_ = left_foot_torque_pitch_time_constant_;

  balance_control_.left_foot_torque_roll_ctrl_.time_constant_sec_ = right_foot_torque_roll_time_constant_;
  balance_control_.left_foot_torque_pitch_ctrl_.time_constant_sec_ = right_foot_torque_pitch_time_constant_;

  int balance_cnt;

  if (on_balance_gain_ == true)
  {
    if (cnt >= balance_gain_time_steps_)
      balance_cnt = balance_gain_time_steps_-1;
    else
      balance_cnt = cnt;

    balance_control_.setGyroBalanceGainRatio(gyro_gain * on_balance_gain_tra_.coeff(balance_cnt,0));

    balance_control_.foot_roll_angle_ctrl_.gain_ *= on_balance_gain_tra_.coeff(balance_cnt,0);
    balance_control_.foot_pitch_angle_ctrl_.gain_ *= on_balance_gain_tra_.coeff(balance_cnt,0);

    balance_control_.left_foot_force_x_ctrl_.gain_ *= on_balance_gain_tra_.coeff(balance_cnt,0);
    balance_control_.left_foot_force_y_ctrl_.gain_ *= on_balance_gain_tra_.coeff(balance_cnt,0);

    balance_control_.right_foot_force_x_ctrl_.gain_ *= on_balance_gain_tra_.coeff(balance_cnt,0);
    balance_control_.right_foot_force_y_ctrl_.gain_ *= on_balance_gain_tra_.coeff(balance_cnt,0);

    balance_control_.foot_force_z_diff_ctrl_.gain_ *= on_balance_gain_tra_.coeff(balance_cnt,0);

    balance_control_.right_foot_torque_roll_ctrl_.gain_ *= on_balance_gain_tra_.coeff(balance_cnt,0);
    balance_control_.right_foot_torque_pitch_ctrl_.gain_ *= on_balance_gain_tra_.coeff(balance_cnt,0);

    balance_control_.left_foot_torque_roll_ctrl_.gain_ *= on_balance_gain_tra_.coeff(balance_cnt,0);
    balance_control_.left_foot_torque_pitch_ctrl_.gain_ *= on_balance_gain_tra_.coeff(balance_cnt,0);
  }
  else
  {
    if (cnt >= balance_gain_time_steps_)
      balance_cnt = balance_gain_time_steps_-1;
    else
      balance_cnt = cnt;

    balance_control_.setGyroBalanceGainRatio(gyro_gain * off_balance_gain_tra_.coeff(balance_cnt,0));

    balance_control_.foot_roll_angle_ctrl_.gain_ *= off_balance_gain_tra_.coeff(balance_cnt,0);
    balance_control_.foot_pitch_angle_ctrl_.gain_ *= off_balance_gain_tra_.coeff(balance_cnt,0);

    balance_control_.left_foot_force_x_ctrl_.gain_ *= off_balance_gain_tra_.coeff(balance_cnt,0);
    balance_control_.left_foot_force_y_ctrl_.gain_ *= off_balance_gain_tra_.coeff(balance_cnt,0);

    balance_control_.right_foot_force_x_ctrl_.gain_ *= off_balance_gain_tra_.coeff(balance_cnt,0);
    balance_control_.right_foot_force_y_ctrl_.gain_ *= off_balance_gain_tra_.coeff(balance_cnt,0);

    balance_control_.foot_force_z_diff_ctrl_.gain_ *= off_balance_gain_tra_.coeff(balance_cnt,0);

    balance_control_.right_foot_torque_roll_ctrl_.gain_ *= off_balance_gain_tra_.coeff(balance_cnt,0);
    balance_control_.right_foot_torque_pitch_ctrl_.gain_ *= off_balance_gain_tra_.coeff(balance_cnt,0);

    balance_control_.left_foot_torque_roll_ctrl_.gain_ *= off_balance_gain_tra_.coeff(balance_cnt,0);
    balance_control_.left_foot_torque_pitch_ctrl_.gain_ *= off_balance_gain_tra_.coeff(balance_cnt,0);
  }
}

//void WholebodyModule::solveInverseKinematics()
//{
//  setInverseKinematics(cnt_);

//  int max_iter = 70;
//  double ik_tol = 1e-5;
//  bool ik_success = robotis_->calcInverseKinematics(ik_id_start_, ik_id_end_,
//                                                    ik_target_position_, ik_target_rotation_,
//                                                    max_iter, ik_tol, ik_weight_);

//  if (ik_success == true)
//  {
//    for (int id=1; id<=MAX_JOINT_ID; id++)
//      goal_joint_position_(id) = robotis_->thormang3_link_data_[id]->joint_angle_;
//  }
//  else
//  {
//    ROS_INFO("----- ik failed -----");
//    ROS_INFO("[end] send trajectory");

//    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Inverse Kinematics Failed");

//    is_moving_ = false;
//    ik_solving_ = false;
//    cnt_ = 0;
//  }
//}

void WholebodyModule::solveWholebodyInverseKinematics()
{
  /* ----- */

  balance_control_.setGyroBalanceEnable(true);
  balance_control_.setOrientationBalanceEnable(true);
  balance_control_.setForceTorqueBalanceEnable(true);

  Eigen::MatrixXd pelvis_pose = Eigen::MatrixXd::Identity(4,4);
  pelvis_pose.block(0,0,3,3) = wb_pelvis_target_rotation_;
  pelvis_pose.block(0,3,3,1) = wb_pelvis_target_position_;

  Eigen::MatrixXd l_foot_pose = Eigen::MatrixXd::Identity(4,4);
  l_foot_pose.block(0,0,3,3) = wb_l_foot_target_rotation_;
  l_foot_pose.block(0,3,3,1) = wb_l_foot_target_position_;

  Eigen::MatrixXd r_foot_pose = Eigen::MatrixXd::Identity(4,4);
  r_foot_pose.block(0,0,3,3) = wb_r_foot_target_rotation_;
  r_foot_pose.block(0,3,3,1) = wb_r_foot_target_position_;

  balance_control_.setDesiredPose(pelvis_pose, r_foot_pose, l_foot_pose);

  balance_control_.setCurrentGyroSensorOutput(imu_data_msg_.angular_velocity.x, imu_data_msg_.angular_velocity.y);

  Eigen::Quaterniond imu_quaternion(imu_data_msg_.orientation.w,
                                    imu_data_msg_.orientation.x,
                                    imu_data_msg_.orientation.y,
                                    imu_data_msg_.orientation.z);
  Eigen::MatrixXd imu_rpy =
      robotis_framework::convertRotationToRPY(robotis_framework::getRotationX(M_PI) * imu_quaternion.toRotationMatrix() * robotis_framework::getRotationZ(M_PI));

  Eigen::MatrixXd g_to_r_foot_force =
    robotis_->thormang3_link_data_[ID_R_LEG_FT]->orientation_ * robotis_framework::getRotationX(M_PI) *
    robotis_framework::getTransitionXYZ(r_foot_ft_data_msg_.force.x, r_foot_ft_data_msg_.force.y, r_foot_ft_data_msg_.force.z);

  Eigen::MatrixXd g_to_r_foot_torque =
    robotis_->thormang3_link_data_[ID_R_LEG_FT]->orientation_ * robotis_framework::getRotationX(M_PI) *
    robotis_framework::getTransitionXYZ(r_foot_ft_data_msg_.torque.x, r_foot_ft_data_msg_.torque.y, r_foot_ft_data_msg_.torque.z);

  Eigen::MatrixXd g_to_l_foot_force =
    robotis_->thormang3_link_data_[ID_L_LEG_FT]->orientation_ * robotis_framework::getRotationX(M_PI) *
    robotis_framework::getTransitionXYZ(l_foot_ft_data_msg_.force.x, l_foot_ft_data_msg_.force.y, l_foot_ft_data_msg_.force.z);

  Eigen::MatrixXd g_to_l_foot_torque =
    robotis_->thormang3_link_data_[ID_L_LEG_FT]->orientation_ * robotis_framework::getRotationX(M_PI) *
    robotis_framework::getTransitionXYZ(l_foot_ft_data_msg_.torque.x, l_foot_ft_data_msg_.torque.y, l_foot_ft_data_msg_.torque.z);

  balance_control_.setCurrentOrientationSensorOutput(imu_rpy.coeff(0,0), imu_rpy.coeff(1,0));
  balance_control_.setCurrentFootForceTorqueSensorOutput(g_to_r_foot_force.coeff(0,0),  g_to_r_foot_force.coeff(1,0),  g_to_r_foot_force.coeff(2,0),
                                                         g_to_r_foot_torque.coeff(0,0), g_to_r_foot_torque.coeff(1,0), g_to_r_foot_torque.coeff(2,0),
                                                         g_to_l_foot_force.coeff(0,0),  g_to_l_foot_force.coeff(1,0),  g_to_l_foot_force.coeff(2,0),
                                                         g_to_l_foot_torque.coeff(0,0), g_to_l_foot_torque.coeff(1,0), g_to_l_foot_torque.coeff(2,0));

  balance_control_.setDesiredCOBGyro(0.0, 0.0);
  balance_control_.setDesiredCOBOrientation(robotis_->thormang3_link_data_[ID_PELVIS_ROT_X]->joint_angle_,
                                            robotis_->thormang3_link_data_[ID_PELVIS_ROT_Y]->joint_angle_);
  balance_control_.setDesiredFootForceTorque(0.0, 0.0, balance_goal_r_foot_ft_, 0.0, 0.0, 0.0, 0.0, 0.0, balance_goal_l_foot_ft_, 0.0, 0.0, 0.0);

  int error;
  balance_control_.process(&error, &pelvis_pose, &r_foot_pose, &l_foot_pose);

  Eigen::MatrixXd wb_pelvis_target_rotation = pelvis_pose.block(0,0,3,3);
  Eigen::MatrixXd wb_pelvis_target_position = pelvis_pose.block(0,3,3,1);

  Eigen::MatrixXd wb_l_foot_target_rotation = l_foot_pose.block(0,0,3,3);
  Eigen::MatrixXd wb_l_foot_target_position = l_foot_pose.block(0,3,3,1);
  Eigen::MatrixXd wb_r_foot_target_rotation = r_foot_pose.block(0,0,3,3);
  Eigen::MatrixXd wb_r_foot_target_position = r_foot_pose.block(0,3,3,1);

  /* ----- */

  robotis_->thormang3_link_data_[ID_PELVIS_POS_X]->relative_position_.coeffRef(0,0) = wb_pelvis_target_position.coeff(0,0);
  robotis_->thormang3_link_data_[ID_PELVIS_POS_Y]->relative_position_.coeffRef(1,0) = wb_pelvis_target_position.coeff(1,0);
  robotis_->thormang3_link_data_[ID_PELVIS_POS_Z]->relative_position_.coeffRef(2,0) = wb_pelvis_target_position.coeff(2,0);

  Eigen::MatrixXd wb_target_rpy = robotis_framework::convertRotationToRPY(wb_pelvis_target_rotation);

  robotis_->thormang3_link_data_[ID_PELVIS_ROT_X]->joint_angle_ = wb_target_rpy.coeff(0,0);
  robotis_->thormang3_link_data_[ID_PELVIS_ROT_Y]->joint_angle_ = wb_target_rpy.coeff(1,0);
  robotis_->thormang3_link_data_[ID_PELVIS_ROT_Z]->joint_angle_ = wb_target_rpy.coeff(2,0);

  int max_iter = 70;
  double ik_tol = 1e-5;
  bool l_foot_ik_success = robotis_->calcInverseKinematics(ID_PELVIS, ID_L_LEG_END, wb_l_foot_target_position, wb_l_foot_target_rotation, max_iter, ik_tol);
  bool r_foot_ik_success = robotis_->calcInverseKinematics(ID_PELVIS, ID_R_LEG_END, wb_r_foot_target_position, wb_r_foot_target_rotation, max_iter, ik_tol);

  if (l_foot_ik_success == true && r_foot_ik_success == true)
  {
    for (int id=1; id<=MAX_JOINT_ID; id++)
      goal_joint_position_(id) = robotis_->thormang3_link_data_[id]->joint_angle_;
  }
  else
  {
    ROS_INFO("----- ik failed -----");
    ROS_INFO("[end] send trajectory");

    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "IK Failed");

    is_moving_ = false;
    wb_l_arm_planning_ = false;
    wb_r_arm_planning_ = false;
    cnt_ = 0;

    wb_pelvis_target_position_ = robotis_->thormang3_link_data_[ID_PELVIS]->position_;
    wb_pelvis_target_rotation_ = robotis_->thormang3_link_data_[ID_PELVIS]->orientation_;

    wb_l_foot_target_position_ = wb_l_foot_default_position_;
    wb_l_foot_target_rotation_ = robotis_framework::convertQuaternionToRotation(wb_l_foot_default_quaternion_);
    wb_r_foot_target_position_ = wb_r_foot_default_position_;
    wb_r_foot_target_rotation_ = robotis_framework::convertQuaternionToRotation(wb_r_foot_default_quaternion_);
  }
}

void WholebodyModule::solveWholebodyInverseKinematicsFull()
{
  robotis_->thormang3_link_data_[ID_PELVIS_POS_X]->relative_position_.coeffRef(0,0) = wb_pelvis_target_position_.coeff(0,0);
  robotis_->thormang3_link_data_[ID_PELVIS_POS_Y]->relative_position_.coeffRef(1,0) = wb_pelvis_target_position_.coeff(1,0);
  robotis_->thormang3_link_data_[ID_PELVIS_POS_Z]->relative_position_.coeffRef(2,0) = wb_pelvis_target_position_.coeff(2,0);

  int max_iter = 70;
  double ik_tol = 1e-5;

  bool l_arm_ik_success = false;
  bool r_arm_ik_success = false;

  if (wb_l_arm_planning_ == true)
  {
    l_arm_ik_success = robotis_->calcInverseKinematics(ID_BASE, ID_L_ARM_END, wb_l_arm_target_position_, wb_l_arm_target_rotation_, max_iter, ik_tol, ik_weight_);
    r_arm_ik_success = true;
  }
  else if (wb_r_arm_planning_ == true)
  {
    l_arm_ik_success = true;
    r_arm_ik_success = robotis_->calcInverseKinematics(ID_BASE, ID_R_ARM_END, wb_r_arm_target_position_, wb_r_arm_target_rotation_, max_iter, ik_tol, ik_weight_);
  }

//  robotis_->calcForwardKinematics(0);

  /* ----- */

  balance_control_.setGyroBalanceEnable(true);
  balance_control_.setOrientationBalanceEnable(true);
  balance_control_.setForceTorqueBalanceEnable(true);

  Eigen::MatrixXd pelvis_pose = Eigen::MatrixXd::Identity(4,4);
  pelvis_pose.block(0,0,3,3) = robotis_->thormang3_link_data_[ID_PELVIS]->orientation_;
  pelvis_pose.block(0,3,3,1) = wb_pelvis_target_position_;

  Eigen::MatrixXd l_foot_pose = Eigen::MatrixXd::Identity(4,4);
  l_foot_pose.block(0,0,3,3) = wb_l_foot_target_rotation_;
  l_foot_pose.block(0,3,3,1) = wb_l_foot_target_position_;

  Eigen::MatrixXd r_foot_pose = Eigen::MatrixXd::Identity(4,4);
  r_foot_pose.block(0,0,3,3) = wb_r_foot_target_rotation_;
  r_foot_pose.block(0,3,3,1) = wb_r_foot_target_position_;

  balance_control_.setDesiredPose(pelvis_pose, r_foot_pose, l_foot_pose);

  balance_control_.setCurrentGyroSensorOutput(imu_data_msg_.angular_velocity.x, imu_data_msg_.angular_velocity.y);

  Eigen::Quaterniond imu_quaternion(imu_data_msg_.orientation.w,
                                    imu_data_msg_.orientation.x,
                                    imu_data_msg_.orientation.y,
                                    imu_data_msg_.orientation.z);
  Eigen::MatrixXd imu_rpy =
      robotis_framework::convertRotationToRPY(robotis_framework::getRotationX(M_PI) * imu_quaternion.toRotationMatrix() * robotis_framework::getRotationZ(M_PI));

  Eigen::MatrixXd g_to_r_foot_force =
    robotis_->thormang3_link_data_[ID_R_LEG_FT]->orientation_ * robotis_framework::getRotationX(M_PI) *
    robotis_framework::getTransitionXYZ(r_foot_ft_data_msg_.force.x, r_foot_ft_data_msg_.force.y, r_foot_ft_data_msg_.force.z);

  Eigen::MatrixXd g_to_r_foot_torque =
    robotis_->thormang3_link_data_[ID_R_LEG_FT]->orientation_ * robotis_framework::getRotationX(M_PI) *
    robotis_framework::getTransitionXYZ(r_foot_ft_data_msg_.torque.x, r_foot_ft_data_msg_.torque.y, r_foot_ft_data_msg_.torque.z);

  Eigen::MatrixXd g_to_l_foot_force =
    robotis_->thormang3_link_data_[ID_L_LEG_FT]->orientation_ * robotis_framework::getRotationX(M_PI) *
    robotis_framework::getTransitionXYZ(l_foot_ft_data_msg_.force.x, l_foot_ft_data_msg_.force.y, l_foot_ft_data_msg_.force.z);

  Eigen::MatrixXd g_to_l_foot_torque =
    robotis_->thormang3_link_data_[ID_L_LEG_FT]->orientation_ * robotis_framework::getRotationX(M_PI) *
    robotis_framework::getTransitionXYZ(l_foot_ft_data_msg_.torque.x, l_foot_ft_data_msg_.torque.y, l_foot_ft_data_msg_.torque.z);

  balance_control_.setCurrentOrientationSensorOutput(imu_rpy.coeff(0,0), imu_rpy.coeff(1,0));
  balance_control_.setCurrentFootForceTorqueSensorOutput(g_to_r_foot_force.coeff(0,0),  g_to_r_foot_force.coeff(1,0),  g_to_r_foot_force.coeff(2,0),
                                                         g_to_r_foot_torque.coeff(0,0), g_to_r_foot_torque.coeff(1,0), g_to_r_foot_torque.coeff(2,0),
                                                         g_to_l_foot_force.coeff(0,0),  g_to_l_foot_force.coeff(1,0),  g_to_l_foot_force.coeff(2,0),
                                                         g_to_l_foot_torque.coeff(0,0), g_to_l_foot_torque.coeff(1,0), g_to_l_foot_torque.coeff(2,0));

  balance_control_.setDesiredCOBGyro(0.0, 0.0);
  balance_control_.setDesiredCOBOrientation(robotis_->thormang3_link_data_[ID_PELVIS_ROT_X]->joint_angle_,
                                            robotis_->thormang3_link_data_[ID_PELVIS_ROT_Y]->joint_angle_);
  balance_control_.setDesiredFootForceTorque(0.0, 0.0, balance_goal_r_foot_ft_, 0.0, 0.0, 0.0, 0.0, 0.0, balance_goal_l_foot_ft_, 0.0, 0.0, 0.0);

  int error;
  balance_control_.process(&error, &pelvis_pose, &r_foot_pose, &l_foot_pose);

  Eigen::MatrixXd wb_pelvis_target_rotation = pelvis_pose.block(0,0,3,3);
  Eigen::MatrixXd wb_pelvis_target_position = pelvis_pose.block(0,3,3,1);

  Eigen::MatrixXd wb_l_foot_target_rotation = l_foot_pose.block(0,0,3,3);
  Eigen::MatrixXd wb_l_foot_target_position = l_foot_pose.block(0,3,3,1);
  Eigen::MatrixXd wb_r_foot_target_rotation = r_foot_pose.block(0,0,3,3);
  Eigen::MatrixXd wb_r_foot_target_position = r_foot_pose.block(0,3,3,1);

  robotis_->thormang3_link_data_[ID_PELVIS_POS_X]->relative_position_.coeffRef(0,0) = wb_pelvis_target_position.coeff(0,0);
  robotis_->thormang3_link_data_[ID_PELVIS_POS_Y]->relative_position_.coeffRef(1,0) = wb_pelvis_target_position.coeff(1,0);
  robotis_->thormang3_link_data_[ID_PELVIS_POS_Z]->relative_position_.coeffRef(2,0) = wb_pelvis_target_position.coeff(2,0);

  Eigen::MatrixXd wb_target_rpy = robotis_framework::convertRotationToRPY(wb_pelvis_target_rotation);

  robotis_->thormang3_link_data_[ID_PELVIS_ROT_X]->joint_angle_ = wb_target_rpy.coeff(0,0);
  robotis_->thormang3_link_data_[ID_PELVIS_ROT_Y]->joint_angle_ = wb_target_rpy.coeff(1,0);
  robotis_->thormang3_link_data_[ID_PELVIS_ROT_Z]->joint_angle_ = wb_target_rpy.coeff(2,0);

  /* ----- */

  bool l_foot_ik_success = robotis_->calcInverseKinematics(ID_PELVIS, ID_L_LEG_END, wb_l_foot_target_position, wb_l_foot_target_rotation, max_iter, ik_tol);
  bool r_foot_ik_success = robotis_->calcInverseKinematics(ID_PELVIS, ID_R_LEG_END, wb_r_foot_target_position, wb_r_foot_target_rotation, max_iter, ik_tol);

  if (l_arm_ik_success == true && r_arm_ik_success == true && l_foot_ik_success == true && r_foot_ik_success == true)
  {
    for (int id=1; id<=MAX_JOINT_ID; id++)
      goal_joint_position_(id) = robotis_->thormang3_link_data_[id]->joint_angle_;
  }
  else
  {
    ROS_INFO("----- ik failed -----");
    ROS_INFO("[end] send trajectory");

    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "IK Failed");

    is_moving_ = false;
    wb_l_arm_planning_ = false;
    wb_r_arm_planning_ = false;
    cnt_ = 0;

    wb_pelvis_target_position_ = robotis_->thormang3_link_data_[ID_PELVIS]->position_;
    wb_pelvis_target_rotation_ = robotis_->thormang3_link_data_[ID_PELVIS]->orientation_;

    wb_l_foot_target_position_ = wb_l_foot_default_position_;
    wb_l_foot_target_rotation_ = robotis_framework::convertQuaternionToRotation(wb_l_foot_default_quaternion_);
    wb_r_foot_target_position_ = wb_r_foot_default_position_;
    wb_r_foot_target_rotation_ = robotis_framework::convertQuaternionToRotation(wb_r_foot_default_quaternion_);
  }
}

void WholebodyModule::process(std::map<std::string, robotis_framework::Dynamixel *> dxls,
                              std::map<std::string, double> sensors)
{
  if (enable_ == false)
    return;

  /*----- Get Joint Data & Sensor Data-----*/
  for (std::map<std::string, robotis_framework::DynamixelState *>::iterator state_iter = result_.begin();
       state_iter != result_.end(); state_iter++)
  {
    std::string joint_name = state_iter->first;

    robotis_framework::Dynamixel *dxl = NULL;
    std::map<std::string, robotis_framework::Dynamixel*>::iterator dxl_it = dxls.find(joint_name);
    if (dxl_it != dxls.end())
      dxl = dxl_it->second;
    else
      continue;

    // Get Joint Data
    present_joint_position_(joint_name_to_id_[joint_name]) = dxl->dxl_state_->present_position_;
    present_joint_velocity_(joint_name_to_id_[joint_name]) = dxl->dxl_state_->present_velocity_;

    goal_joint_position_(joint_name_to_id_[joint_name]) = dxl->dxl_state_->goal_position_;

    // Get Sensor Data
    l_foot_ft_data_msg_.force.x = sensors["l_foot_fx_scaled_N"];
    l_foot_ft_data_msg_.force.y = sensors["l_foot_fy_scaled_N"];
    l_foot_ft_data_msg_.force.z = sensors["l_foot_fz_scaled_N"];
    l_foot_ft_data_msg_.torque.x = sensors["l_foot_tx_scaled_Nm"];
    l_foot_ft_data_msg_.torque.y = sensors["l_foot_ty_scaled_Nm"];
    l_foot_ft_data_msg_.torque.z = sensors["l_foot_tz_scaled_Nm"];

    r_foot_ft_data_msg_.force.x = sensors["r_foot_fx_scaled_N"];
    r_foot_ft_data_msg_.force.y = sensors["r_foot_fy_scaled_N"];
    r_foot_ft_data_msg_.force.z = sensors["r_foot_fz_scaled_N"];
    r_foot_ft_data_msg_.torque.x = sensors["r_foot_tx_scaled_Nm"];
    r_foot_ft_data_msg_.torque.y = sensors["r_foot_ty_scaled_Nm"];
    r_foot_ft_data_msg_.torque.z = sensors["r_foot_tz_scaled_Nm"];
  }

  /*----- Forward Kinematics -----*/
  for (int id=1; id<=MAX_JOINT_ID; id++)
    robotis_->thormang3_link_data_[id]->joint_angle_ = goal_joint_position_(id);

  robotis_->calcForwardKinematics(0);

  /*----- Center of Mass -----*/
  Eigen::MatrixXd mass_center = robotis_->calcMassCenter(0);
  center_of_mass_ = robotis_->calcCenterOfMass(mass_center);

  calcGoalFT();

  /* ----- Movement Event -----*/
  if (is_balancing_ == true)
  {
    if (balance_gain_cnt_ == 0)
    {
      wb_pelvis_target_position_ = robotis_->thormang3_link_data_[ID_PELVIS]->position_;
      wb_pelvis_target_rotation_ = robotis_->thormang3_link_data_[ID_PELVIS]->orientation_;

      wb_l_foot_target_position_ = wb_l_foot_default_position_;
      wb_l_foot_target_rotation_ = robotis_framework::convertQuaternionToRotation(wb_l_foot_default_quaternion_);
      wb_r_foot_target_position_ = wb_r_foot_default_position_;
      wb_r_foot_target_rotation_ = robotis_framework::convertQuaternionToRotation(wb_r_foot_default_quaternion_);
    }

    if (is_moving_ == true)
    {
      if (cnt_ == 0)
        setStartTrajectory();

      if (wb_ik_solving_ == true)
      {
        setPelvisPose(cnt_);
        setInverseKinematicsLeftFoot(cnt_);
        setInverseKinematicsRightFoot(cnt_);

        if(wb_l_arm_planning_ == true)
          setInverseKinematicsLeftArm(cnt_);
        else if (wb_r_arm_planning_ == true)
          setInverseKinematicsRightArm(cnt_);
      }
      cnt_++;
    }

    setBalanceControlGain(balance_gain_cnt_);

    if (wb_l_arm_planning_ == true || wb_r_arm_planning_ == true)
      solveWholebodyInverseKinematicsFull();
    else
      solveWholebodyInverseKinematics();

    balance_gain_cnt_++;
  }
  else
  {
    if (is_moving_ == true)
    {
      if (cnt_ == 0)
        publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Start Trajectory");

      // joint space control
      for ( int id = 1; id <= MAX_JOINT_ID; id++ )
        goal_joint_position_(id) = goal_joint_tra_(cnt_, id);

      cnt_++;
    }
  }

  /* ---- Position P Gain Update -----*/
//  if (is_gain_updating_ == true )
//  {
//    ROS_INFO("Knee P Gain Update : %d", p_gain_);

//    result_["r_leg_kn_p"]->position_p_gain_ = p_gain_;
//    result_["l_leg_kn_p"]->position_p_gain_ = p_gain_;
//    is_gain_updating_ = false;
//  }

  /* ---- Send Goal Joint Data -----*/
  for (std::map<std::string, robotis_framework::DynamixelState *>::iterator state_iter = result_.begin();
       state_iter != result_.end(); state_iter++)
  {
    std::string joint_name = state_iter->first;
    result_[joint_name]->goal_position_ = goal_joint_position_(joint_name_to_id_[joint_name]);
  }

  /*---------- Movement End Event ----------*/
  setEndTrajectory();
}

void WholebodyModule::stop()
{
  is_moving_ = false;

  wb_ik_solving_ = false;

  wb_l_arm_planning_ = false;
  wb_r_arm_planning_ = false;

  cnt_ = 0;
  balance_gain_cnt_ = 0;

  return;
}

bool WholebodyModule::isRunning()
{
  return is_moving_;
}

void WholebodyModule::publishStatusMsg(unsigned int type, std::string msg)
{
  robotis_controller_msgs::StatusMsg status_msg;
  status_msg.header.stamp = ros::Time::now();
  status_msg.type = type;
  status_msg.module_name = "Wholebody";
  status_msg.status_msg = msg;

  status_msg_pub_.publish(status_msg);
}
