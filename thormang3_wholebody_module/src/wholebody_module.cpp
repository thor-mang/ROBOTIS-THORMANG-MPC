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
  : control_cycle_msec_(0.008),
    is_moving_(false),
    ik_solving_(false),
    wb_ik_solving_(false),
    is_balancing_(false),
    on_balance_gain_(false),
    off_balance_gain_(false)
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

  ik_target_position_ = Eigen::MatrixXd::Zero(3,1);
  wb_pelvis_target_position_ = Eigen::MatrixXd::Zero(3,1);
  wb_l_foot_target_position_ = Eigen::MatrixXd::Zero(3,1);
  wb_r_foot_target_position_ = Eigen::MatrixXd::Zero(3,1);

  ik_weight_ = Eigen::MatrixXd::Zero(MAX_JOINT_ID+1,1);

  /* ----- robot tree ----- */
  robotis_ = new KinematicsDynamics(WholeBody);

  balance_control_.setGyroBalanceEnable(false);
  balance_control_.setOrientationBalanceEnable(false); // IMU
  balance_control_.setForceTorqueBalanceEnable(false); // FT

  double balance_gain_mov_time_ = 1.0;
  balance_gain_time_steps_ = int(balance_gain_mov_time_/control_cycle_msec_) + 1;

  on_balance_gain_tra_ = robotis_framework::calcMinimumJerkTra(0.0, 0.0, 0.0,
                                                               1.0, 0.0, 0.0,
                                                               control_cycle_msec_, balance_gain_mov_time_);
  off_balance_gain_tra_ = robotis_framework::calcMinimumJerkTra(1.0, 0.0, 0.0,
                                                                0.0, 0.0, 0.0,
                                                                control_cycle_msec_, balance_gain_mov_time_);
}

WholebodyModule::~WholebodyModule()
{
  queue_thread_.join();
}

void WholebodyModule::initialize(const int control_cycle_msec, robotis_framework::Robot *robot)
{
  control_cycle_msec_ = control_cycle_msec * 0.001;
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

  all_time_steps_ = int(mov_time_ / control_cycle_msec_) + 1;
  goal_joint_tra_.resize(all_time_steps_, MAX_JOINT_ID + 1);
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

      tra_gene_tread_ = new boost::thread(boost::bind(&WholebodyModule::traGeneProcForIniPose, this));
      delete tra_gene_tread_;
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
    tra_gene_tread_ = new boost::thread(boost::bind(&WholebodyModule::traGeneProcForJointSpace, this));
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
      if (goal_kinematics_pose_msg_.name == "left_arm" || goal_kinematics_pose_msg_.name == "left_arm_with_torso")
      {
        ik_id_start_ = ID_L_ARM_START;
        if (goal_kinematics_pose_msg_.name == "left_arm_with_torso")
          ik_id_start_ = ID_TORSO;

        ik_id_end_ = ID_L_ARM_END;

        tra_gene_tread_ = new boost::thread(boost::bind(&WholebodyModule::traGeneProcForTaskSpace, this));
        delete tra_gene_tread_;
      }
      else if (goal_kinematics_pose_msg_.name == "right_arm" || goal_kinematics_pose_msg_.name == "right_arm_with_torso")
      {
        ik_id_start_ = ID_R_ARM_START;
        if (goal_kinematics_pose_msg_.name == "right_arm_with_torso")
          ik_id_start_ = ID_TORSO;

        ik_id_end_ = ID_R_ARM_END;

        tra_gene_tread_ = new boost::thread(boost::bind(&WholebodyModule::traGeneProcForTaskSpace, this));
        delete tra_gene_tread_;
      }
      else if (goal_kinematics_pose_msg_.name == "pelvis")
      {
        ik_id_start_ = ID_PELVIS;
        ik_id_end_ = ID_PELVIS;

        tra_gene_tread_ = new boost::thread(boost::bind(&WholebodyModule::traGeneProcForWholebody, this));
        delete tra_gene_tread_;
      }
      else if (goal_kinematics_pose_msg_.name == "stand_wheel_pose" || goal_kinematics_pose_msg_.name == "stand_wheel_pose_back")
      {
        ik_id_start_ = ID_PELVIS;
        ik_id_end_ = ID_PELVIS;

        tra_gene_tread_ = new boost::thread(boost::bind(&WholebodyModule::traGeneProcForStandWheelPose, this));
        delete tra_gene_tread_;
      }
    }
    else
      ROS_INFO("previous task is alive");
  }
  else
    ROS_INFO("balance is off");

  return;
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
    ik_solving_ = false;
    wb_ik_solving_ = false;
  }
  else if(msg->data == "balance_off")
  {
    ROS_INFO("balance off");

//    is_balancing_ = false;
    off_balance_gain_ = true;
    balance_gain_cnt_ = 0;

    is_moving_ = false;
    ik_solving_ = false;
    wb_ik_solving_ = false;
  }
}

void WholebodyModule::imuDataCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
  imu_data_msg_ = *msg;

  imu_data_msg_.angular_velocity.x *= -1.0;
  imu_data_msg_.angular_velocity.y *= -1.0;
}

void WholebodyModule::traGeneProcForIniPose()
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
                                                  control_cycle_msec_, mov_time_);
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
                                                               control_cycle_msec_, via_time_, mov_time_);
    }

    goal_joint_tra_.block(0, id, all_time_steps_, 1) = tra;
  }

  is_moving_ = true;
  cnt_ = 0;
  ROS_INFO("[start] send trajectory");
}

void WholebodyModule::traGeneProcForStandWheelPose()
{
  mov_time_ = 3.0;
  all_time_steps_ = int(mov_time_/control_cycle_msec_) + 1;

  goal_pevlis_tra_.resize(all_time_steps_, 3);
  goal_l_foot_tra_.resize(all_time_steps_, 3);
  goal_r_foot_tra_.resize(all_time_steps_, 3);

  Eigen::MatrixXd pelvis_tar_vector = Eigen::MatrixXd::Zero(3,1);
  pelvis_tar_vector.coeffRef(2,0) = 0.0;

  if ( goal_kinematics_pose_msg_.name == "stand_wheel_pose_back" )
  {
    pelvis_tar_vector.coeffRef(2,0) = 0.0;
  }

  /* calculate trajectory */
  for (int dim=0; dim<3; dim++)
  {
    double ini_value = robotis_->thormang3_link_data_[ID_PELVIS]->position_.coeff(dim,0);
    double tar_value = ini_value + pelvis_tar_vector.coeff(dim,0);

    Eigen::MatrixXd tra = robotis_framework::calcMinimumJerkTra(ini_value, 0.0, 0.0,
                                                                tar_value, 0.0, 0.0,
                                                                control_cycle_msec_, mov_time_);

    goal_pevlis_tra_.block(0, dim, all_time_steps_, 1) = tra;
  }

  for (int dim=0; dim<3; dim++)
  {
    double ini_value = robotis_->thormang3_link_data_[ID_L_LEG_END]->position_.coeff(dim,0);
    double tar_value = robotis_->thormang3_link_data_[ID_L_LEG_END]->position_.coeff(dim,0);

    Eigen::MatrixXd tra = robotis_framework::calcMinimumJerkTra(ini_value, 0.0, 0.0,
                                                                tar_value, 0.0, 0.0,
                                                                control_cycle_msec_, mov_time_);

    goal_l_foot_tra_.block(0, dim, all_time_steps_, 1) = tra;
  }

  for (int dim=0; dim<3; dim++)
  {
    double ini_value = robotis_->thormang3_link_data_[ID_R_LEG_END]->position_.coeff(dim,0);
    double tar_value = robotis_->thormang3_link_data_[ID_R_LEG_END]->position_.coeff(dim,0);

    Eigen::MatrixXd tra = robotis_framework::calcMinimumJerkTra(ini_value, 0.0, 0.0,
                                                                tar_value, 0.0, 0.0,
                                                                control_cycle_msec_, mov_time_);

    goal_r_foot_tra_.block(0, dim, all_time_steps_, 1) = tra;
  }

  /* target quaternion */
  Eigen::Quaterniond pelvis_target_quaternion =
      robotis_framework::convertRotationToQuaternion(robotis_->thormang3_link_data_[ID_PELVIS]->orientation_);

  wb_pelvis_goal_quaternion_ = pelvis_target_quaternion;

  double l_foot_target_roll = -14.0 * DEGREE2RADIAN;
  double r_foot_target_roll = 14.0 * DEGREE2RADIAN;

  if ( goal_kinematics_pose_msg_.name == "stand_wheel_pose_back" )
  {
    l_foot_target_roll = 0.0;
    r_foot_target_roll = 0.0;
  }

  Eigen::Quaterniond l_foot_target_quaternion =
      robotis_framework::convertRPYToQuaternion(l_foot_target_roll, 0.0, 0.0);

  wb_l_foot_goal_quaternion_ = l_foot_target_quaternion;

  Eigen::Quaterniond r_foot_target_quaternion =
      robotis_framework::convertRPYToQuaternion(r_foot_target_roll, 0.0, 0.0);

  wb_r_foot_goal_quaternion_ = r_foot_target_quaternion;

  cnt_ = 0;
  is_moving_ = true;
  wb_ik_solving_ = true;

  ROS_INFO("[start] send trajectory");
}

void WholebodyModule::traGeneProcForJointSpace()
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
    int all_time_steps = int(floor((mov_time_ / control_cycle_msec_) + 1));
    mov_time_ = double (all_time_steps-1) * control_cycle_msec_;

    if (mov_time_ < mov_time)
      mov_time_= mov_time;
  }
  else
  {
    mov_time_ = goal_joint_pose_msg_.time;
    int all_time_steps = int(floor((mov_time_/control_cycle_msec_) + 1 ));
    mov_time_ = double (all_time_steps - 1) * control_cycle_msec_;
  }

  all_time_steps_ = int(mov_time_ / control_cycle_msec_) + 1;
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
                                              control_cycle_msec_, mov_time_);

    goal_joint_tra_.block(0, id, all_time_steps_, 1) = tra;
  }

  cnt_ = 0;
  is_moving_ = true;

  ROS_INFO("[start] send trajectory");
}

void WholebodyModule::traGeneProcForTaskSpace()
{
  if (goal_kinematics_pose_msg_.time <= 0.0)
  {
    /* set movement time */
    double tol = 0.1; // m per sec
    double mov_time = 2.0;

    double diff = sqrt(pow(robotis_->thormang3_link_data_[ik_id_end_]->position_.coeff(0,0) - goal_kinematics_pose_msg_.pelvis_pose.position.x, 2) +
                       pow(robotis_->thormang3_link_data_[ik_id_end_]->position_.coeff(1,0) - goal_kinematics_pose_msg_.pelvis_pose.position.y, 2) +
                       pow(robotis_->thormang3_link_data_[ik_id_end_]->position_.coeff(2,0) - goal_kinematics_pose_msg_.pelvis_pose.position.z, 2));

    mov_time_ = diff / tol;
    int all_time_steps = int(floor((mov_time_/control_cycle_msec_) + 1 ));
    mov_time_ = double (all_time_steps - 1) * control_cycle_msec_;

    if ( mov_time_ < mov_time )
      mov_time_ = mov_time;
  }
  else
  {
    mov_time_ = goal_kinematics_pose_msg_.time;
    int all_time_steps = int(floor((mov_time_/control_cycle_msec_) + 1 ));
    mov_time_ = double (all_time_steps - 1) * control_cycle_msec_;
  }

  all_time_steps_ = int(mov_time_/control_cycle_msec_) + 1;
  goal_task_tra_.resize(all_time_steps_, 3);

  /* calculate trajectory */
  for ( int dim = 0; dim < 3; dim++ )
  {
    double ini_value = robotis_->thormang3_link_data_[ik_id_end_]->position_.coeff(dim,0);
    double tar_value;
    if ( dim == 0 )
      tar_value = goal_kinematics_pose_msg_.pelvis_pose.position.x ;
    else if ( dim == 1 )
      tar_value = goal_kinematics_pose_msg_.pelvis_pose.position.y ;
    else if ( dim == 2 )
      tar_value = goal_kinematics_pose_msg_.pelvis_pose.position.z ;

    Eigen::MatrixXd tra = robotis_framework::calcMinimumJerkTra(ini_value, 0.0, 0.0,
                                                                tar_value, 0.0, 0.0,
                                                                control_cycle_msec_, mov_time_);

    goal_task_tra_.block(0, dim, all_time_steps_, 1) = tra;
  }

  /* target quaternion */
  Eigen::Quaterniond target_quaternion(goal_kinematics_pose_msg_.pelvis_pose.orientation.w,
                                       goal_kinematics_pose_msg_.pelvis_pose.orientation.x,
                                       goal_kinematics_pose_msg_.pelvis_pose.orientation.y,
                                       goal_kinematics_pose_msg_.pelvis_pose.orientation.z);

  ik_goal_quaternion_ = target_quaternion;

  cnt_ = 0;
  is_moving_ = true;
  ik_solving_ = true;

  ROS_INFO("[start] send trajectory");
}

void WholebodyModule::traGeneProcForWholebody()
{
  if (goal_kinematics_pose_msg_.time <= 0.0)
  {
    /* set movement time */
    double tol = 0.1; // m per sec
    double mov_time = 2.0;

    double diff = sqrt(pow(robotis_->thormang3_link_data_[ik_id_end_]->position_.coeff(0,0) - goal_kinematics_pose_msg_.pelvis_pose.position.x, 2) +
                       pow(robotis_->thormang3_link_data_[ik_id_end_]->position_.coeff(1,0) - goal_kinematics_pose_msg_.pelvis_pose.position.y, 2) +
                       pow(robotis_->thormang3_link_data_[ik_id_end_]->position_.coeff(2,0) - goal_kinematics_pose_msg_.pelvis_pose.position.z, 2));

    mov_time_ = diff / tol;
    int all_time_steps = int(floor((mov_time_/control_cycle_msec_) + 1 ));
    mov_time_ = double (all_time_steps - 1) * control_cycle_msec_;

    if ( mov_time_ < mov_time )
      mov_time_ = mov_time;
  }
  else
  {
    mov_time_ = goal_kinematics_pose_msg_.time;
    int all_time_steps = int(floor((mov_time_/control_cycle_msec_) + 1 ));
    mov_time_ = double (all_time_steps - 1) * control_cycle_msec_;
  }

  all_time_steps_ = int(mov_time_/control_cycle_msec_) + 1;
  goal_pevlis_tra_.resize(all_time_steps_, 3);
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
                                                                control_cycle_msec_, mov_time_);

    goal_pevlis_tra_.block(0, dim, all_time_steps_, 1) = tra;
  }

  for (int dim=0; dim<3; dim++)
  {
    double ini_value = robotis_->thormang3_link_data_[ID_L_LEG_END]->position_.coeff(dim,0);
    double tar_value = robotis_->thormang3_link_data_[ID_L_LEG_END]->position_.coeff(dim,0);

    Eigen::MatrixXd tra = robotis_framework::calcMinimumJerkTra(ini_value, 0.0, 0.0,
                                                                tar_value, 0.0, 0.0,
                                                                control_cycle_msec_, mov_time_);

    goal_l_foot_tra_.block(0, dim, all_time_steps_, 1) = tra;
  }

  for (int dim=0; dim<3; dim++)
  {
    double ini_value = robotis_->thormang3_link_data_[ID_R_LEG_END]->position_.coeff(dim,0);
    double tar_value = robotis_->thormang3_link_data_[ID_R_LEG_END]->position_.coeff(dim,0);

    Eigen::MatrixXd tra = robotis_framework::calcMinimumJerkTra(ini_value, 0.0, 0.0,
                                                                tar_value, 0.0, 0.0,
                                                                control_cycle_msec_, mov_time_);

    goal_r_foot_tra_.block(0, dim, all_time_steps_, 1) = tra;
  }


  /* target quaternion */
  Eigen::Quaterniond pelvis_target_quaternion(goal_kinematics_pose_msg_.pelvis_pose.orientation.w,
                                              goal_kinematics_pose_msg_.pelvis_pose.orientation.x,
                                              goal_kinematics_pose_msg_.pelvis_pose.orientation.y,
                                              goal_kinematics_pose_msg_.pelvis_pose.orientation.z);

  wb_pelvis_goal_quaternion_ = pelvis_target_quaternion;

  Eigen::Quaterniond l_foot_target_quaternion =
      robotis_framework::convertRotationToQuaternion(robotis_->thormang3_link_data_[ID_L_LEG_END]->orientation_);

  wb_l_foot_goal_quaternion_ = l_foot_target_quaternion;

  Eigen::Quaterniond r_foot_target_quaternion =
      robotis_framework::convertRotationToQuaternion(robotis_->thormang3_link_data_[ID_R_LEG_END]->orientation_);

  wb_r_foot_goal_quaternion_ = r_foot_target_quaternion;

  cnt_ = 0;
  is_moving_ = true;
  wb_ik_solving_ = true;

  ROS_INFO("[start] send trajectory");
}

void WholebodyModule::traGeneProcForPelvis()
{
  if (goal_kinematics_pose_msg_.time <= 0.0)
  {
    /* set movement time */
    double tol = 0.1; // m per sec
    double mov_time = 2.0;

    double diff = sqrt(pow(robotis_->thormang3_link_data_[ik_id_end_]->position_.coeff(0,0) - goal_kinematics_pose_msg_.pelvis_pose.position.x, 2) +
                       pow(robotis_->thormang3_link_data_[ik_id_end_]->position_.coeff(1,0) - goal_kinematics_pose_msg_.pelvis_pose.position.y, 2) +
                       pow(robotis_->thormang3_link_data_[ik_id_end_]->position_.coeff(2,0) - goal_kinematics_pose_msg_.pelvis_pose.position.z, 2));

    mov_time_ = diff / tol;
    int all_time_steps = int(floor((mov_time_/control_cycle_msec_) + 1 ));
    mov_time_ = double (all_time_steps - 1) * control_cycle_msec_;

    if ( mov_time_ < mov_time )
      mov_time_ = mov_time;
  }
  else
  {
    mov_time_ = goal_kinematics_pose_msg_.time;
    int all_time_steps = int(floor((mov_time_/control_cycle_msec_) + 1 ));
    mov_time_ = double (all_time_steps - 1) * control_cycle_msec_;
  }

  all_time_steps_ = int(mov_time_/control_cycle_msec_) + 1;
  goal_pevlis_tra_.resize(all_time_steps_, 3);

  Eigen::MatrixXd tar_vector = Eigen::MatrixXd::Zero(3,1);
  tar_vector.coeffRef(0,0) = goal_kinematics_pose_msg_.pelvis_pose.position.x;
  tar_vector.coeffRef(1,0) = goal_kinematics_pose_msg_.pelvis_pose.position.y;
  tar_vector.coeffRef(2,0) = goal_kinematics_pose_msg_.pelvis_pose.position.z;

  /* calculate trajectory */
  for (int dim=0; dim<3; dim++)
  {
    double ini_value = robotis_->thormang3_link_data_[ik_id_end_]->position_.coeff(dim,0);
    double tar_value = tar_vector.coeff(dim,0);

    Eigen::MatrixXd tra = robotis_framework::calcMinimumJerkTra(ini_value, 0.0, 0.0,
                                                                tar_value, 0.0, 0.0,
                                                                control_cycle_msec_, mov_time_);

    goal_pevlis_tra_.block(0, dim, all_time_steps_, 1) = tra;
  }

  /* target quaternion */
  Eigen::Quaterniond target_quaternion(goal_kinematics_pose_msg_.pelvis_pose.orientation.w,
                                       goal_kinematics_pose_msg_.pelvis_pose.orientation.x,
                                       goal_kinematics_pose_msg_.pelvis_pose.orientation.y,
                                       goal_kinematics_pose_msg_.pelvis_pose.orientation.z);

  wb_pelvis_goal_quaternion_ = target_quaternion;

  cnt_ = 0;
  is_moving_ = true;
  wb_ik_solving_ = true;

  ROS_INFO("[start] send trajectory");
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

  if (req.group_name == "left_arm" || req.group_name == "left_arm_with_torso")
    end_index = ID_L_ARM_END;
  else if (req.group_name == "right_arm" || req.group_name == "right_arm_with_torso")
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

  return  true;
}

void WholebodyModule::setInverseKinematics(int cnt)
{
  for (int dim=0; dim<3; dim++)
    ik_target_position_.coeffRef(dim, 0) = goal_task_tra_.coeff(cnt, dim);

  double time_step = ( double ) cnt / ( double ) all_time_steps_;
  Eigen::Quaterniond quaternion = ik_start_quaternion_.slerp(time_step, ik_goal_quaternion_);

  ik_target_rotation_ = robotis_framework::convertQuaternionToRotation(quaternion);
}

void WholebodyModule::setPelvisPose(int cnt)
{
  for ( int dim = 0; dim < 3; dim++ )
    wb_pelvis_target_position_.coeffRef(dim, 0) = goal_pevlis_tra_.coeff(cnt, dim);

  double time_step = ( double ) cnt / ( double ) all_time_steps_;
  Eigen::Quaterniond quaternion = wb_pelvis_start_quaternion_.slerp(time_step, wb_pelvis_goal_quaternion_);

  wb_pelvis_target_rotation_ = robotis_framework::convertQuaternionToRotation(quaternion);
}

void WholebodyModule::setInverseKinematicsForLeftFoot(int cnt)
{
  for (int dim=0; dim<3; dim++)
    wb_l_foot_target_position_.coeffRef(dim, 0) = goal_l_foot_tra_.coeff(cnt, dim);

  double time_step = ( double ) cnt / ( double ) all_time_steps_;
  Eigen::Quaterniond quaternion = wb_l_foot_start_quaternion_.slerp(time_step, wb_l_foot_goal_quaternion_);

  wb_l_foot_target_rotation_ = robotis_framework::convertQuaternionToRotation(quaternion);
}

void WholebodyModule::setInverseKinematicsForRightFoot(int cnt)
{
  for (int dim=0; dim<3; dim++)
    wb_r_foot_target_position_.coeffRef(dim, 0) = goal_r_foot_tra_.coeff(cnt, dim);

  double time_step = ( double ) cnt / ( double ) all_time_steps_;
  Eigen::Quaterniond quaternion = wb_r_foot_start_quaternion_.slerp(time_step, wb_r_foot_goal_quaternion_);

  wb_r_foot_target_rotation_ = robotis_framework::convertQuaternionToRotation(quaternion);
}

void WholebodyModule::setStartTrajectory()
{
  publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Start Trajectory");

  if (ik_solving_ == true)
  {
    Eigen::MatrixXd ik_start_rotation = robotis_->thormang3_link_data_[ik_id_end_]->orientation_;
    ik_start_quaternion_ = robotis_framework::convertRotationToQuaternion(ik_start_rotation);
  }
  else if (wb_ik_solving_ == true)
  {
    Eigen::MatrixXd wb_start_rotation = robotis_->thormang3_link_data_[ID_PELVIS]->orientation_;
    wb_pelvis_start_quaternion_ = robotis_framework::convertRotationToQuaternion(wb_start_rotation);

    Eigen::MatrixXd wb_l_foot_start_rotation = robotis_->thormang3_link_data_[ID_L_LEG_END]->orientation_;
    wb_l_foot_start_quaternion_ = robotis_framework::convertRotationToQuaternion(wb_l_foot_start_rotation);

    Eigen::MatrixXd wb_r_foot_start_rotation = robotis_->thormang3_link_data_[ID_R_LEG_END]->orientation_;
    wb_r_foot_start_quaternion_ = robotis_framework::convertRotationToQuaternion(wb_r_foot_start_rotation);
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

      ik_solving_ = false;
      wb_ik_solving_ = false;

      cnt_ = 0;
    }
  }
}

void WholebodyModule::setBalanceControlGain(int cnt)
{
  if (on_balance_gain_ == true)
  {
    if (balance_gain_cnt_ >= balance_gain_time_steps_)
    {
      ROS_INFO("balance gain activated");
      on_balance_gain_ = false;
    }
    else
    {
      balance_control_.setGyroBalanceGainRatio(0.5 * on_balance_gain_tra_.coeff(cnt,0));

      balance_control_.foot_roll_angle_ctrl_.gain_ = -1.0 * on_balance_gain_tra_.coeff(cnt,0);
      balance_control_.foot_pitch_angle_ctrl_.gain_ = -1.0 * on_balance_gain_tra_.coeff(cnt,0);

      balance_control_.left_foot_force_x_ctrl_.gain_ = -0.1 * on_balance_gain_tra_.coeff(cnt,0);
      balance_control_.left_foot_force_y_ctrl_.gain_ = -0.1 * on_balance_gain_tra_.coeff(cnt,0);

      balance_control_.right_foot_force_y_ctrl_.gain_ = -0.1 * on_balance_gain_tra_.coeff(cnt,0);
      balance_control_.right_foot_force_y_ctrl_.gain_ = -0.1 * on_balance_gain_tra_.coeff(cnt,0);

      balance_control_.foot_force_z_diff_ctrl_.gain_ = -0.05 * on_balance_gain_tra_.coeff(cnt,0);

      balance_control_.left_foot_force_x_ctrl_.time_constant_sec_ = 0.2;
    }
  }

  if (off_balance_gain_ == true)
  {
    if (balance_gain_cnt_ >= balance_gain_time_steps_)
    {
      ROS_INFO("balance gain disactivated");
      off_balance_gain_ = false;
    }
    else
    {
      balance_control_.setGyroBalanceGainRatio(0.5 * off_balance_gain_tra_.coeff(cnt,0));

      balance_control_.foot_roll_angle_ctrl_.gain_ = -1.0 * off_balance_gain_tra_.coeff(cnt,0);
      balance_control_.foot_pitch_angle_ctrl_.gain_ = -1.0 * off_balance_gain_tra_.coeff(cnt,0);

      balance_control_.left_foot_force_x_ctrl_.gain_ = -0.1 * off_balance_gain_tra_.coeff(cnt,0);
      balance_control_.left_foot_force_y_ctrl_.gain_ = -0.1 * off_balance_gain_tra_.coeff(cnt,0);

      balance_control_.right_foot_force_y_ctrl_.gain_ = -0.1 * off_balance_gain_tra_.coeff(cnt,0);
      balance_control_.right_foot_force_y_ctrl_.gain_ = -0.1 * off_balance_gain_tra_.coeff(cnt,0);

      balance_control_.foot_force_z_diff_ctrl_.gain_ = -0.05 * off_balance_gain_tra_.coeff(cnt,0);

      balance_control_.left_foot_force_x_ctrl_.time_constant_sec_ = 0.2;
    }
  }
}

void WholebodyModule::solveInverseKinematics()
{
  setInverseKinematics(cnt_);

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
}

void WholebodyModule::solveWholebodyInverseKinematics()
{
  setPelvisPose(cnt_);
  setInverseKinematicsForLeftFoot(cnt_);
  setInverseKinematicsForRightFoot(cnt_);

//  robotis_->thormang3_link_data_[ID_PELVIS_POS_X]->relative_position_.coeffRef(0,0) = wb_pelvis_target_position_.coeff(0,0);
//  robotis_->thormang3_link_data_[ID_PELVIS_POS_Y]->relative_position_.coeffRef(1,0) = wb_pelvis_target_position_.coeff(1,0);
//  robotis_->thormang3_link_data_[ID_PELVIS_POS_Z]->relative_position_.coeffRef(2,0) = wb_pelvis_target_position_.coeff(2,0);

//  Eigen::MatrixXd wb_target_rpy = robotis_framework::convertRotationToRPY(wb_pelvis_target_rotation_);

//  robotis_->thormang3_link_data_[ID_PELVIS_ROT_X]->joint_angle_ = wb_target_rpy.coeff(0,0);
//  robotis_->thormang3_link_data_[ID_PELVIS_ROT_Y]->joint_angle_ = wb_target_rpy.coeff(1,0);
//  robotis_->thormang3_link_data_[ID_PELVIS_ROT_Z]->joint_angle_ = wb_target_rpy.coeff(2,0);

  /* ----- */

//  int max_iter = 70;
//  double ik_tol = 1e-5;
//  bool l_foot_ik_success = robotis_->calcInverseKinematics(ID_PELVIS, ID_L_LEG_END, wb_l_foot_target_position_, wb_l_foot_target_rotation_, max_iter, ik_tol);
//  bool r_foot_ik_success = robotis_->calcInverseKinematics(ID_PELVIS, ID_R_LEG_END, wb_r_foot_target_position_, wb_r_foot_target_rotation_, max_iter, ik_tol);

//  if (l_foot_ik_success == true && r_foot_ik_success == true)
//  {
//    for (int id=1; id<=MAX_JOINT_ID; id++)
//      goal_joint_position_(id) = robotis_->thormang3_link_data_[id]->joint_angle_;
//  }
//  else
//  {
//    ROS_INFO("----- ik failed -----");
//    ROS_INFO("[end] send trajectory");

//    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "IK Failed");

//    is_moving_ = false;
//    cnt_ = 0;
//  }
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

  /* ----- Movement Event -----*/
  if (is_balancing_ == true)
  {
    if (balance_gain_cnt_ == 0)
    {
      wb_pelvis_target_position_ = robotis_->thormang3_link_data_[ID_PELVIS]->position_;
      wb_pelvis_target_rotation_ = robotis_->thormang3_link_data_[ID_PELVIS]->orientation_;

      wb_l_foot_target_position_ = robotis_->thormang3_link_data_[ID_L_LEG_END]->position_;
      wb_l_foot_target_rotation_ = robotis_->thormang3_link_data_[ID_L_LEG_END]->orientation_;
      wb_r_foot_target_position_ = robotis_->thormang3_link_data_[ID_R_LEG_END]->position_;
      wb_r_foot_target_rotation_ = robotis_->thormang3_link_data_[ID_R_LEG_END]->orientation_;
    }

    setBalanceControlGain(balance_gain_cnt_);

    if (is_moving_ == true)
    {
      if (cnt_ == 0)
        setStartTrajectory();

      if (wb_ik_solving_ == true)
        solveWholebodyInverseKinematics();

      cnt_++;
    }

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

    setBalanceControlGain(balance_gain_cnt_);

//    balance_control_.setCurrentGyroSensorOutput(0.0, 0.0);
    balance_control_.setCurrentGyroSensorOutput(imu_data_msg_.angular_velocity.x, imu_data_msg_.angular_velocity.y);

    Eigen::Quaterniond imu_quaternion(imu_data_msg_.orientation.w,
                                      imu_data_msg_.orientation.x,
                                      imu_data_msg_.orientation.y,
                                      imu_data_msg_.orientation.z);
    Eigen::MatrixXd imu_rpy = robotis_framework::convertQuaternionToRPY(imu_quaternion);

//    balance_control_.setCurrentOrientationSensorOutput(0.0, 0.0);
//    balance_control_.setCurrentFootForceTorqueSensorOutput(0.0, 0.0, 0.0,
//                                                           0.0, 0.0, 0.0,
//                                                           0.0, 0.0, 0.0,
//                                                           0.0, 0.0, 0.0);

    Eigen::Quaterniond imu_quaternion(imu_data_msg_.orientation.w,
                                      imu_data_msg_.orientation.x,
                                      imu_data_msg_.orientation.y,
                                      imu_data_msg_.orientation.z);
    Eigen::MatrixXd imu_rpy = robotis_framework::convertQuaternionToRPY(imu_quaternion);

    balance_control_.setCurrentOrientationSensorOutput(imu_rpy.coeff(0,0), imu_rpy.coeff(1,0));
    balance_control_.setCurrentFootForceTorqueSensorOutput(r_foot_ft_data_msg_.force.x,  r_foot_ft_data_msg_.force.y,  r_foot_ft_data_msg_.force.z,
                                                           r_foot_ft_data_msg_.torque.x, r_foot_ft_data_msg_.torque.y, r_foot_ft_data_msg_.torque.z,
                                                           l_foot_ft_data_msg_.force.x,  l_foot_ft_data_msg_.force.y,  l_foot_ft_data_msg_.force.z,
                                                           l_foot_ft_data_msg_.torque.x, l_foot_ft_data_msg_.torque.y, l_foot_ft_data_msg_.torque.z);

    balance_control_.setDesiredCOBGyro(0.0, 0.0);
    balance_control_.setDesiredCOBOrientation(0.0, 0.0);
    balance_control_.setDesiredFootForceTorque(0.0, 0.0, -210.0, 0.0, 0.0, 0.0, 0.0, 0.0, -210, 0.0, 0.0, 0.0);

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

      is_balancing_ = false;
      cnt_ = 0;
    }

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



//  if (is_moving_ == true)
//  {
//    if (cnt_ == 0)
//      setStartTrajectory();

//    if (ik_solving_ == true)
//      solveInverseKinematics();
//    else if (wb_ik_solving_ == true)
//      solveWholebodyInverseKinematics();
//    else
//    {
//      // joint space control
//      for ( int id = 1; id <= MAX_JOINT_ID; id++ )
//        goal_joint_position_(id) = goal_joint_tra_(cnt_, id);
//    }

//    cnt_++;
//  }

//  if (is_balancing_ == true)
//  {
//    balance_control_.setGyroBalanceEnable(true);
//    balance_control_.setOrientationBalanceEnable(true);
//    balance_control_.setForceTorqueBalanceEnable(true);

//    Eigen::MatrixXd pelvis_pose = Eigen::MatrixXd::Identity(4,4);

//    pelvis_pose.block(0,0,3,3) = wb_pelvis_target_rotation_;
//    pelvis_pose.block(0,3,3,1) = wb_pelvis_target_position_;

//    Eigen::MatrixXd l_foot_pose = Eigen::MatrixXd::Identity(4,4);
//    Eigen::MatrixXd r_foot_pose = Eigen::MatrixXd::Identity(4,4);

//    l_foot_pose.block(0,0,3,3) = wb_l_foot_target_rotation_;
//    l_foot_pose.block(0,3,3,1) = wb_l_foot_target_position_;
//    r_foot_pose.block(0,0,3,3) = wb_r_foot_target_rotation_;
//    r_foot_pose.block(0,3,3,1) = wb_r_foot_target_position_;

//    balance_control_.setDesiredPose(pelvis_pose, r_foot_pose, l_foot_pose);

//    setBalanceControlGain(balance_gain_cnt_);

////    balance_control_.setCurrentGyroSensorOutput(0.0, 0.0);
//    balance_control_.setCurrentGyroSensorOutput(imu_data_msg_.angular_velocity.x, imu_data_msg_.angular_velocity.y);

//    Eigen::Quaterniond imu_quaternion(imu_data_msg_.orientation.w,
//                                      imu_data_msg_.orientation.x,
//                                      imu_data_msg_.orientation.y,
//                                      imu_data_msg_.orientation.z);
//    Eigen::MatrixXd imu_rpy = robotis_framework::convertQuaternionToRPY(imu_quaternion);

////    balance_control_.setCurrentOrientationSensorOutput(0.0, 0.0);
////    balance_control_.setCurrentFootForceTorqueSensorOutput(0.0, 0.0, 0.0,
////                                                           0.0, 0.0, 0.0,
////                                                           0.0, 0.0, 0.0,
////                                                           0.0, 0.0, 0.0);

////    Eigen::Quaterniond imu_quaternion(imu_data_msg_.orientation.w,
////                                      imu_data_msg_.orientation.x,
////                                      imu_data_msg_.orientation.y,
////                                      imu_data_msg_.orientation.z);
////    Eigen::MatrixXd imu_rpy = robotis_framework::convertQuaternionToRPY(imu_quaternion);

////    balance_control_.setCurrentOrientationSensorOutput(imu_rpy.coeff(0,0), imu_rpy.coeff(1,0));
////    balance_control_.setCurrentFootForceTorqueSensorOutput(r_foot_ft_data_msg_.force.x,  r_foot_ft_data_msg_.force.y,  r_foot_ft_data_msg_.force.z,
////                                                           r_foot_ft_data_msg_.torque.x, r_foot_ft_data_msg_.torque.y, r_foot_ft_data_msg_.torque.z,
////                                                           l_foot_ft_data_msg_.force.x,  l_foot_ft_data_msg_.force.y,  l_foot_ft_data_msg_.force.z,
////                                                           l_foot_ft_data_msg_.torque.x, l_foot_ft_data_msg_.torque.y, l_foot_ft_data_msg_.torque.z);

//    balance_control_.setDesiredCOBGyro(0.0, 0.0);
//    balance_control_.setDesiredCOBOrientation(0.0, 0.0);
//    balance_control_.setDesiredFootForceTorque(0.0, 0.0, -210.0, 0.0, 0.0, 0.0, 0.0, 0.0, -210, 0.0, 0.0, 0.0);

//    int error;
//    balance_control_.process(&error, &pelvis_pose, &r_foot_pose, &l_foot_pose);

//    Eigen::MatrixXd wb_pelvis_target_rotation = pelvis_pose.block(0,0,3,3);
//    Eigen::MatrixXd wb_pelvis_target_position = pelvis_pose.block(0,3,3,1);

//    Eigen::MatrixXd wb_l_foot_target_rotation = l_foot_pose.block(0,0,3,3);
//    Eigen::MatrixXd wb_l_foot_target_position = l_foot_pose.block(0,3,3,1);
//    Eigen::MatrixXd wb_r_foot_target_rotation = r_foot_pose.block(0,0,3,3);
//    Eigen::MatrixXd wb_r_foot_target_position = r_foot_pose.block(0,3,3,1);

//    robotis_->thormang3_link_data_[ID_PELVIS_POS_X]->relative_position_.coeffRef(0,0) = wb_pelvis_target_position.coeff(0,0);
//    robotis_->thormang3_link_data_[ID_PELVIS_POS_Y]->relative_position_.coeffRef(1,0) = wb_pelvis_target_position.coeff(1,0);
//    robotis_->thormang3_link_data_[ID_PELVIS_POS_Z]->relative_position_.coeffRef(2,0) = wb_pelvis_target_position.coeff(2,0);

//    Eigen::MatrixXd wb_target_rpy = robotis_framework::convertRotationToRPY(wb_pelvis_target_rotation);

//    robotis_->thormang3_link_data_[ID_PELVIS_ROT_X]->joint_angle_ = wb_target_rpy.coeff(0,0);
//    robotis_->thormang3_link_data_[ID_PELVIS_ROT_Y]->joint_angle_ = wb_target_rpy.coeff(1,0);
//    robotis_->thormang3_link_data_[ID_PELVIS_ROT_Z]->joint_angle_ = wb_target_rpy.coeff(2,0);

//    int max_iter = 70;
//    double ik_tol = 1e-5;
//    bool l_foot_ik_success = robotis_->calcInverseKinematics(ID_PELVIS, ID_L_LEG_END, wb_l_foot_target_position, wb_l_foot_target_rotation, max_iter, ik_tol);
//    bool r_foot_ik_success = robotis_->calcInverseKinematics(ID_PELVIS, ID_R_LEG_END, wb_r_foot_target_position, wb_r_foot_target_rotation, max_iter, ik_tol);

//    if (l_foot_ik_success == true && r_foot_ik_success == true)
//    {
//      for (int id=1; id<=MAX_JOINT_ID; id++)
//        goal_joint_position_(id) = robotis_->thormang3_link_data_[id]->joint_angle_;
//    }
//    else
//    {
//      ROS_INFO("----- ik failed -----");
//      ROS_INFO("[end] send trajectory");

//      publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "IK Failed");

//      is_balancing_ = false;
//      cnt_ = 0;
//      balance_gain_cnt_ = 0;
//    }

//    balance_gain_cnt_++;
//  }

  /* ---- Send Goal Joint Data -----*/
  sensor_msgs::JointState goal_joint_states_msg;

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

  ik_solving_ = false;
  wb_ik_solving_ = false;

  cnt_ = 0;

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
