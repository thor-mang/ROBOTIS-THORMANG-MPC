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
 *  manipulation_module.cpp
 *
 *  Created on: December 13, 2016
 *      Author: sch
 */

#include "thormang3_manipulation_module/manipulation_module.h"

using namespace thormang3;

ManipulationModule::ManipulationModule()
  : control_cycle_sec_(0.008),
    is_moving_(false),
    ik_solving_(false),
    arm_angle_display_(false)
{
  enable_       = false;
  module_name_  = "manipulation_module";
  control_mode_ = robotis_framework::PositionControl;

  /* arm */
  result_["r_arm_sh_p1"]  = new robotis_framework::DynamixelState();
  result_["l_arm_sh_p1"]  = new robotis_framework::DynamixelState();
  result_["r_arm_sh_r"]   = new robotis_framework::DynamixelState();
  result_["l_arm_sh_r"]   = new robotis_framework::DynamixelState();
  result_["r_arm_sh_p2"]  = new robotis_framework::DynamixelState();
  result_["l_arm_sh_p2"]  = new robotis_framework::DynamixelState();
  result_["r_arm_el_y"]   = new robotis_framework::DynamixelState();
  result_["l_arm_el_y"]   = new robotis_framework::DynamixelState();
  result_["r_arm_wr_r"]   = new robotis_framework::DynamixelState();
  result_["l_arm_wr_r"]   = new robotis_framework::DynamixelState();
  result_["r_arm_wr_y"]   = new robotis_framework::DynamixelState();
  result_["l_arm_wr_y"]   = new robotis_framework::DynamixelState();
  result_["r_arm_wr_p"]   = new robotis_framework::DynamixelState();
  result_["l_arm_wr_p"]   = new robotis_framework::DynamixelState();
  result_["torso_y"]      = new robotis_framework::DynamixelState();

  /* arm */
  joint_name_to_id_["r_arm_sh_p1"] = 1;
  joint_name_to_id_["l_arm_sh_p1"] = 2;
  joint_name_to_id_["r_arm_sh_r"]  = 3;
  joint_name_to_id_["l_arm_sh_r"]  = 4;
  joint_name_to_id_["r_arm_sh_p2"] = 5;
  joint_name_to_id_["l_arm_sh_p2"] = 6;
  joint_name_to_id_["r_arm_el_y"]  = 7;
  joint_name_to_id_["l_arm_el_y"]  = 8;
  joint_name_to_id_["r_arm_wr_r"]  = 9;
  joint_name_to_id_["l_arm_wr_r"]  = 10;
  joint_name_to_id_["r_arm_wr_y"]  = 11;
  joint_name_to_id_["l_arm_wr_y"]  = 12;
  joint_name_to_id_["r_arm_wr_p"]  = 13;
  joint_name_to_id_["l_arm_wr_p"]  = 14;
  joint_name_to_id_["torso_y"]     = 27;

  /* etc */
  joint_name_to_id_["r_arm_end"]   = 35;
  joint_name_to_id_["l_arm_end"]   = 34;

  /* parameter */
  present_joint_position_   = Eigen::VectorXd::Zero(MAX_JOINT_ID+1);
  goal_joint_position_      = Eigen::VectorXd::Zero(MAX_JOINT_ID+1);
  init_joint_position_      = Eigen::VectorXd::Zero(MAX_JOINT_ID+1);

  ik_id_start_  = 0;
  ik_id_end_    = 0;

  ik_target_position_   = Eigen::MatrixXd::Zero(3,1);
  ik_weight_            = Eigen::MatrixXd::Zero(MAX_JOINT_ID+1, 1);
  ik_weight_.fill(1.0);

  robotis_                   = new KinematicsDynamics(WholeBody);
}

ManipulationModule::~ManipulationModule()
{
  queue_thread_.join();
}

void ManipulationModule::initialize(const int control_cycle_msec, robotis_framework::Robot *robot)
{
  control_cycle_sec_ = control_cycle_msec * 0.001;
  queue_thread_      = boost::thread(boost::bind(&ManipulationModule::queueThread, this));

  ros::NodeHandle ros_node;

  /* publish topics */

  // for gui
  status_msg_pub_     = ros_node.advertise<robotis_controller_msgs::StatusMsg>("/robotis/status", 1);
  movement_done_pub_  = ros_node.advertise<std_msgs::String>("/robotis/movement_done", 1);

  std::string _path = ros::package::getPath("thormang3_manipulation_module") + "/config/ik_weight.yaml";
  parseData(_path);
}

void ManipulationModule::parseData(const std::string &path)
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

  YAML::Node ik_weight_node = doc["weight_value"];
  for (YAML::iterator it = ik_weight_node.begin(); it != ik_weight_node.end(); ++it)
  {
    int     id    = it->first.as<int>();
    double  value = it->second.as<double>();

    ik_weight_.coeffRef(id, 0) = value;
  }
}

void ManipulationModule::parseIniPoseData(const std::string &path)
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
  double mov_time = doc["mov_time"].as<double>();
  mov_time_ = mov_time;

  // parse target pose
  YAML::Node tar_pose_node = doc["tar_pose"];
  for (YAML::iterator it = tar_pose_node.begin(); it != tar_pose_node.end(); ++it)
  {
    int     id    = it->first.as<int>();
    double  value = it->second.as<double>();

    init_joint_position_(id) = value * DEGREE2RADIAN;
  }

  all_time_steps_ = int(mov_time_ / control_cycle_sec_) + 1;
  goal_joint_tra_.resize(all_time_steps_, MAX_JOINT_ID + 1);
}

void ManipulationModule::queueThread()
{
  ros::NodeHandle     ros_node;
  ros::CallbackQueue  callback_queue;

  ros_node.setCallbackQueue(&callback_queue);

  /* subscribe topics */
  ros::Subscriber ini_pose_msg_sub = ros_node.subscribe("/robotis/manipulation/ini_pose_msg", 5,
                                                        &ManipulationModule::initPoseMsgCallback, this);
  ros::Subscriber joint_pose_msg_sub = ros_node.subscribe("/robotis/manipulation/joint_pose_msg", 5,
                                                          &ManipulationModule::jointPoseMsgCallback, this);
  ros::Subscriber kinematics_pose_msg_sub = ros_node.subscribe("/robotis/manipulation/kinematics_pose_msg", 5,
                                                               &ManipulationModule::kinematicsPoseMsgCallback, this);

  /* service */
  ros::ServiceServer get_joint_pose_server = ros_node.advertiseService("/robotis/manipulation/get_joint_pose",
                                                                       &ManipulationModule::getJointPoseCallback, this);
  ros::ServiceServer get_kinematics_pose_server = ros_node.advertiseService("/robotis/manipulation/get_kinematics_pose",
                                                                            &ManipulationModule::getKinematicsPoseCallback, this);

  ros::WallDuration duration(control_cycle_sec_);
  while(ros_node.ok())
    callback_queue.callAvailable(duration);
}

void ManipulationModule::initPoseMsgCallback(const std_msgs::String::ConstPtr& msg)
{
  if (enable_ == false)
    return;

  if (is_moving_ == false)
  {
    if (msg->data == "ini_pose")
    {
      // parse initial pose
      std::string ini_pose_path = ros::package::getPath("thormang3_manipulation_module") + "/config/ini_pose.yaml";
      parseIniPoseData(ini_pose_path);

      traj_generate_tread_ = new boost::thread(boost::bind(&ManipulationModule::initPoseTrajGenerateProc, this));
      delete traj_generate_tread_;
    }
  }
  else
  {
    ROS_INFO("previous task is alive");
  }

  movement_done_msg_.data = "manipulation_init";

  return;
}

bool ManipulationModule::getJointPoseCallback(thormang3_manipulation_module_msgs::GetJointPose::Request &req,
                                              thormang3_manipulation_module_msgs::GetJointPose::Response &res)
{
  if (enable_ == false)
    return false;

  for (int name_index = 1; name_index <= MAX_JOINT_ID; name_index++)
  {
    if (robotis_->thormang3_link_data_[name_index]->name_ == req.joint_name)
    {
      res.joint_value = goal_joint_position_(name_index);
      return true;
    }
  }

  return false;
}

bool ManipulationModule::getKinematicsPoseCallback(thormang3_manipulation_module_msgs::GetKinematicsPose::Request &req,
                                                   thormang3_manipulation_module_msgs::GetKinematicsPose::Response &res)
{
  if (enable_ == false)
    return false;

  int end_index;

  if (req.group_name == "left_arm")
    end_index = ID_L_ARM_END;
  else if (req.group_name == "right_arm")
    end_index = ID_R_ARM_END;
  else if (req.group_name == "left_arm_with_torso")
    end_index = ID_L_ARM_END;
  else if (req.group_name == "right_arm_with_torso")
    end_index = ID_R_ARM_END;
  else
    return false;

  res.group_pose.position.x = robotis_->thormang3_link_data_[end_index]->position_.coeff(0, 0);
  res.group_pose.position.y = robotis_->thormang3_link_data_[end_index]->position_.coeff(1, 0);
  res.group_pose.position.z = robotis_->thormang3_link_data_[end_index]->position_.coeff(2, 0);

  Eigen::Quaterniond quaternion = robotis_framework::convertRotationToQuaternion(robotis_->thormang3_link_data_[end_index]->orientation_);

  res.group_pose.orientation.w = quaternion.w();
  res.group_pose.orientation.x = quaternion.x();
  res.group_pose.orientation.y = quaternion.y();
  res.group_pose.orientation.z = quaternion.z();

  return true;
}

void ManipulationModule::kinematicsPoseMsgCallback(const thormang3_manipulation_module_msgs::KinematicsPose::ConstPtr& msg)
{
  if (enable_ == false)
    return;

  movement_done_msg_.data = "manipulation";

  goal_kinematics_pose_msg_ = *msg;

  if (goal_kinematics_pose_msg_.name == "left_arm")
  {
    ik_id_start_  = ID_L_ARM_START;
    ik_id_end_    = ID_L_ARM_END;
  }
  else if (goal_kinematics_pose_msg_.name == "right_arm")
  {
    ik_id_start_  = ID_R_ARM_START;
    ik_id_end_    = ID_R_ARM_END;
  }
  else if (goal_kinematics_pose_msg_.name == "left_arm_with_torso")
  {
    ik_id_start_  = ID_TORSO;
    ik_id_end_    = ID_L_ARM_END;
  }
  else if (goal_kinematics_pose_msg_.name == "right_arm_with_torso")
  {
    ik_id_start_  = ID_TORSO;
    ik_id_end_    = ID_R_ARM_END;
  }

  if (is_moving_ == false)
  {
    traj_generate_tread_ = new boost::thread(boost::bind(&ManipulationModule::taskTrajGenerateProc, this));
    delete traj_generate_tread_;
  }
  else
  {
    ROS_INFO("previous task is alive");
  }

  return;
}

void ManipulationModule::jointPoseMsgCallback(const thormang3_manipulation_module_msgs::JointPose::ConstPtr& msg)
{
  if (enable_ == false)
    return;

  goal_joint_pose_msg_ = *msg;

  if (is_moving_ == false)
  {
    traj_generate_tread_ = new boost::thread(boost::bind(&ManipulationModule::jointTrajGenerateProc, this));
    delete traj_generate_tread_;
  }
  else
    ROS_INFO("previous task is alive");

  return;
}

void ManipulationModule::initPoseTrajGenerateProc()
{
  for (int id = 1; id <= MAX_JOINT_ID; id++)
  {
    double ini_value = goal_joint_position_(id);
    double tar_value = init_joint_position_(id);

    Eigen::MatrixXd tra = robotis_framework::calcMinimumJerkTra(ini_value, 0.0, 0.0,
                                                                tar_value, 0.0, 0.0,
                                                                control_cycle_sec_,
                                                                mov_time_);

    goal_joint_tra_.block(0, id, all_time_steps_, 1) = tra;
  }

  cnt_        = 0;
  is_moving_  = true;

  ROS_INFO("[start] send trajectory");
}

void ManipulationModule::jointTrajGenerateProc()
{
  if (goal_joint_pose_msg_.time <= 0.0)
  {
    /* set movement time */
    double tol        = 10 * DEGREE2RADIAN; // rad per sec
    double mov_time   = 2.0;

    int    id    = joint_name_to_id_[goal_joint_pose_msg_.name];

    double ini_value  = goal_joint_position_(id);
    double tar_value  = goal_joint_pose_msg_.value;
    double diff       = fabs(tar_value - ini_value);

    mov_time_ = diff / tol;
    int _all_time_steps = int(floor((mov_time_ / control_cycle_sec_) + 1.0));
    mov_time_ = double(_all_time_steps - 1) * control_cycle_sec_;

    if (mov_time_ < mov_time)
      mov_time_ = mov_time;
  }
  else
  {
    mov_time_ = goal_joint_pose_msg_.time;
  }

  all_time_steps_ = int(mov_time_ / control_cycle_sec_) + 1;

  goal_joint_tra_.resize(all_time_steps_, MAX_JOINT_ID + 1);

  /* calculate joint trajectory */
  for (int id = 1; id <= MAX_JOINT_ID; id++)
  {
    double ini_value = goal_joint_position_(id);
    double tar_value = goal_joint_position_(id);

    if (robotis_->thormang3_link_data_[id]->name_ == goal_joint_pose_msg_.name)
      tar_value = goal_joint_pose_msg_.value;

    Eigen::MatrixXd tra = robotis_framework::calcMinimumJerkTra(ini_value, 0.0, 0.0, tar_value, 0.0, 0.0,
                                                                control_cycle_sec_,
                                                                mov_time_);

    goal_joint_tra_.block(0, id, all_time_steps_, 1) = tra;
  }

  cnt_        = 0;
  is_moving_  = true;

  ROS_INFO("[start] send trajectory");
}

void ManipulationModule::taskTrajGenerateProc()
{
  if (goal_kinematics_pose_msg_.time <= 0.0)
  {
    /* set movement time */
    double tol      = 0.1; // m per sec
    double mov_time = 2.0;

    double diff     = sqrt(
                          pow(robotis_->thormang3_link_data_[ik_id_end_]->position_.coeff(0,0) - goal_kinematics_pose_msg_.pose.position.x, 2)
                        + pow(robotis_->thormang3_link_data_[ik_id_end_]->position_.coeff(1,0) - goal_kinematics_pose_msg_.pose.position.y, 2)
                        + pow(robotis_->thormang3_link_data_[ik_id_end_]->position_.coeff(2,0) - goal_kinematics_pose_msg_.pose.position.z, 2)
                      );

    mov_time_ = diff / tol;
    int all_time_steps = int(floor((mov_time_ / control_cycle_sec_) + 1.0));
    mov_time_ = double(all_time_steps - 1) * control_cycle_sec_;

    if (mov_time_ < mov_time)
      mov_time_ = mov_time;
  }
  else
  {
    mov_time_ = goal_kinematics_pose_msg_.time;
  }

  all_time_steps_ = int(mov_time_ / control_cycle_sec_) + 1;
  goal_task_tra_.resize(all_time_steps_, 3);

  /* calculate trajectory */
  for (int dim = 0; dim < 3; dim++)
  {
    double ini_value = robotis_->thormang3_link_data_[ik_id_end_]->position_.coeff(dim, 0);
    double tar_value;
    if (dim == 0)
      tar_value = goal_kinematics_pose_msg_.pose.position.x;
    else if (dim == 1)
      tar_value = goal_kinematics_pose_msg_.pose.position.y;
    else if (dim == 2)
      tar_value = goal_kinematics_pose_msg_.pose.position.z;

    Eigen::MatrixXd tra = robotis_framework::calcMinimumJerkTra(ini_value, 0.0, 0.0, tar_value, 0.0, 0.0,
                                                                control_cycle_sec_,
                                                                mov_time_);

    goal_task_tra_.block(0, dim, all_time_steps_, 1) = tra;
  }

  cnt_          = 0;
  is_moving_    = true;
  ik_solving_   = true;

  ROS_INFO("[start] send trajectory");
}

void ManipulationModule::setInverseKinematics(int cnt, Eigen::MatrixXd start_rotation)
{
  for (int dim = 0; dim < 3; dim++)
    ik_target_position_.coeffRef(dim, 0) = goal_task_tra_.coeff(cnt, dim);

  Eigen::Quaterniond start_quaternion = robotis_framework::convertRotationToQuaternion(start_rotation);

  Eigen::Quaterniond target_quaternion(goal_kinematics_pose_msg_.pose.orientation.w,
                                       goal_kinematics_pose_msg_.pose.orientation.x,
                                       goal_kinematics_pose_msg_.pose.orientation.y,
                                       goal_kinematics_pose_msg_.pose.orientation.z);

  double count = (double) cnt / (double) all_time_steps_;

  Eigen::Quaterniond _quaternion = start_quaternion.slerp(count, target_quaternion);

  ik_target_rotation_ = robotis_framework::convertQuaternionToRotation(_quaternion);
}

void ManipulationModule::process(std::map<std::string, robotis_framework::Dynamixel *> dxls,
                                 std::map<std::string, double> sensors)
{
  if (enable_ == false)
    return;

  /*----- write curr position -----*/

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

    double joint_curr_position = dxl->dxl_state_->present_position_;
    double joint_goal_position = dxl->dxl_state_->goal_position_;

    present_joint_position_(joint_name_to_id_[joint_name]) = joint_curr_position;
    goal_joint_position_(joint_name_to_id_[joint_name]) = joint_goal_position;
  }

  /*----- forward kinematics -----*/
  for (int id = 1; id <= MAX_JOINT_ID; id++)
    robotis_->thormang3_link_data_[id]->joint_angle_ = goal_joint_position_(id);

  robotis_->calcForwardKinematics(0);

  /* ----- send trajectory ----- */

  if (is_moving_ == true)
  {
    if (cnt_ == 0)
    {
      publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Start Trajectory");

      ik_start_rotation_ = robotis_->thormang3_link_data_[ik_id_end_]->orientation_;
    }

    if (ik_solving_ == true)
    {
      /* ----- inverse kinematics ----- */
      setInverseKinematics(cnt_, ik_start_rotation_);

      int     max_iter    = 30;
      double  ik_tol      = 1e-3;
      bool    ik_success  = robotis_->calcInverseKinematics(ik_id_start_,
                                                            ik_id_end_,
                                                            ik_target_position_,
                                                            ik_target_rotation_,
                                                            max_iter, ik_tol,
                                                            ik_weight_);

      if (ik_success == true)
      {
        for (int id = 1; id <= MAX_JOINT_ID; id++)
          goal_joint_position_(id) = robotis_->thormang3_link_data_[id]->joint_angle_;
      }
      else
      {
        ROS_INFO("----- ik failed -----");
        ROS_INFO("[end] send trajectory");

        publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "IK Failed");

        is_moving_  = false;
        ik_solving_   = false;
        cnt_        = 0;

        movement_done_msg_.data = "manipulation_fail";
        movement_done_pub_.publish(movement_done_msg_);
        movement_done_msg_.data = "";
      }
    }
    else
    {
      for (int id = 1; id <= MAX_JOINT_ID; id++)
        goal_joint_position_(id) = goal_joint_tra_(cnt_, id);
    }

    cnt_++;
  }

  /*----- set joint data -----*/
  for (std::map<std::string, robotis_framework::DynamixelState *>::iterator state_iter = result_.begin();
      state_iter != result_.end(); state_iter++)
  {
    std::string joint_name = state_iter->first;
    result_[joint_name]->goal_position_ = goal_joint_position_(joint_name_to_id_[joint_name]);
  }

  /*---------- initialize count number ----------*/
  if (is_moving_ == true)
  {
    if (cnt_ >= all_time_steps_)
    {
      ROS_INFO("[end] send trajectory");

      publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "End Trajectory");

      is_moving_  = false;
      ik_solving_ = false;
      cnt_        = 0;

      movement_done_pub_.publish(movement_done_msg_);
      movement_done_msg_.data = "";

      if (arm_angle_display_ == true)
      {
        ROS_INFO("l_arm_sh_p1 : %f", goal_joint_position_(joint_name_to_id_["l_arm_sh_p1"]) * RADIAN2DEGREE );
        ROS_INFO("l_arm_sh_r  : %f", goal_joint_position_(joint_name_to_id_["l_arm_sh_r"])  * RADIAN2DEGREE );
        ROS_INFO("l_arm_sh_p2 : %f", goal_joint_position_(joint_name_to_id_["l_arm_sh_p2"]) * RADIAN2DEGREE );
        ROS_INFO("l_arm_el_y  : %f", goal_joint_position_(joint_name_to_id_["l_arm_el_y"])  * RADIAN2DEGREE );
        ROS_INFO("l_arm_wr_r  : %f", goal_joint_position_(joint_name_to_id_["l_arm_wr_r"])  * RADIAN2DEGREE );
        ROS_INFO("l_arm_wr_y  : %f", goal_joint_position_(joint_name_to_id_["l_arm_wr_y"])  * RADIAN2DEGREE );
        ROS_INFO("l_arm_wr_p  : %f", goal_joint_position_(joint_name_to_id_["l_arm_wr_p"])  * RADIAN2DEGREE );

        ROS_INFO("r_arm_sh_p1 : %f", goal_joint_position_(joint_name_to_id_["r_arm_sh_p1"]) * RADIAN2DEGREE );
        ROS_INFO("r_arm_sh_r  : %f", goal_joint_position_(joint_name_to_id_["r_arm_sh_r"])  * RADIAN2DEGREE );
        ROS_INFO("r_arm_sh_p2 : %f", goal_joint_position_(joint_name_to_id_["r_arm_sh_p2"]) * RADIAN2DEGREE );
        ROS_INFO("r_arm_el_y  : %f", goal_joint_position_(joint_name_to_id_["r_arm_el_y"])  * RADIAN2DEGREE );
        ROS_INFO("r_arm_wr_r  : %f", goal_joint_position_(joint_name_to_id_["r_arm_wr_r"])  * RADIAN2DEGREE );
        ROS_INFO("r_arm_wr_y  : %f", goal_joint_position_(joint_name_to_id_["r_arm_wr_y"])  * RADIAN2DEGREE );
        ROS_INFO("r_arm_wr_p  : %f", goal_joint_position_(joint_name_to_id_["r_arm_wr_p"])  * RADIAN2DEGREE );
      }

    }
  }
}

void ManipulationModule::stop()
{
  is_moving_  = false;
  ik_solving_   = false;
  cnt_        = 0;

  return;
}

bool ManipulationModule::isRunning()
{
  return is_moving_;
}

void ManipulationModule::publishStatusMsg(unsigned int type, std::string msg)
{
  robotis_controller_msgs::StatusMsg status;
  status.header.stamp = ros::Time::now();
  status.type         = type;
  status.module_name  = "Manipulation";
  status.status_msg   = msg;

  status_msg_pub_.publish(status);
}
