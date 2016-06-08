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
 *  Created on: June 7, 2016
 *      Author: sch
 */

#include "thormang3_manipulation_module/manipulation_module.h"

using namespace thormang3;

ManipulationModule::ManipulationModule() :
    control_cycle_msec_(0)
{
  enable_ = false;
  module_name_ = "manipulation_module";
  control_mode_ = robotis_framework::PositionControl;

  /* arm */
  result_["r_arm_sh_p1"] = new robotis_framework::DynamixelState();
  result_["l_arm_sh_p1"] = new robotis_framework::DynamixelState();
  result_["r_arm_sh_r"] = new robotis_framework::DynamixelState();
  result_["l_arm_sh_r"] = new robotis_framework::DynamixelState();
  result_["r_arm_sh_p2"] = new robotis_framework::DynamixelState();
  result_["l_arm_sh_p2"] = new robotis_framework::DynamixelState();
  result_["r_arm_el_y"] = new robotis_framework::DynamixelState();
  result_["l_arm_el_y"] = new robotis_framework::DynamixelState();
  result_["r_arm_wr_r"] = new robotis_framework::DynamixelState();
  result_["l_arm_wr_r"] = new robotis_framework::DynamixelState();
  result_["r_arm_wr_y"] = new robotis_framework::DynamixelState();
  result_["l_arm_wr_y"] = new robotis_framework::DynamixelState();
  result_["r_arm_wr_p"] = new robotis_framework::DynamixelState();
  result_["l_arm_wr_p"] = new robotis_framework::DynamixelState();
  result_["torso_y"] = new robotis_framework::DynamixelState();
  result_["r_arm_grip"] = new robotis_framework::DynamixelState();
  result_["l_arm_grip"] = new robotis_framework::DynamixelState();

  /* arm */
  joint_name_to_id["r_arm_sh_p1"] = 1;
  joint_name_to_id["l_arm_sh_p1"] = 2;
  joint_name_to_id["r_arm_sh_r"] = 3;
  joint_name_to_id["l_arm_sh_r"] = 4;
  joint_name_to_id["r_arm_sh_p2"] = 5;
  joint_name_to_id["l_arm_sh_p2"] = 6;
  joint_name_to_id["r_arm_el_y"] = 7;
  joint_name_to_id["l_arm_el_y"] = 8;
  joint_name_to_id["r_arm_wr_r"] = 9;
  joint_name_to_id["l_arm_wr_r"] = 10;
  joint_name_to_id["r_arm_wr_y"] = 11;
  joint_name_to_id["l_arm_wr_y"] = 12;
  joint_name_to_id["r_arm_wr_p"] = 13;
  joint_name_to_id["l_arm_wr_p"] = 14;
  joint_name_to_id["torso_y"] = 27;
  joint_name_to_id["r_arm_grip"] = 31;
  joint_name_to_id["l_arm_grip"] = 30;

  /* etc */
  joint_name_to_id["r_arm_end"] = 35;
  joint_name_to_id["l_arm_end"] = 34;

  /* parameter */
  Humanoid = new KinematicsDynamics(WholeBody);
  Robotis = new ROBOTIS_MANIPULATION::RobotisState();
  JointState = new ManipulationJointState();
}

ManipulationModule::~ManipulationModule()
{
  queue_thread_.join();
}

void ManipulationModule::initialize(const int control_cycle_msec, robotis_framework::Robot *robot)
{
  control_cycle_msec_ = control_cycle_msec;
  queue_thread_ = boost::thread(boost::bind(&ManipulationModule::QueueThread, this));

  ros::NodeHandle _ros_node;

  /* publish topics */

  // for gui
  status_msg_pub_ = _ros_node.advertise<robotis_controller_msgs::StatusMsg>("/robotis/status", 1);

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

  YAML::Node _ik_weight_node = doc["weight_value"];
  for (YAML::iterator _it = _ik_weight_node.begin(); _it != _ik_weight_node.end(); ++_it)
  {
    int _id = _it->first.as<int>();
    double _value = _it->second.as<double>();

    Robotis->ik_weight.coeffRef(_id, 0) = _value;
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
  double _mov_time;
  _mov_time = doc["mov_time"].as<double>();

  Robotis->mov_time = _mov_time;

  // parse target pose
  YAML::Node _tar_pose_node = doc["tar_pose"];
  for (YAML::iterator _it = _tar_pose_node.begin(); _it != _tar_pose_node.end(); ++_it)
  {
    int _id;
    double _value;

    _id = _it->first.as<int>();
    _value = _it->second.as<double>();

    Robotis->joint_ini_pose.coeffRef(_id, 0) = _value * DEGREE2RADIAN;
  }

  Robotis->all_time_steps = int(Robotis->mov_time / Robotis->smp_time) + 1;
  Robotis->calc_joint_tra.resize(Robotis->all_time_steps, MAX_JOINT_ID + 1);
}

void ManipulationModule::QueueThread()
{
  ros::NodeHandle _ros_node;
  ros::CallbackQueue _callback_queue;

  _ros_node.setCallbackQueue(&_callback_queue);

  /* subscribe topics */
  ros::Subscriber ini_pose_msg_sub = _ros_node.subscribe("/robotis/manipulation/ini_pose_msg", 5,
      &ManipulationModule::IniPoseMsgCallback, this);
  ros::Subscriber joint_pose_msg_sub = _ros_node.subscribe("/robotis/manipulation/joint_pose_msg", 5,
      &ManipulationModule::JointPoseMsgCallback, this);
  ros::Subscriber kinematics_pose_msg_sub = _ros_node.subscribe("/robotis/manipulation/kinematics_pose_msg", 5,
      &ManipulationModule::KinematicsPoseMsgCallback, this);

  /* service */
  ros::ServiceServer get_joint_pose_server = _ros_node.advertiseService("/robotis/manipulation/get_joint_pose",
      &ManipulationModule::GetJointPoseCallback, this);
  ros::ServiceServer get_kinematics_pose_server = _ros_node.advertiseService(
      "/robotis/manipulation/get_kinematics_pose", &ManipulationModule::GetKinematicsPoseCallback, this);

  while (_ros_node.ok())
  {
    _callback_queue.callAvailable();
    usleep(100);
  }
}

void ManipulationModule::IniPoseMsgCallback(const std_msgs::String::ConstPtr& msg)
{
  if (enable_ == false)
    return;

  if (Robotis->is_moving == false)
  {
    if (msg->data == "ini_pose")
    {
      // parse initial pose
      std::string _ini_pose_path = ros::package::getPath("thormang3_manipulation_module") + "/config/ini_pose.yaml";
      parseIniPoseData(_ini_pose_path);

      tra_gene_tread_ = new boost::thread(boost::bind(&ManipulationModule::IniposeTraGeneProc, this));
      delete tra_gene_tread_;
    }
  }
  else
    ROS_INFO("previous task is alive");

  return;
}

bool ManipulationModule::GetJointPoseCallback(thormang3_manipulation_module_msgs::GetJointPose::Request &req,
    thormang3_manipulation_module_msgs::GetJointPose::Response &res)
{
  if (enable_ == false)
    return false;

  for (int _name_index = 1; _name_index <= MAX_JOINT_ID; _name_index++)
  {
    if (Humanoid->thormang3_link_data_[_name_index]->name_ == req.joint_name)
    {
      res.joint_value = JointState->goal_joint_state[_name_index].position;
      return true;
    }
  }

  return false;
}

bool ManipulationModule::GetKinematicsPoseCallback(thormang3_manipulation_module_msgs::GetKinematicsPose::Request &req,
    thormang3_manipulation_module_msgs::GetKinematicsPose::Response &res)
{
  if (enable_ == false)
    return false;

  int _end_index;

  if (req.group_name == "left_arm")
    _end_index = ID_L_ARM_END;
  else if (req.group_name == "right_arm")
    _end_index = ID_R_ARM_END;
  else if (req.group_name == "left_arm_with_torso")
    _end_index = ID_L_ARM_END;
  else if (req.group_name == "right_arm_with_torso")
    _end_index = ID_R_ARM_END;
  else
    return false;

  res.group_pose.position.x = Humanoid->thormang3_link_data_[_end_index]->position_.coeff(0, 0);
  res.group_pose.position.y = Humanoid->thormang3_link_data_[_end_index]->position_.coeff(1, 0);
  res.group_pose.position.z = Humanoid->thormang3_link_data_[_end_index]->position_.coeff(2, 0);

  Eigen::Quaterniond _quaternion = robotis_framework::convertRotationToQuaternion(Humanoid->thormang3_link_data_[_end_index]->orientation_);

  res.group_pose.orientation.w = _quaternion.w();
  res.group_pose.orientation.x = _quaternion.x();
  res.group_pose.orientation.y = _quaternion.y();
  res.group_pose.orientation.z = _quaternion.z();

  return true;
}

void ManipulationModule::KinematicsPoseMsgCallback(
    const thormang3_manipulation_module_msgs::KinematicsPose::ConstPtr& msg)
{
  if (enable_ == false)
    return;

  Robotis->goal_kinematics_pose_msg = *msg;

  if (Robotis->goal_kinematics_pose_msg.name == "left_arm")
  {
    Robotis->ik_id_start = ID_L_ARM_START;
    Robotis->ik_id_end = ID_L_ARM_END;
  }
  else if (Robotis->goal_kinematics_pose_msg.name == "right_arm")
  {
    Robotis->ik_id_start = ID_R_ARM_START;
    Robotis->ik_id_end = ID_R_ARM_END;
  }
  else if (Robotis->goal_kinematics_pose_msg.name == "left_arm_with_torso")
  {
    Robotis->ik_id_start = ID_TORSO;
    Robotis->ik_id_end = ID_L_ARM_END;
  }
  else if (Robotis->goal_kinematics_pose_msg.name == "right_arm_with_torso")
  {
    Robotis->ik_id_start = ID_TORSO;
    Robotis->ik_id_end = ID_R_ARM_END;
  }

  if (Robotis->is_moving == false)
  {
    tra_gene_tread_ = new boost::thread(boost::bind(&ManipulationModule::TaskTraGeneProc, this));
    delete tra_gene_tread_;
  }
  else
    ROS_INFO("previous task is alive");

  return;
}

void ManipulationModule::JointPoseMsgCallback(const thormang3_manipulation_module_msgs::JointPose::ConstPtr& msg)
{
  if (enable_ == false)
    return;

  Robotis->goal_joint_pose_msg = *msg;

  if (Robotis->is_moving == false)
  {
    tra_gene_tread_ = new boost::thread(boost::bind(&ManipulationModule::JointTraGeneProc, this));
    delete tra_gene_tread_;
  }
  else
    ROS_INFO("previous task is alive");

  return;
}

void ManipulationModule::IniposeTraGeneProc()
{
  for (int id = 1; id <= MAX_JOINT_ID; id++)
  {
    double ini_value = JointState->goal_joint_state[id].position;
    double tar_value = Robotis->joint_ini_pose.coeff(id, 0);

    Eigen::MatrixXd tra = robotis_framework::calcMinimumJerkTra(ini_value, 0.0, 0.0, tar_value, 0.0, 0.0, Robotis->smp_time,
        Robotis->mov_time);

    Robotis->calc_joint_tra.block(0, id, Robotis->all_time_steps, 1) = tra;
  }

  Robotis->cnt = 0;
  Robotis->is_moving = true;

  ROS_INFO("[start] send trajectory");
}

void ManipulationModule::JointTraGeneProc()
{
  if (Robotis->goal_joint_pose_msg.time <= 0.0)
  {
    /* set movement time */
    double _tol = 10 * DEGREE2RADIAN; // rad per sec
    double _mov_time = 2.0;

    int _ctrl_id = joint_name_to_id[Robotis->goal_joint_pose_msg.name];

    double _ini_value = JointState->goal_joint_state[_ctrl_id].position;
    double _tar_value = Robotis->goal_joint_pose_msg.value;
    double _diff = fabs(_tar_value - _ini_value);

    Robotis->mov_time = _diff / _tol;
    int _all_time_steps = int(floor((Robotis->mov_time / Robotis->smp_time) + 1.0));
    Robotis->mov_time = double(_all_time_steps - 1) * Robotis->smp_time;

    if (Robotis->mov_time < _mov_time)
      Robotis->mov_time = _mov_time;
  }
  else
    Robotis->mov_time = Robotis->goal_joint_pose_msg.time;

  Robotis->all_time_steps = int(Robotis->mov_time / Robotis->smp_time) + 1;

  Robotis->calc_joint_tra.resize(Robotis->all_time_steps, MAX_JOINT_ID + 1);

  /* calculate joint trajectory */
  for (int id = 1; id <= MAX_JOINT_ID; id++)
  {
    double ini_value = JointState->goal_joint_state[id].position;
    double tar_value = JointState->goal_joint_state[id].position;

    if (Humanoid->thormang3_link_data_[id]->name_ == Robotis->goal_joint_pose_msg.name)
      tar_value = Robotis->goal_joint_pose_msg.value;

    Eigen::MatrixXd tra = robotis_framework::calcMinimumJerkTra(ini_value, 0.0, 0.0, tar_value, 0.0, 0.0, Robotis->smp_time,
        Robotis->mov_time);

    Robotis->calc_joint_tra.block(0, id, Robotis->all_time_steps, 1) = tra;
  }

  Robotis->cnt = 0;
  Robotis->is_moving = true;

  ROS_INFO("[start] send trajectory");
}

void ManipulationModule::TaskTraGeneProc()
{
  if (Robotis->goal_joint_pose_msg.time <= 0.0)
  {
    /* set movement time */
    double _tol = 0.1; // m per sec
    double _mov_time = 2.0;

    double _diff = sqrt(
        pow(
            Humanoid->thormang3_link_data_[Robotis->ik_id_end]->position_.coeff(0, 0)
                - Robotis->goal_kinematics_pose_msg.pose.position.x, 2)
            + pow(
                Humanoid->thormang3_link_data_[Robotis->ik_id_end]->position_.coeff(1, 0)
                    - Robotis->goal_kinematics_pose_msg.pose.position.y, 2)
            + pow(
                Humanoid->thormang3_link_data_[Robotis->ik_id_end]->position_.coeff(2, 0)
                    - Robotis->goal_kinematics_pose_msg.pose.position.z, 2));

    Robotis->mov_time = _diff / _tol;
    int _all_time_steps = int(floor((Robotis->mov_time / Robotis->smp_time) + 1.0));
    Robotis->mov_time = double(_all_time_steps - 1) * Robotis->smp_time;

    if (Robotis->mov_time < _mov_time)
      Robotis->mov_time = _mov_time;
  }
  else
    Robotis->mov_time = Robotis->goal_joint_pose_msg.time;

  Robotis->all_time_steps = int(Robotis->mov_time / Robotis->smp_time) + 1;

  Robotis->calc_task_tra.resize(Robotis->all_time_steps, 3);

  /* calculate trajectory */
  for (int dim = 0; dim < 3; dim++)
  {
    double ini_value = Humanoid->thormang3_link_data_[Robotis->ik_id_end]->position_.coeff(dim, 0);
    double tar_value;
    if (dim == 0)
      tar_value = Robotis->goal_kinematics_pose_msg.pose.position.x;
    else if (dim == 1)
      tar_value = Robotis->goal_kinematics_pose_msg.pose.position.y;
    else if (dim == 2)
      tar_value = Robotis->goal_kinematics_pose_msg.pose.position.z;

    Eigen::MatrixXd tra = robotis_framework::calcMinimumJerkTra(ini_value, 0.0, 0.0, tar_value, 0.0, 0.0, Robotis->smp_time,
        Robotis->mov_time);

    Robotis->calc_task_tra.block(0, dim, Robotis->all_time_steps, 1) = tra;
  }

  Robotis->cnt = 0;
  Robotis->is_moving = true;
  Robotis->ik_solve = true;

  ROS_INFO("[start] send trajectory");
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
    std::string _joint_name = state_iter->first;

    robotis_framework::Dynamixel *_dxl = NULL;
    std::map<std::string, robotis_framework::Dynamixel*>::iterator _dxl_it = dxls.find(_joint_name);
    if (_dxl_it != dxls.end())
      _dxl = _dxl_it->second;
    else
      continue;

    double _joint_curr_position = _dxl->dxl_state_->present_position_;
    double _joint_goal_position = _dxl->dxl_state_->goal_position_;

    JointState->curr_joint_state[joint_name_to_id[_joint_name]].position = _joint_curr_position;
    JointState->goal_joint_state[joint_name_to_id[_joint_name]].position = _joint_goal_position;
  }

  /*----- forward kinematics -----*/

  for (int id = 1; id <= MAX_JOINT_ID; id++)
    Humanoid->thormang3_link_data_[id]->joint_angle_ = JointState->goal_joint_state[id].position;

  Humanoid->calcForwardKinematics(0);

  /* ----- send trajectory ----- */

  if (Robotis->is_moving == true)
  {
    if (Robotis->cnt == 0)
    {
      PublishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Start Trajectory");

      Robotis->ik_start_rotation = Humanoid->thormang3_link_data_[Robotis->ik_id_end]->orientation_;
    }

    if (Robotis->ik_solve == true)
    {
      /* ----- inverse kinematics ----- */
      Robotis->setInverseKinematics(Robotis->cnt, Robotis->ik_start_rotation);

      int max_iter = 30;
      double ik_tol = 1e-3;
      bool ik_success = Humanoid->calcInverseKinematics(Robotis->ik_id_start, Robotis->ik_id_end,
          Robotis->ik_target_position, Robotis->ik_target_rotation, max_iter, ik_tol, Robotis->ik_weight);

      if (ik_success == true)
      {
        for (int id = 1; id <= MAX_JOINT_ID; id++)
          JointState->goal_joint_state[id].position = Humanoid->thormang3_link_data_[id]->joint_angle_;
      }
      else
      {
        ROS_INFO("----- ik failed -----");
        ROS_INFO("[end] send trajectory");

        PublishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "IK Failed");

        Robotis->is_moving = false;
        Robotis->ik_solve = false;
        Robotis->cnt = 0;
      }
    }
    else
    {
      for (int id = 1; id <= MAX_JOINT_ID; id++)
        JointState->goal_joint_state[id].position = Robotis->calc_joint_tra(Robotis->cnt, id);
    }

    Robotis->cnt++;
  }

  /*----- set joint data -----*/
  for (std::map<std::string, robotis_framework::DynamixelState *>::iterator state_iter = result_.begin();
      state_iter != result_.end(); state_iter++)
  {
    std::string _joint_name = state_iter->first;
    result_[_joint_name]->goal_position_ = JointState->goal_joint_state[joint_name_to_id[_joint_name]].position;
  }

  /*---------- initialize count number ----------*/
  if (Robotis->is_moving == true)
  {
    if (Robotis->cnt >= Robotis->all_time_steps)
    {
      ROS_INFO("[end] send trajectory");

      PublishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "End Trajectory");

      Robotis->is_moving = false;
      Robotis->ik_solve = false;
      Robotis->cnt = 0;
    }
  }

}

void ManipulationModule::stop()
{
  Robotis->is_moving = false;
  Robotis->ik_solve = false;
  Robotis->cnt = 0;

  return;
}

bool ManipulationModule::isRunning()
{
  return Robotis->is_moving;
}

void ManipulationModule::PublishStatusMsg(unsigned int type, std::string msg)
{
  robotis_controller_msgs::StatusMsg _status;
  _status.header.stamp = ros::Time::now();
  _status.type = type;
  _status.module_name = "Manipulation";
  _status.status_msg = msg;

  status_msg_pub_.publish(_status);
}
