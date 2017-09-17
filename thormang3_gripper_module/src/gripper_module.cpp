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

#include <stdio.h>
#include "thormang3_gripper_module/gripper_module.h"

using namespace thormang3;

GripperModule::GripperModule()
  : control_cycle_sec_(0.008),
    is_moving_(false)
{
  enable_       = false;
  module_name_  = "gripper_module";
  control_mode_ = robotis_framework::PositionControl;

  /* gripper */
  result_["r_arm_grip"] = new robotis_framework::DynamixelState();
  result_["l_arm_grip"] = new robotis_framework::DynamixelState();

  /* gripper */
  joint_name_to_id_["r_arm_grip"] = 0;
  joint_name_to_id_["l_arm_grip"] = 1;

  /* ----- parameter initialization ----- */
  present_joint_position_ = Eigen::VectorXd::Zero(NUM_GRIPPER_JOINTS);
  present_joint_velocity_ = Eigen::VectorXd::Zero(NUM_GRIPPER_JOINTS);
  goal_joint_position_    = Eigen::VectorXd::Zero(NUM_GRIPPER_JOINTS);
}

GripperModule::~GripperModule()
{
  queue_thread_.join();
}

void GripperModule::initialize(const int control_cycle_msec, robotis_framework::Robot *robot)
{
  control_cycle_sec_ = control_cycle_msec * 0.001;
  queue_thread_ = boost::thread(boost::bind(&GripperModule::queueThread, this));
}

void GripperModule::queueThread()
{
  ros::NodeHandle    ros_node;
  ros::CallbackQueue callback_queue;

  ros_node.setCallbackQueue(&callback_queue);

  /* publish topics */
  status_msg_pub_ = ros_node.advertise<robotis_controller_msgs::StatusMsg>("/robotis/status", 1);
  set_ctrl_module_pub_ = ros_node.advertise<std_msgs::String>("/robotis/enable_ctrl_module", 1);
  goal_torque_limit_pub_ = ros_node.advertise<robotis_controller_msgs::SyncWriteItem>("/robotis/sync_write_item", 1);
  movement_done_pub_  = ros_node.advertise<std_msgs::String>("/robotis/movement_done", 1);

  /* subscribe topics */
  ros::Subscriber set_mode_msg_sub = ros_node.subscribe("/robotis/gripper/set_mode_msg", 5,
                                                        &GripperModule::setModeMsgCallback, this);
  ros::Subscriber joint_pose_msg_sub = ros_node.subscribe("/robotis/gripper/joint_pose_msg", 5,
                                                          &GripperModule::setJointPoseMsgCallback, this);

  /* service */

  ros::WallDuration duration(control_cycle_sec_);
  while(ros_node.ok())
    callback_queue.callAvailable(duration);
}

void GripperModule::setModeMsgCallback(const std_msgs::String::ConstPtr& msg)
{
  //  ROS_INFO("--- Set Torque Control Mode ---");
  std_msgs::String str_msg;
  str_msg.data = "gripper_module";
  set_ctrl_module_pub_.publish(str_msg);
  return;
}

void GripperModule::setJointPoseMsgCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  if(enable_ == false)
    return;

  goal_joint_pose_msg_ = *msg;

  if (is_moving_ == false)
  {
    setTorqueLimit();

    movement_done_msg_.data = "gripper";

    tra_gene_tread_ = new boost::thread(boost::bind(&GripperModule::traGeneProcJointSpace, this));
    delete tra_gene_tread_;
  }
  else
    ROS_INFO("previous task is alive");

  return;
}

void GripperModule::traGeneProcJointSpace()
{
  mov_time_ = 1.5;
  int all_time_steps = int(floor((mov_time_/control_cycle_sec_) + 1 ));
  mov_time_ = double (all_time_steps - 1) * control_cycle_sec_;

  all_time_steps_ = int(mov_time_ / control_cycle_sec_) + 1;
  goal_joint_tra_.resize(all_time_steps_, NUM_GRIPPER_JOINTS + 1);

  /* calculate joint trajectory */
  for (int dim=0; dim<NUM_GRIPPER_JOINTS; dim++)
  {
    double ini_value = goal_joint_position_(dim);
    double tar_value = goal_joint_position_(dim);

    Eigen::MatrixXd tra =
        robotis_framework::calcMinimumJerkTra(ini_value, 0.0, 0.0,
                                              tar_value , 0.0 , 0.0 ,
                                              control_cycle_sec_, mov_time_);

    goal_joint_tra_.block(0, dim, all_time_steps_, 1) = tra;
  }

  for (int dim=0; dim<goal_joint_pose_msg_.name.size(); dim++)
  {
    std::string joint_name = goal_joint_pose_msg_.name[dim];
    int id = joint_name_to_id_[joint_name];

    double ini_value = goal_joint_position_(id);
    double tar_value = goal_joint_pose_msg_.position[dim];

    Eigen::MatrixXd tra =
        robotis_framework::calcMinimumJerkTra(ini_value, 0.0, 0.0,
                                              tar_value , 0.0 , 0.0 ,
                                              control_cycle_sec_, mov_time_);

    goal_joint_tra_.block(0, id, all_time_steps_, 1) = tra;
  }

  cnt_ = 0;
  is_moving_ = true;

  ROS_INFO("[start] send trajectory");
}

void GripperModule::setTorqueLimit()
{
  robotis_controller_msgs::SyncWriteItem sync_write_msg;
  sync_write_msg.item_name = "goal_torque";

  for (int dim=0; dim<goal_joint_pose_msg_.name.size(); dim++)
  {    
    std::string joint_name = goal_joint_pose_msg_.name[dim];
    int torque_limit = (int) goal_joint_pose_msg_.effort[dim];

    sync_write_msg.joint_name.push_back(joint_name);
    sync_write_msg.value.push_back(torque_limit);
  }

  goal_torque_limit_pub_.publish(sync_write_msg);
}

void GripperModule::setEndTrajectory()
{
  if (is_moving_ == true)
  {
    if (cnt_ >= all_time_steps_)
    {
      ROS_INFO("[end] send trajectory");
      publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "End Trajectory");

      movement_done_pub_.publish(movement_done_msg_);
      movement_done_msg_.data = "";

      is_moving_ = false;
      cnt_ = 0;
    }
  }
}

void GripperModule::process(std::map<std::string, robotis_framework::Dynamixel *> dxls,
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
  }

  /* ----- Movement Event -----*/
  if (is_moving_ == true)
  {
    if (cnt_ == 0)
      publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Start Trajectory");

    // joint space control
    for (int dim=0; dim<NUM_GRIPPER_JOINTS; dim++)
      goal_joint_position_(dim) = goal_joint_tra_(cnt_, dim);

    cnt_++;
  }

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

void GripperModule::stop()
{
  is_moving_ = false;

  return;
}

bool GripperModule::isRunning()
{
  return is_moving_;
}

void GripperModule::publishStatusMsg(unsigned int type, std::string msg)
{
  robotis_controller_msgs::StatusMsg status_msg;
  status_msg.header.stamp = ros::Time::now();
  status_msg.type = type;
  status_msg.module_name = "Gripper";
  status_msg.status_msg = msg;

  status_msg_pub_.publish(status_msg);
}
