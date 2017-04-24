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
 *  manipulation_module.h
 *
 *  Created on: June 7, 2016
 *      Author: sch
 */

#ifndef THORMANG3_MANIPULATION_MODULE_MANIPULATION_MODULE_H_
#define THORMANG3_MANIPULATION_MODULE_MANIPULATION_MODULE_H_

#include <map>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/package.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <boost/thread.hpp>
#include <yaml-cpp/yaml.h>

#include "robotis_math/robotis_math.h"
#include "robotis_framework_common/motion_module.h"

#include "robotis_controller_msgs/JointCtrlModule.h"
#include "robotis_controller_msgs/StatusMsg.h"

#include "thormang3_kinematics_dynamics/kinematics_dynamics.h"

#include "thormang3_manipulation_module_msgs/JointPose.h"
#include "thormang3_manipulation_module_msgs/KinematicsPose.h"

#include "thormang3_manipulation_module_msgs/GetJointPose.h"
#include "thormang3_manipulation_module_msgs/GetKinematicsPose.h"

namespace thormang3
{

class ManipulationModule: public robotis_framework::MotionModule,
                          public robotis_framework::Singleton<ManipulationModule>
{
public:
  ManipulationModule();
  virtual ~ManipulationModule();

  /* ROS Topic Callback Functions */
  void initPoseMsgCallback(const std_msgs::String::ConstPtr& msg);
  void jointPoseMsgCallback(const thormang3_manipulation_module_msgs::JointPose::ConstPtr& msg);
  void kinematicsPoseMsgCallback(const thormang3_manipulation_module_msgs::KinematicsPose::ConstPtr& msg);

  bool getJointPoseCallback(thormang3_manipulation_module_msgs::GetJointPose::Request &req,
                            thormang3_manipulation_module_msgs::GetJointPose::Response &res);
  bool getKinematicsPoseCallback(thormang3_manipulation_module_msgs::GetKinematicsPose::Request &req,
                                 thormang3_manipulation_module_msgs::GetKinematicsPose::Response &res);

  /* ROS Calculation Functions */
  void initPoseTrajGenerateProc();
  void jointTrajGenerateProc();
  void taskTrajGenerateProc();

  /* ROS Framework Functions */
  void initialize(const int control_cycle_msec, robotis_framework::Robot *robot);
  void process(std::map<std::string, robotis_framework::Dynamixel *> dxls, std::map<std::string, double> sensors);
  void stop();
  bool isRunning();

  void publishStatusMsg(unsigned int type, std::string msg);

  /* Parameter */
  KinematicsDynamics *robotis_;

private:
  void queueThread();

  void parseData(const std::string &path);
  void parseIniPoseData(const std::string &path);

  bool arm_angle_display_;

  double          control_cycle_sec_;
  boost::thread   queue_thread_;
  boost::thread  *traj_generate_tread_;

  std_msgs::String movement_done_msg_;

  ros::Publisher  status_msg_pub_;
  ros::Publisher  movement_done_pub_;

  /* joint state */
  Eigen::VectorXd present_joint_position_;
  Eigen::VectorXd goal_joint_position_;
  Eigen::VectorXd init_joint_position_;

  /* trajectory */
  bool    is_moving_;
  double  mov_time_;
  int     cnt_;
  int     all_time_steps_;

  Eigen::MatrixXd goal_joint_tra_;
  Eigen::MatrixXd goal_task_tra_;

    /* msgs */
  thormang3_manipulation_module_msgs::JointPose       goal_joint_pose_msg_;
  thormang3_manipulation_module_msgs::KinematicsPose  goal_kinematics_pose_msg_;

  /* inverse kinematics */
  bool  ik_solving_;
  int   ik_id_start_;
  int   ik_id_end_;

  Eigen::MatrixXd ik_target_position_;
  Eigen::MatrixXd ik_start_rotation_;
  Eigen::MatrixXd ik_target_rotation_;
  Eigen::MatrixXd ik_weight_;

  void setInverseKinematics(int cnt, Eigen::MatrixXd start_rotation);

  std::map<std::string, int> joint_name_to_id_;
};

}

#endif /* THORMANG3_MANIPULATION_MODULE_MANIPULATION_MODULE_H_ */
