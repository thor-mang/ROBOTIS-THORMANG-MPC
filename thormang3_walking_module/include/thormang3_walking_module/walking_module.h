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
 * walking_module.h
 *
 *  Created on: 2016. 1. 25.
 *      Author: Jay Song
 */

#ifndef THORMANG3_WALKING_MODULE_WALKING_MODULE_H_
#define THORMANG3_WALKING_MODULE_WALKING_MODULE_H_


#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <boost/thread.hpp>

#include "robotis_framework_common/motion_module.h"

#include "robotis_controller_msgs/StatusMsg.h"

#include "thormang3_walking_module_msgs/RobotPose.h"
#include "thormang3_walking_module_msgs/GetReferenceStepData.h"
#include "thormang3_walking_module_msgs/AddStepDataArray.h"
#include "thormang3_walking_module_msgs/StartWalking.h"
#include "thormang3_walking_module_msgs/SetBalanceParam.h"
#include "thormang3_walking_module_msgs/IsRunning.h"
#include "thormang3_walking_module_msgs/RemoveExistingStepData.h"

#include "thormang3_online_walking.h"


namespace thormang3
{

class WalkingMotionModule : public robotis_framework::MotionModule, public robotis_framework::Singleton<WalkingMotionModule>
{
public:
  WalkingMotionModule();
  virtual ~WalkingMotionModule();

  void initialize(const int control_cycle_msec, robotis_framework::Robot *robot);
  void process(std::map<std::string, robotis_framework::Dynamixel *> dxls, std::map<std::string, double> sensors);

  void onModuleEnable();
  void onModuleDisable();

  void stop();
  bool isRunning();

  double gyro_roll_, gyro_pitch_;
  double orientation_roll_, orientation_pitch_;
  double r_foot_fx_N_,  r_foot_fy_N_,  r_foot_fz_N_;
  double r_foot_Tx_Nm_, r_foot_Ty_Nm_, r_foot_Tz_Nm_;
  double l_foot_fx_N_,  l_foot_fy_N_,  l_foot_fz_N_;
  double l_foot_Tx_Nm_, l_foot_Ty_Nm_, l_foot_Tz_Nm_;

protected:
  void publishRobotPose(void);

  void publishStatusMsg(unsigned int type, std::string msg);

  /* ROS Topic Callback Functions */
  void imuDataOutputCallback(const sensor_msgs::Imu::ConstPtr &msg);

  /* ROS Service Callback Functions */
  bool getReferenceStepDataServiceCallback(thormang3_walking_module_msgs::GetReferenceStepData::Request  &req,
                                            thormang3_walking_module_msgs::GetReferenceStepData::Response &res);
  bool addStepDataServiceCallback(thormang3_walking_module_msgs::AddStepDataArray::Request  &req,
                                   thormang3_walking_module_msgs::AddStepDataArray::Response &res);
  bool startWalkingServiceCallback(thormang3_walking_module_msgs::StartWalking::Request  &req,
                                   thormang3_walking_module_msgs::StartWalking::Response &res);
  bool IsRunningServiceCallback(thormang3_walking_module_msgs::IsRunning::Request  &req,
                                thormang3_walking_module_msgs::IsRunning::Response &res);
  bool setBalanceParamServiceCallback(thormang3_walking_module_msgs::SetBalanceParam::Request  &req,
                                      thormang3_walking_module_msgs::SetBalanceParam::Response &res);
  bool removeExistingStepDataServiceCallback(thormang3_walking_module_msgs::RemoveExistingStepData::Request  &req,
                                             thormang3_walking_module_msgs::RemoveExistingStepData::Response &res);

  int convertStepDataMsgToStepData(thormang3_walking_module_msgs::StepData& src, robotis_framework::StepData& des);
  int convertStepDataToStepDataMsg(robotis_framework::StepData& src, thormang3_walking_module_msgs::StepData& des);

  void setBalanceParam(thormang3_walking_module_msgs::BalanceParam& balance_param_msg);

  void queueThread();

  int             control_cycle_msec_;
  boost::thread   queue_thread_;
  boost::mutex    publish_mutex_;

  Eigen::MatrixXd rot_x_pi_3d_, rot_z_pi_3d_;
  Eigen::MatrixXd desired_matrix_g_to_cob_;
  Eigen::MatrixXd desired_matrix_g_to_rfoot_;
  Eigen::MatrixXd desired_matrix_g_to_lfoot_;

  bool previous_running_, present_running;

  /* ROS Topic Publish Functions */
  int r_foot_ft_publish_checker_;
  int l_foot_ft_publish_checker_;
  ros::Publisher robot_pose_pub_;
  ros::Publisher status_msg_pub_;

  thormang3_walking_module_msgs::RobotPose  robot_pose_msg_;
  bool balance_update_with_loop_;
  double balance_update_duration_;
  double balance_update_sys_time_;
  Eigen::MatrixXd balance_update_polynomial_coeff_;
  thormang3_walking_module_msgs::BalanceParam previous_balance_param_;
  thormang3_walking_module_msgs::BalanceParam current_balance_param_;
  thormang3_walking_module_msgs::BalanceParam desired_balance_param_;
};

}

#endif /* THORMANG3_WALKING_MODULE_WALKING_MODULE_H_ */
