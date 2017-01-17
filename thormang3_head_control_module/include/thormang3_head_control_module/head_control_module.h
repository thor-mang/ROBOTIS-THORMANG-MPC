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
 * HeadControlModule.h
 *
 *  Created on: 2016. 1. 27.
 *      Author: kayman
 */

#ifndef THORMANG3_HEAD_CONTROL_MODULE_HEAD_CONTROL_MODULE_H_
#define THORMANG3_HEAD_CONTROL_MODULE_HEAD_CONTROL_MODULE_H_

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <boost/thread.hpp>

#include "robotis_framework_common/motion_module.h"
#include "robotis_math/robotis_math.h"
#include "robotis_controller_msgs/StatusMsg.h"
#include "thormang3_head_control_module_msgs/HeadJointPose.h"

namespace thormang3
{

class HeadControlModule : public robotis_framework::MotionModule, public robotis_framework::Singleton<HeadControlModule>
{
 public:
  HeadControlModule();
  virtual ~HeadControlModule();

  void initialize(const int control_cycle_msec, robotis_framework::Robot *robot);
  void process(std::map<std::string, robotis_framework::Dynamixel *> dxls, std::map<std::string, double> sensors);

  void stop();
  bool isRunning();

 private:
  const double SCAN_START_ANGLE = -10 * M_PI / 180;
  const double SCAN_END_ANGLE = 85 * M_PI / 180;

  /* ROS Topic Callback Functions */
  void get3DLidarCallback(const std_msgs::String::ConstPtr &msg);
  void get3DLidarRangeCallback(const std_msgs::Float64::ConstPtr &msg);
  void setHeadJointCallback(const sensor_msgs::JointState::ConstPtr &msg);
  void setHeadJointTimeCallback(const thormang3_head_control_module_msgs::HeadJointPose::ConstPtr &msg);

  void queueThread();
  void jointTraGeneThread();
  void lidarJointTraGeneThread();

  void beforeMoveLidar(double start_angle);
  void startMoveLidar(double target_angle);
  void afterMoveLidar();
  void publishLidarMoveMsg(std::string msg_data);
  void publishDoneMsg(const std::string done_msg);

  void startMoving();
  void finishMoving();
  void stopMoving();

  void publishStatusMsg(unsigned int type, std::string msg);

  Eigen::MatrixXd calcMinimumJerkTraPVA(double pos_start, double vel_start, double accel_start, double pos_end,
                                        double vel_end, double accel_end, double smp_time, double mov_time);

  Eigen::MatrixXd calcLinearInterpolationTra(double pos_start, double pos_end, double smp_time, double mov_time);

  int control_cycle_msec_;
  boost::thread queue_thread_;
  boost::thread *tra_gene_thread_;
  boost::mutex tra_lock_;
  ros::Publisher moving_head_pub_;
  ros::Publisher status_msg_pub_;
  ros::Publisher movement_done_pub_;
  const bool DEBUG;
  bool stop_process_;
  bool is_moving_;
  bool is_direct_control_;
  int tra_count_, tra_size_;
  double moving_time_;
  int current_state_;
  double original_position_lidar_;
  double scan_range_;

  Eigen::MatrixXd target_position_;
  Eigen::MatrixXd current_position_;
  Eigen::MatrixXd goal_position_;
  Eigen::MatrixXd goal_velocity_;
  Eigen::MatrixXd goal_acceleration_;
  Eigen::MatrixXd calc_joint_tra_;
  Eigen::MatrixXd calc_joint_vel_tra_;
  Eigen::MatrixXd calc_joint_accel_tra_;

  std::map<std::string, int> using_joint_name_;

  enum HeadLidarMode
  {
    None,
    BeforeStart,
    StartMove,
    EndMove,
    AfterMove,
    ModeCount
  };
};
}

#endif /* THORMANG3_HEAD_CONTROL_MODULE_HEAD_CONTROL_MODULE_H_ */
