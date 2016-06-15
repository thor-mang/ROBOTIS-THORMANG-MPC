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
 *  manipulation_module_state.h
 *
 *  Created on: June 7, 2016
 *      Author: sch
 */

#ifndef THORMANG3_MANIPULATION_MODULE_MANIPULATION_MODULE_STATE_H_
#define THORMANG3_MANIPULATION_MODULE_MANIPULATION_MODULE_STATE_H_

#define EIGEN_NO_DEBUG
#define EIGEN_NO_STATIC_ASSERT

#include <std_msgs/Float64.h>
#include <std_msgs/String.h>

#include "robotis_math/robotis_math.h"
#include "thormang3_kinematics_dynamics/kinematics_dynamics.h"

#include "thormang3_manipulation_module_msgs/JointPose.h"
#include "thormang3_manipulation_module_msgs/KinematicsPose.h"

namespace thormang3
{

class ManipulationModuleState
{
public:
  ManipulationModuleState();
  ~ManipulationModuleState();

  void setInverseKinematics(int cnt, Eigen::MatrixXd start_rotation);

  bool is_moving_;

  /* trajectory */
  int cnt_; // counter number

  double mov_time_; // movement time
  double smp_time_; // sampling time

  int all_time_steps_; // all time steps of movement time

  Eigen::MatrixXd calc_joint_tra_; // calculated joint trajectory
  Eigen::MatrixXd calc_task_tra_; // calculated task trajectory

  Eigen::MatrixXd joint_ini_pose_;

  /* msgs */
  thormang3_manipulation_module_msgs::JointPose goal_joint_pose_msg_;
  thormang3_manipulation_module_msgs::KinematicsPose goal_kinematics_pose_msg_;

  /* ik */
  bool  ik_solve_;
  int   ik_id_start_;
  int   ik_id_end_;
  Eigen::MatrixXd ik_target_position_;
  Eigen::MatrixXd ik_start_rotation_;
  Eigen::MatrixXd ik_target_rotation_;

  Eigen::MatrixXd ik_weight_;
};

}

#endif /* THORMANG3_MANIPULATION_MODULE_MANIPULATION_MODULE_STATE_H_ */
