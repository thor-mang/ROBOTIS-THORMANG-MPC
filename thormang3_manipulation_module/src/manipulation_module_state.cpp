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
 *  manipulation_module_state.cpp
 *
 *  Created on: June 7, 2016
 *      Author: sch
 */

#include "thormang3_manipulation_module/manipulation_module_state.h"

using namespace thormang3;

ManipulationModuleState::ManipulationModuleState()
{
  is_moving_ = false;

  /* trajectory */
  cnt_ = 0;

  mov_time_ = 1.0;
  smp_time_ = 0.008;
  all_time_steps_ = int(mov_time_ / smp_time_) + 1;

  calc_joint_tra_ = Eigen::MatrixXd::Zero(all_time_steps_, MAX_JOINT_ID + 1);
  calc_task_tra_  = Eigen::MatrixXd::Zero(all_time_steps_, 3);

  joint_ini_pose_ = Eigen::MatrixXd::Zero( MAX_JOINT_ID + 1, 1);

  /* ik */
  ik_solve_ = false;

  ik_target_position_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);

  ik_start_rotation_  = robotis_framework::getRotation4d(0.0, 0.0, 0.0);
  ik_target_rotation_ = robotis_framework::getRotation4d(0.0, 0.0, 0.0);

  ik_id_start_  = 0;
  ik_id_end_    = 0;

  ik_weight_    = Eigen::MatrixXd::Zero( MAX_JOINT_ID + 1, 1);
  ik_weight_.fill(1.0);
}

ManipulationModuleState::~ManipulationModuleState()
{
}

void ManipulationModuleState::setInverseKinematics(int cnt, Eigen::MatrixXd start_rotation)
{
  for (int dim = 0; dim < 3; dim++)
    ik_target_position_.coeffRef(dim, 0) = calc_task_tra_.coeff(cnt, dim);

  Eigen::Quaterniond start_quaternion = robotis_framework::convertRotationToQuaternion(start_rotation);

  Eigen::Quaterniond target_quaternion(goal_kinematics_pose_msg_.pose.orientation.w,
                                        goal_kinematics_pose_msg_.pose.orientation.x,
                                        goal_kinematics_pose_msg_.pose.orientation.y,
                                        goal_kinematics_pose_msg_.pose.orientation.z);

  double count = (double) cnt / (double) all_time_steps_;

  Eigen::Quaterniond _quaternion = start_quaternion.slerp(count, target_quaternion);

  ik_target_rotation_ = robotis_framework::convertQuaternionToRotation(_quaternion);
}

