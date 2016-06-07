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
 * kinematcis_dynamics.h
 *
 *  Created on: June 7, 2016
 *      Author: sch
 */

#ifndef THORMANG3_KINEMATICS_DYNAMICS_KINEMATICS_DYNAMICS_H_
#define THORMANG3_KINEMATICS_DYNAMICS_KINEMATICS_DYNAMICS_H_

#include <vector>

#include "kinematics_dynamics_define.h"
#include "link_data.h"

namespace thormang3
{

enum TreeSelect {
  Manipulation,
  Walking,
  WholeBody
};

class KinematicsDynamics
{

public:
  KinematicsDynamics();
  ~KinematicsDynamics();
  KinematicsDynamics(TreeSelect tree);

  std::vector<int> findRoute(int to);
  std::vector<int> findRoute(int from, int to);

  double calcTotalMass(int joint_id);
  Eigen::MatrixXd calcMC(int joint_id);
  Eigen::MatrixXd calcCOM(Eigen::MatrixXd mc);

  void calcForwardKinematics(int joint_ID);

  Eigen::MatrixXd calcJacobian(std::vector<int> idx);
  Eigen::MatrixXd calcJacobianCOM(std::vector<int> idx);
  Eigen::MatrixXd calcVWerr(Eigen::MatrixXd tar_position, Eigen::MatrixXd curr_position, Eigen::MatrixXd tar_orientation, Eigen::MatrixXd curr_orientation);

  bool calcInverseKinematics(int to, Eigen::MatrixXd tar_position, Eigen::MatrixXd tar_orientation , int max_iter, double ik_err);
  bool calcInverseKinematics(int from, int to, Eigen::MatrixXd tar_position, Eigen::MatrixXd tar_orientation, int max_iter, double ik_err);

  // with weight
  bool calcInverseKinematics(int to, Eigen::MatrixXd tar_position, Eigen::MatrixXd tar_orientation, int max_iter, double ik_err , Eigen::MatrixXd weight);
  bool calcInverseKinematics(int from, int to, Eigen::MatrixXd tar_position,
      Eigen::MatrixXd tar_orientation, int max_iter, double ik_err,
      Eigen::MatrixXd weight) {
    bool ik_success = false;
    bool limit_success = false;
    //  calcForwardKinematics(0);
    std::vector<int> idx = findRoute(from, to);
    /* weight */
    Eigen::MatrixXd weight_matrix = Eigen::MatrixXd::Identity(idx.size(),
        idx.size());
    for (int ix = 0; ix < idx.size(); ix++)
      weight_matrix.coeffRef(ix, ix) = weight.coeff(idx[ix], 0);
    /* damping */
    Eigen::MatrixXd eval = Eigen::MatrixXd::Zero(6, 6);
    double p_damping = 1e-5;
    double R_damping = 1e-5;
    for (int ix = 0; ix < 3; ix++) {
      eval.coeffRef(ix, ix) = p_damping;
      eval.coeffRef(ix + 3, ix + 3) = R_damping;
    }
    /* ik */
    for (int iter = 0; iter < max_iter; iter++) {
      Eigen::MatrixXd jacobian = calcJacobian(idx);
      Eigen::MatrixXd curr_position = thormang3_link_data_[to]->position;
      Eigen::MatrixXd curr_orientation = thormang3_link_data_[to]->orientation;
      Eigen::MatrixXd err = calcVWerr(tar_position, curr_position,
          tar_orientation, curr_orientation);
      if (err.norm() < ik_err) {
        ik_success = true;
        break;
      } else
        ik_success = false;

      Eigen::MatrixXd jacobian_trans = (jacobian * weight_matrix
          * jacobian.transpose() + eval);
      Eigen::MatrixXd jacobian_inv = weight_matrix * jacobian.transpose()
          * jacobian_trans.inverse();
      Eigen::MatrixXd delta_angle = jacobian_inv * err;
      for (int id = 0; id < idx.size(); id++) {
        int joint_id = idx[id];
        thormang3_link_data_[joint_id]->joint_angle += delta_angle.coeff(id);
      }
      calcForwardKinematics(0);
    }
    /* check joint limit */
    for (int id = 0; id < idx.size(); id++) {
      int _joint_num = idx[id];
      if (thormang3_link_data_[_joint_num]->joint_angle
          >= thormang3_link_data_[_joint_num]->joint_limit_max) {
        limit_success = false;
        break;
      } else if (thormang3_link_data_[_joint_num]->joint_angle
          <= thormang3_link_data_[_joint_num]->joint_limit_min) {
        limit_success = false;
        break;
      } else
        limit_success = true;
    }
    if (ik_success == true && limit_success == true)
      return true;
    else
      return false;
  }

  bool calcInverseKinematicsForLeg(double *out, double x, double y, double z, double roll, double pitch, double yaw);
  bool calcInverseKinematicsForRightLeg(double *out, double x, double y, double z, double roll, double pitch, double yaw);
  bool calcInverseKinematicsForLeftLeg(double *out, double x, double y, double z, double roll, double pitch, double yaw);


  LinkData *thormang3_link_data_ [ ALL_JOINT_ID + 1 ];

  double thigh_length_m_;
  double calf_length_m_;
  double ankle_length_m_;
  double leg_side_offset_m_;
};

}

#endif /* THORMANG3_KINEMATICS_DYNAMICS_KINEMATICS_DYNAMICS_H_ */
