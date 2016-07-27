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
 * robotis_balance_control.h
 *
 *  Created on: July 12, 2016
 *      Author: jay
 */

#include <iostream>
#include "thormang3_balance_control/thormang3_balance_control.h"

using namespace thormang3;

DampingController::DampingController()
{
  desired_ = 0.0;

  gain_ = 0.0;
  time_constant_sec_ = 1.0;
  output_ = 0.0;
  control_cycle_sec_ = 0.008;

  previous_result_ = 0.0;
}

DampingController::DampingController(double time_unit_sec)
{
  desired_ = 0.0;

  gain_ = 0.0;
  time_constant_sec_ = 1.0;
  output_ = 0.0;
  control_cycle_sec_ = time_unit_sec;

  previous_result_ = 0.0;
}

DampingController::~DampingController()
{  }

double DampingController::getDampingControllerOutput(double present_sensor_output)
{
  double cut_off_freq = 1.0/time_constant_sec_;
  double alpha = 1.0;
  alpha = (2.0*M_PI*cut_off_freq*control_cycle_sec_)/(1.0+2.0*M_PI*cut_off_freq*control_cycle_sec_);

  previous_result_ = alpha*(desired_ - present_sensor_output) + (1.0 - alpha)*previous_result_;
  output_ = gain_*previous_result_;

  return output_;
}




RobotisBalanceControl::RobotisBalanceControl()
{
  balance_control_error_ = BalanceControlError::NoError;
  control_cycle_sec_ = 0.008;

  desired_global_to_robot_      = Eigen::MatrixXd::Identity(4,4);
  desired_global_to_cob_        = Eigen::MatrixXd::Identity(4,4);
  desired_global_to_right_foot_ = Eigen::MatrixXd::Identity(4,4);
  desired_global_to_left_foot_  = Eigen::MatrixXd::Identity(4,4);
  desired_robot_to_right_foot_  = Eigen::MatrixXd::Identity(4,4);
  desired_robot_to_left_foot_   = Eigen::MatrixXd::Identity(4,4);

  gyro_cut_off_freq_  = 10.0;
  gyro_lpf_alpha_     =  2.0*M_PI*gyro_cut_off_freq_*control_cycle_sec_/(1.0 + 2.0*M_PI*gyro_cut_off_freq_*control_cycle_sec_);
  gyro_roll_filtered_ = gyro_pitch_filtered_ = 0;
  desired_gyro_roll_  = desired_gyro_pitch_ = 0;

  //sensed values
  current_gyro_roll_rad_per_sec_ = current_gyro_pitch_rad_per_sec_ = 0;

  current_orientation_roll_rad_ = current_orientation_pitch_rad_ = 0;

  current_right_fx_N_  = current_right_fy_N_  = current_right_fz_N_  = 0;
  current_right_tx_Nm_ = current_right_ty_Nm_ = current_right_tz_Nm_ = 0;
  current_left_fx_N_   = current_left_fy_N_   = current_left_fz_N_   = 0;
  current_left_tx_Nm_  = current_left_ty_Nm_  = current_left_tz_Nm_  = 0;

  // balance algorithm result
  foot_roll_adjustment_by_gyro_roll_ = 0;
  foot_pitch_adjustment_by_gyro_pitch_ = 0;

  foot_roll_adjustment_by_orientation_roll_ = 0;
  foot_pitch_adjustment_by_orientation_pitch_ = 0;

  foot_z_adjustment_by_force_z_difference_ = 0;

  r_foot_x_adjustment_by_force_x_ = 0;
  r_foot_y_adjustment_by_force_y_ = 0;
  r_foot_roll_adjustment_by_torque_roll_ = 0;
  r_foot_pitch_adjustment_by_torque_pitch_ = 0;

  l_foot_x_adjustment_by_force_x_ = 0;
  l_foot_y_adjustment_by_force_y_ = 0;
  l_foot_roll_adjustment_by_torque_roll_ = 0;
  l_foot_pitch_adjustment_by_torque_pitch_ = 0;

  // manual cob adjustment
  cob_x_manual_adjustment_m_ = 0;
  cob_y_manual_adjustment_m_ = 0;
  cob_z_manual_adjustment_m_ = 0;

  // gyro gain
  gyro_balance_gain_ratio_ = 0.0;
  gyro_balance_roll_gain_  = -0.10*0.75*gyro_balance_gain_ratio_;
  gyro_balance_pitch_gain_ = -0.10*0.5 *gyro_balance_gain_ratio_;


  // maximum adjustment
  cob_x_adjustment_abs_max_m_ = 0.05;
  cob_y_adjustment_abs_max_m_ = 0.05;
  cob_z_adjustment_abs_max_m_ = 0.05;
  cob_roll_adjustment_abs_max_rad_  = 15.0*DEGREE2RADIAN;
  cob_pitch_adjustment_abs_max_rad_ = 15.0*DEGREE2RADIAN;
  cob_yaw_adjustment_abs_max_rad_   = 15.0*DEGREE2RADIAN;
  foot_x_adjustment_abs_max_m_ = 0.05;
  foot_y_adjustment_abs_max_m_ = 0.05;
  foot_z_adjustment_abs_max_m_ = 0.05;
  foot_roll_adjustment_abs_max_rad_  = 15.0*DEGREE2RADIAN;
  foot_pitch_adjustment_abs_max_rad_ = 15.0*DEGREE2RADIAN;
  foot_yaw_adjustment_abs_max_rad_   = 15.0*DEGREE2RADIAN;

  mat_cob_adjustment_        = Eigen::MatrixXd::Identity(4,4);
  mat_right_foot_adjustment_ = Eigen::MatrixXd::Identity(4,4);
  mat_left_foot_adjustment_  = Eigen::MatrixXd::Identity(4,4);
  pose_cob_adjustment_         = Eigen::VectorXd::Zero(6);
  pose_right_foot_adjustment_  = Eigen::VectorXd::Zero(6);;
  pose_left_foot_adjustment_   = Eigen::VectorXd::Zero(6);;
}

RobotisBalanceControl::~RobotisBalanceControl()
{  }

void RobotisBalanceControl::initialize(const int control_cycle_msec)
{
  balance_control_error_ = BalanceControlError::NoError;

  control_cycle_sec_ = control_cycle_msec * 0.001;

  gyro_cut_off_freq_ = 10.0;
  gyro_lpf_alpha_ = 2.0*M_PI*gyro_cut_off_freq_*control_cycle_sec_/(1.0 + 2.0*M_PI*gyro_cut_off_freq_*control_cycle_sec_);

  pose_cob_adjustment_.fill(0);
  pose_right_foot_adjustment_.fill(0);
  pose_left_foot_adjustment_.fill(0);

  foot_roll_angle_ctrl_.control_cycle_sec_ = control_cycle_sec_;
  foot_pitch_angle_ctrl_.control_cycle_sec_ = control_cycle_sec_;

  foot_force_z_diff_ctrl_.control_cycle_sec_ = control_cycle_sec_;

  right_foot_force_x_ctrl_.control_cycle_sec_ = control_cycle_sec_;
  right_foot_force_y_ctrl_.control_cycle_sec_ = control_cycle_sec_;
  right_foot_torque_roll_ctrl_.control_cycle_sec_ = control_cycle_sec_;
  right_foot_torque_pitch_ctrl_.control_cycle_sec_ = control_cycle_sec_;

  left_foot_force_x_ctrl_.control_cycle_sec_ = control_cycle_sec_;
  left_foot_force_y_ctrl_.control_cycle_sec_ = control_cycle_sec_;
  left_foot_torque_roll_ctrl_.control_cycle_sec_ = control_cycle_sec_;
  left_foot_torque_pitch_ctrl_.control_cycle_sec_ = control_cycle_sec_;
}

void RobotisBalanceControl::process(int *balance_error, Eigen::MatrixXd *cob_adjustment, Eigen::MatrixXd *right_foot_adjustment, Eigen::MatrixXd *left_foot_adjustment)
{
  balance_control_error_ = BalanceControlError::NoError;

  pose_cob_adjustment_.fill(0);
  pose_right_foot_adjustment_.fill(0);
  pose_left_foot_adjustment_.fill(0);

  // gyro
  gyro_roll_filtered_  = current_gyro_roll_rad_per_sec_*gyro_lpf_alpha_  + (1.0 - gyro_lpf_alpha_)*gyro_roll_filtered_;
  gyro_pitch_filtered_ = current_gyro_pitch_rad_per_sec_*gyro_lpf_alpha_ + (1.0 - gyro_lpf_alpha_)*gyro_pitch_filtered_;

  foot_roll_adjustment_by_gyro_roll_   = (desired_gyro_roll_  - gyro_roll_filtered_)  * gyro_balance_roll_gain_;
  foot_pitch_adjustment_by_gyro_pitch_ = (desired_gyro_pitch_ - gyro_pitch_filtered_) * gyro_balance_pitch_gain_;

  // z by imu
  foot_roll_adjustment_by_orientation_roll_ = foot_roll_angle_ctrl_.getDampingControllerOutput(current_orientation_roll_rad_);
  foot_pitch_adjustment_by_orientation_pitch_ = foot_pitch_angle_ctrl_.getDampingControllerOutput(current_orientation_pitch_rad_);

  Eigen::MatrixXd mat_orientation_adjustment_by_imu = robotis_framework::getRotation4d(foot_roll_adjustment_by_gyro_roll_ + foot_roll_adjustment_by_orientation_roll_, foot_pitch_adjustment_by_gyro_pitch_ + foot_pitch_adjustment_by_orientation_pitch_, 0.0);
  Eigen::MatrixXd mat_r_xy, mat_l_xy;
  mat_r_xy.resize(4,1); mat_l_xy.resize(4,1);
  mat_r_xy.coeffRef(0,0) = desired_robot_to_right_foot_.coeff(0,3);
  mat_r_xy.coeffRef(1,0) = desired_robot_to_right_foot_.coeff(1,3);
  mat_r_xy.coeffRef(2,0) = 0.0;
  mat_r_xy.coeffRef(3,0) = 1;

  mat_l_xy.coeffRef(0,0) = desired_robot_to_left_foot_.coeff(0,3);
  mat_l_xy.coeffRef(1,0) = desired_robot_to_left_foot_.coeff(1,3);
  mat_l_xy.coeffRef(2,0) = 0.0;
  mat_l_xy.coeffRef(3,0) = 1;

  mat_r_xy = mat_orientation_adjustment_by_imu * mat_r_xy;
  mat_l_xy = mat_orientation_adjustment_by_imu * mat_l_xy;

  // ft sensor
  foot_z_adjustment_by_force_z_difference_ = 0.001*foot_force_z_diff_ctrl_.getDampingControllerOutput(current_left_fz_N_ - current_right_fz_N_);

  r_foot_x_adjustment_by_force_x_ = 0.001*right_foot_force_x_ctrl_.getDampingControllerOutput(current_right_fx_N_);
  r_foot_y_adjustment_by_force_y_ = 0.001*right_foot_force_y_ctrl_.getDampingControllerOutput(current_right_fy_N_);
  r_foot_roll_adjustment_by_torque_roll_ = right_foot_torque_roll_ctrl_.getDampingControllerOutput(current_right_tx_Nm_);
  r_foot_pitch_adjustment_by_torque_pitch_ = right_foot_torque_pitch_ctrl_.getDampingControllerOutput(current_right_ty_Nm_);

  l_foot_x_adjustment_by_force_x_ = 0.001*left_foot_force_x_ctrl_.getDampingControllerOutput(current_left_fx_N_);
  l_foot_y_adjustment_by_force_y_ = 0.001*left_foot_force_y_ctrl_.getDampingControllerOutput(current_left_fy_N_);
  l_foot_roll_adjustment_by_torque_roll_ = left_foot_torque_roll_ctrl_.getDampingControllerOutput(current_left_tx_Nm_);
  l_foot_pitch_adjustment_by_torque_pitch_ = left_foot_torque_pitch_ctrl_.getDampingControllerOutput(current_left_ty_Nm_);


  // sum of sensory balance result
  pose_cob_adjustment_.coeffRef(0) = cob_x_manual_adjustment_m_;
  pose_cob_adjustment_.coeffRef(1) = cob_y_manual_adjustment_m_;
  pose_cob_adjustment_.coeffRef(2) = cob_z_manual_adjustment_m_;

  pose_right_foot_adjustment_.coeffRef(0) = r_foot_x_adjustment_by_force_x_;
  pose_right_foot_adjustment_.coeffRef(1) = r_foot_y_adjustment_by_force_y_;
  pose_right_foot_adjustment_.coeffRef(2) = 0.5*foot_z_adjustment_by_force_z_difference_ + mat_r_xy.coeff(2, 0);
  pose_right_foot_adjustment_.coeffRef(3) = (foot_roll_adjustment_by_gyro_roll_ + foot_roll_adjustment_by_orientation_roll_ + r_foot_roll_adjustment_by_torque_roll_);
  pose_right_foot_adjustment_.coeffRef(4) = (foot_pitch_adjustment_by_gyro_pitch_ + foot_pitch_adjustment_by_orientation_pitch_ + r_foot_pitch_adjustment_by_torque_pitch_);

  pose_left_foot_adjustment_.coeffRef(0) = l_foot_x_adjustment_by_force_x_;
  pose_left_foot_adjustment_.coeffRef(1) = l_foot_y_adjustment_by_force_y_;
  pose_left_foot_adjustment_.coeffRef(2) = -0.5*foot_z_adjustment_by_force_z_difference_ + mat_l_xy.coeff(2, 0);
  pose_left_foot_adjustment_.coeffRef(3) = (foot_roll_adjustment_by_gyro_roll_ + foot_roll_adjustment_by_orientation_roll_ + l_foot_roll_adjustment_by_torque_roll_);
  pose_left_foot_adjustment_.coeffRef(4) = (foot_pitch_adjustment_by_gyro_pitch_ + foot_pitch_adjustment_by_orientation_pitch_ + l_foot_pitch_adjustment_by_torque_pitch_);

  // check limitation
  if((fabs(pose_cob_adjustment_.coeff(0)) == cob_x_adjustment_abs_max_m_      ) ||
     (fabs(pose_cob_adjustment_.coeff(1)) == cob_y_adjustment_abs_max_m_      ) ||
     (fabs(pose_cob_adjustment_.coeff(2)) == cob_z_adjustment_abs_max_m_      ) ||
     (fabs(pose_cob_adjustment_.coeff(3)) == cob_roll_adjustment_abs_max_rad_ ) ||
     (fabs(pose_cob_adjustment_.coeff(4)) == cob_pitch_adjustment_abs_max_rad_) ||
     (fabs(pose_right_foot_adjustment_.coeff(0)) == foot_x_adjustment_abs_max_m_      ) ||
     (fabs(pose_right_foot_adjustment_.coeff(1)) == foot_y_adjustment_abs_max_m_      ) ||
     (fabs(pose_right_foot_adjustment_.coeff(2)) == foot_z_adjustment_abs_max_m_      ) ||
     (fabs(pose_right_foot_adjustment_.coeff(3)) == foot_roll_adjustment_abs_max_rad_ ) ||
     (fabs(pose_right_foot_adjustment_.coeff(4)) == foot_pitch_adjustment_abs_max_rad_) ||
     (fabs(pose_left_foot_adjustment_.coeff(0)) == foot_x_adjustment_abs_max_m_      ) ||
     (fabs(pose_left_foot_adjustment_.coeff(1)) == foot_y_adjustment_abs_max_m_      ) ||
     (fabs(pose_left_foot_adjustment_.coeff(2)) == foot_z_adjustment_abs_max_m_      ) ||
     (fabs(pose_left_foot_adjustment_.coeff(3)) == foot_roll_adjustment_abs_max_rad_ ) ||
     (fabs(pose_left_foot_adjustment_.coeff(4)) == foot_pitch_adjustment_abs_max_rad_))
    balance_control_error_ &= BalanceControlError::BalanceLimit;

  pose_cob_adjustment_.coeffRef(0) = copysign(fmin(fabs(pose_cob_adjustment_.coeff(0)), cob_x_adjustment_abs_max_m_      ), pose_cob_adjustment_.coeff(0));
  pose_cob_adjustment_.coeffRef(1) = copysign(fmin(fabs(pose_cob_adjustment_.coeff(1)), cob_x_adjustment_abs_max_m_      ), pose_cob_adjustment_.coeff(1));
  pose_cob_adjustment_.coeffRef(2) = copysign(fmin(fabs(pose_cob_adjustment_.coeff(2)), cob_x_adjustment_abs_max_m_      ), pose_cob_adjustment_.coeff(2));
  pose_cob_adjustment_.coeffRef(3) = copysign(fmin(fabs(pose_cob_adjustment_.coeff(3)), cob_roll_adjustment_abs_max_rad_ ), pose_cob_adjustment_.coeff(3));
  pose_cob_adjustment_.coeffRef(4) = copysign(fmin(fabs(pose_cob_adjustment_.coeff(4)), cob_pitch_adjustment_abs_max_rad_), pose_cob_adjustment_.coeff(4));
  pose_cob_adjustment_.coeffRef(5)   = 0;

  pose_right_foot_adjustment_.coeffRef(0) = copysign(fmin(fabs(pose_right_foot_adjustment_.coeff(0)), foot_x_adjustment_abs_max_m_      ), pose_right_foot_adjustment_.coeff(0));
  pose_right_foot_adjustment_.coeffRef(1) = copysign(fmin(fabs(pose_right_foot_adjustment_.coeff(1)), foot_y_adjustment_abs_max_m_      ), pose_right_foot_adjustment_.coeff(1));
  pose_right_foot_adjustment_.coeffRef(2) = copysign(fmin(fabs(pose_right_foot_adjustment_.coeff(2)), foot_z_adjustment_abs_max_m_      ), pose_right_foot_adjustment_.coeff(2));
  pose_right_foot_adjustment_.coeffRef(3) = copysign(fmin(fabs(pose_right_foot_adjustment_.coeff(3)), foot_roll_adjustment_abs_max_rad_ ), pose_right_foot_adjustment_.coeff(3));
  pose_right_foot_adjustment_.coeffRef(4) = copysign(fmin(fabs(pose_right_foot_adjustment_.coeff(4)), foot_pitch_adjustment_abs_max_rad_), pose_right_foot_adjustment_.coeff(4));
  pose_right_foot_adjustment_.coeffRef(5)   = 0;

  pose_left_foot_adjustment_.coeffRef(0) = copysign(fmin(fabs(pose_left_foot_adjustment_.coeff(0)), foot_x_adjustment_abs_max_m_      ), pose_left_foot_adjustment_.coeff(0));
  pose_left_foot_adjustment_.coeffRef(1) = copysign(fmin(fabs(pose_left_foot_adjustment_.coeff(1)), foot_y_adjustment_abs_max_m_      ), pose_left_foot_adjustment_.coeff(1));
  pose_left_foot_adjustment_.coeffRef(2) = copysign(fmin(fabs(pose_left_foot_adjustment_.coeff(2)), foot_z_adjustment_abs_max_m_      ), pose_left_foot_adjustment_.coeff(2));
  pose_left_foot_adjustment_.coeffRef(3) = copysign(fmin(fabs(pose_left_foot_adjustment_.coeff(3)), foot_roll_adjustment_abs_max_rad_ ), pose_left_foot_adjustment_.coeff(3));
  pose_left_foot_adjustment_.coeffRef(4) = copysign(fmin(fabs(pose_left_foot_adjustment_.coeff(4)), foot_pitch_adjustment_abs_max_rad_), pose_left_foot_adjustment_.coeff(4));
  pose_left_foot_adjustment_.coeffRef(5)   = 0;

  // return result
  mat_cob_adjustment_ = robotis_framework::getTransformationXYZRPY(pose_cob_adjustment_.coeff(0), pose_cob_adjustment_.coeff(1), pose_cob_adjustment_.coeff(2),
      pose_cob_adjustment_.coeff(3), pose_cob_adjustment_.coeff(4), pose_cob_adjustment_.coeff(5));

  mat_right_foot_adjustment_ = robotis_framework::getTransformationXYZRPY(pose_right_foot_adjustment_.coeff(0), pose_right_foot_adjustment_.coeff(1), pose_right_foot_adjustment_.coeff(2),
      pose_right_foot_adjustment_.coeff(3), pose_right_foot_adjustment_.coeff(4), pose_right_foot_adjustment_.coeff(5));

  mat_left_foot_adjustment_ = robotis_framework::getTransformationXYZRPY(pose_left_foot_adjustment_.coeff(0), pose_left_foot_adjustment_.coeff(1), pose_left_foot_adjustment_.coeff(2),
      pose_left_foot_adjustment_.coeff(3), pose_left_foot_adjustment_.coeff(4), pose_left_foot_adjustment_.coeff(5));

  if(balance_error != 0)
    *balance_error = balance_control_error_;

//  *cob_adjustment        = Eigen::MatrixXd::Identity(4,4);
//  *right_foot_adjustment = Eigen::MatrixXd::Identity(4,4);
//  *left_foot_adjustment  = Eigen::MatrixXd::Identity(4,4);
  *cob_adjustment        = mat_cob_adjustment_;
  *right_foot_adjustment = mat_right_foot_adjustment_;
  *left_foot_adjustment  = mat_left_foot_adjustment_;
}

void RobotisBalanceControl::setDesiredPose(const Eigen::MatrixXd &global_to_cob, const Eigen::MatrixXd &global_to_right_foot, const Eigen::MatrixXd &global_to_left_foot)
{
  desired_global_to_cob_        = global_to_cob;
  desired_global_to_right_foot_ = global_to_right_foot;
  desired_global_to_left_foot_  = global_to_left_foot;

  double cob_yaw = atan2( desired_global_to_cob_.coeff(1,0), desired_global_to_cob_.coeff(0,0));
  desired_global_to_robot_ = robotis_framework::getTransformationXYZRPY(desired_global_to_cob_.coeff(0, 3), desired_global_to_cob_.coeff(1, 3), desired_global_to_cob_.coeff(2, 3), 0, 0, cob_yaw);

  Eigen::MatrixXd robot_to_global = robotis_framework::getInverseTransformation(desired_global_to_robot_);

  desired_robot_to_right_foot_ = robot_to_global * desired_global_to_right_foot_;
  desired_robot_to_left_foot_  = robot_to_global * desired_global_to_left_foot_;;
}

void RobotisBalanceControl::setDesiredCOBGyro(double gyro_roll, double gyro_pitch)\
{
  desired_gyro_roll_  = gyro_roll;
  desired_gyro_pitch_ = gyro_pitch;
}

void RobotisBalanceControl::setDesiredCOBOrientation(double cob_orientation_roll, double cob_orientation_pitch)
{
  foot_roll_angle_ctrl_.desired_  = cob_orientation_roll;
  foot_pitch_angle_ctrl_.desired_ = cob_orientation_pitch;
}

void RobotisBalanceControl::setDesiredFootForceTorque(double r_force_x_N,      double r_force_y_N,       double r_force_z_N,
                                               double r_torque_roll_Nm, double r_torque_pitch_Nm, double r_torque_yaw_Nm,
                                               double l_force_x_N,      double l_force_y_N,       double l_force_z_N,
                                               double l_torque_roll_Nm, double l_torque_pitch_Nm, double l_torque_yaw_Nm)
{
  foot_force_z_diff_ctrl_.desired_ = l_force_z_N - r_force_z_N;

  right_foot_force_x_ctrl_.desired_      = r_force_x_N;
  right_foot_force_y_ctrl_.desired_      = r_force_y_N;
  right_foot_torque_roll_ctrl_.desired_  = r_torque_roll_Nm;
  right_foot_torque_pitch_ctrl_.desired_ = r_torque_pitch_Nm;

  left_foot_force_x_ctrl_.desired_      = l_force_x_N;
  left_foot_force_y_ctrl_.desired_      = l_force_y_N;
  left_foot_torque_roll_ctrl_.desired_  = l_torque_roll_Nm;
  left_foot_torque_pitch_ctrl_.desired_ = l_torque_pitch_Nm;
}


void RobotisBalanceControl::setCurrentGyroSensorOutput(double gyro_roll, double gyro_pitch)
{
  current_gyro_roll_rad_per_sec_  = gyro_roll;
  current_gyro_pitch_rad_per_sec_ = gyro_pitch;
}

void RobotisBalanceControl::setCurrentOrientationSensorOutput(double cob_orientation_roll, double cob_orientation_pitch)
{
  current_orientation_roll_rad_  = cob_orientation_roll;
  current_orientation_pitch_rad_ = cob_orientation_pitch;
}

void RobotisBalanceControl::setCurrentFootForceTorqueSensorOutput(double r_force_x_N,      double r_force_y_N,       double r_force_z_N,
                                                           double r_torque_roll_Nm, double r_torque_pitch_Nm, double r_torque_yaw_Nm,
                                                           double l_force_x_N,      double l_force_y_N,       double l_force_z_N,
                                                           double l_torque_roll_Nm, double l_torque_pitch_Nm, double l_torque_yaw_Nm)
{
  current_right_fx_N_  = r_force_x_N;
  current_right_fy_N_  = r_force_y_N;
  current_right_fz_N_  = r_force_z_N;
  current_right_tx_Nm_ = r_torque_roll_Nm;
  current_right_ty_Nm_ = r_torque_pitch_Nm;
  current_right_tz_Nm_ = r_torque_yaw_Nm;

  current_left_fx_N_  = l_force_x_N;
  current_left_fy_N_  = l_force_y_N;
  current_left_fz_N_  = l_force_z_N;
  current_left_tx_Nm_ = l_torque_roll_Nm;
  current_left_ty_Nm_ = l_torque_pitch_Nm;
  current_left_tz_Nm_ = l_torque_yaw_Nm;
}

// set maximum adjustment
void RobotisBalanceControl::setMaximumAdjustment(double cob_x_max_adjustment_m,  double cob_y_max_adjustment_m,  double cob_z_max_adjustment_m,
                                                 double cob_roll_max_adjustment_rad, double cob_pitch_max_adjustment_rad, double cob_yaw_max_adjustment_rad,
                                                 double foot_x_max_adjustment_m, double foot_y_max_adjustment_m, double foot_z_max_adjustment_m,
                                                 double foot_roll_max_adjustment_rad, double foot_pitch_max_adjustment_rad, double foot_yaw_max_adjustment_rad)
{
  cob_x_adjustment_abs_max_m_        = cob_x_max_adjustment_m;
  cob_y_adjustment_abs_max_m_        = cob_y_max_adjustment_m;
  cob_z_adjustment_abs_max_m_        = cob_z_max_adjustment_m;
  cob_roll_adjustment_abs_max_rad_   = cob_roll_max_adjustment_rad;
  cob_pitch_adjustment_abs_max_rad_  = cob_pitch_max_adjustment_rad;
  cob_yaw_adjustment_abs_max_rad_    = cob_yaw_max_adjustment_rad;
  foot_x_adjustment_abs_max_m_       = foot_x_max_adjustment_m;
  foot_y_adjustment_abs_max_m_       = foot_y_max_adjustment_m;
  foot_z_adjustment_abs_max_m_       = foot_z_max_adjustment_m;
  foot_roll_adjustment_abs_max_rad_  = foot_roll_max_adjustment_rad;
  foot_pitch_adjustment_abs_max_rad_ = foot_pitch_max_adjustment_rad;
  foot_yaw_adjustment_abs_max_rad_   = foot_yaw_max_adjustment_rad;
}

//Manual Adjustment
void RobotisBalanceControl::setCOBManualAdjustment(double cob_x_adjustment_m, double cob_y_adjustment_m, double cob_z_adjustment_m)
{
  cob_x_manual_adjustment_m_ = cob_x_adjustment_m;
  cob_y_manual_adjustment_m_ = cob_y_adjustment_m;
  cob_z_manual_adjustment_m_ = cob_z_adjustment_m;
}

double RobotisBalanceControl::getCOBManualAdjustmentX()
{
  return cob_x_manual_adjustment_m_;
}

double RobotisBalanceControl::getCOBManualAdjustmentY()
{
  return cob_y_manual_adjustment_m_;
}

double RobotisBalanceControl::getCOBManualAdjustmentZ()
{
  return cob_z_manual_adjustment_m_;
}


void RobotisBalanceControl::setGyroBalanceGainRatio(double gyro_balance_gain_ratio)
{
  gyro_balance_gain_ratio_ = gyro_balance_gain_ratio;
  gyro_balance_roll_gain_  = -0.10*0.75*gyro_balance_gain_ratio_;
  gyro_balance_pitch_gain_ = -0.10*0.5 *gyro_balance_gain_ratio_;
}

double RobotisBalanceControl::getGyroBalanceGainRatio(void)
{
  return gyro_balance_gain_ratio_;
}
