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
 * thormang3_online_walking.h
 *
 *  Created on: 2016. 6. 10.
 *      Author: Jay Song
 */


#ifndef THORMANG3_WALKING_MODULE_THORMANG3_ONLINEL_WALKING_H_
#define THORMANG3_WALKING_MODULE_THORMANG3_ONLINEL_WALKING_H_

#include <vector>
#include <boost/thread.hpp>

#include "thormang3_balance_control/thormang3_balance_control.h"
#include "robotis_framework_common/singleton.h"
#include "thormang3_kinematics_dynamics/kinematics_dynamics.h"
#include "robotis_math/robotis_math.h"

#include <iostream>
#include <fstream>

#define _USE_PD_BALANCE_

namespace thormang3
{

class THORMANG3OnlineWalking : public robotis_framework::Singleton<THORMANG3OnlineWalking>
{
public:
  THORMANG3OnlineWalking();
  virtual ~THORMANG3OnlineWalking();

  void initialize();
  void reInitialize();
  void start();
  void stop();
  void process();
  bool isRunning();

  void printStepIdxData(Eigen::VectorXi idx_data);

  bool addStepData(robotis_framework::StepData step_data);
  void eraseLastStepData();
  int  getNumofRemainingUnreservedStepData();
  void getReferenceStepDatafotAddition(robotis_framework::StepData *ref_step_data_for_addition);

  void setRefZMPDecisionParameter(double X_ZMP_CenterShift, double Y_ZMP_CenterShift, double Y_ZMP_Convergence);

  bool setInitialPose(double r_foot_x, double r_foot_y, double r_foot_z, double r_foot_roll, double r_foot_pitch, double r_foot_yaw,
      double l_foot_x, double l_foot_y, double l_foot_z, double l_foot_roll, double l_foot_pitch, double l_foot_yaw,
      double center_of_body_x, double center_of_body_y, double center_of_body_z,
      double center_of_body_roll, double center_of_body_pitch, double center_of_body_yaw);

  void setInitalWaistYawAngle(double waist_yaw_angle_rad);

  void setInitialRightShoulderAngle(double shoulder_angle_rad);
  void setInitialLeftShoulderAngle(double shoulder_angle_rad);
  void setInitialRightElbowAngle(double elbow_angle_rad);
  void setInitialLeftElbowAngle(double elbow_angle_rad);

  void setCurrentIMUSensorOutput(double gyro_x, double gyro_y, double quat_x, double quat_y, double quat_z, double quat_w);

  Eigen::MatrixXd mat_cob_to_g_,  mat_g_to_cob_;
  Eigen::MatrixXd mat_robot_to_cob_, mat_cob_to_robot_;
  Eigen::MatrixXd mat_robot_to_g_, mat_g_to_robot_;
  Eigen::MatrixXd mat_cob_to_rhip_, mat_rhip_to_cob_;
  Eigen::MatrixXd mat_cob_to_lhip_, mat_lhip_to_cob_;

  Eigen::MatrixXd mat_g_to_rfoot_, mat_g_to_lfoot_;

  double r_shoulder_out_angle_rad_;
  double l_shoulder_out_angle_rad_;
  double r_elbow_out_angle_rad_;
  double l_elbow_out_angle_rad_;
  double r_leg_out_angle_rad_[6];
  double l_leg_out_angle_rad_[6];
  double out_angle_rad_[16];

  double hip_roll_feedforward_angle_rad_;

  double curr_angle_rad_[12];
  thormang3::BalancePDController leg_angle_feed_back_[12];

  // balance control
  int balance_error_;
  thormang3::BalanceControlUsingPDController balance_ctrl_;


  // sensor value
  double current_right_fx_N_,  current_right_fy_N_,  current_right_fz_N_;
  double current_right_tx_Nm_, current_right_ty_Nm_, current_right_tz_Nm_;
  double current_left_fx_N_,  current_left_fy_N_,  current_left_fz_N_;
  double current_left_tx_Nm_, current_left_ty_Nm_, current_left_tz_Nm_;

  Eigen::Quaterniond quat_current_imu_;
  Eigen::MatrixXd mat_current_imu_;
  double current_imu_roll_rad_, current_imu_pitch_rad_;
  double current_gyro_roll_rad_per_sec_, current_gyro_pitch_rad_per_sec_;

private:
  std::string toString(Eigen::MatrixXd& mat);

  void calcStepIdxData();
  void calcRefZMP();
  void calcDesiredPose();

  double wsin(double time, double period, double period_shift, double mag, double mag_shift);
  double wsigmoid(double time, double period, double time_shift, double mag, double mag_shift, double sigmoid_ratio, double distortion_ratio);

  KinematicsDynamics* thormang3_kd_;

  double left_fz_trajectory_start_time_;
  double left_fz_trajectory_end_time_;
  double left_fz_trajectory_target_;
  double left_fz_trajectory_shift_;

  double total_mass_of_robot_;
  double right_dsp_fz_N_, right_ssp_fz_N_;
  double left_dsp_fz_N_,  left_ssp_fz_N_;

  Eigen::MatrixXd mat_robot_to_cob_modified_, mat_cob_to_robot_modified_;
  Eigen::MatrixXd mat_robot_to_rf_modified_;
  Eigen::MatrixXd mat_robot_to_lf_modified_;
  Eigen::MatrixXd mat_robot_to_rfoot_;
  Eigen::MatrixXd mat_robot_to_lfoot_;

  std::vector<robotis_framework::StepData> added_step_data_;

  double goal_waist_yaw_angle_rad_;
  robotis_framework::StepData current_step_data_;
  robotis_framework::StepData reference_step_data_for_addition_;
  robotis_framework::Pose3D initial_right_foot_pose_, initial_left_foot_pose_, initial_body_pose_;
  robotis_framework::Pose3D present_right_foot_pose_, present_left_foot_pose_, present_body_pose_;
  robotis_framework::Pose3D previous_step_right_foot_pose_, previous_step_left_foot_pose_, previous_step_body_pose_;
  robotis_framework::Pose3D rhip_to_rfoot_pose_, lhip_to_lfoot_pose_;
  robotis_framework::FifthOrderPolynomialTrajectory foot_x_tra_,    foot_y_tra_,     foot_z_tra_;
  robotis_framework::FifthOrderPolynomialTrajectory foot_roll_tra_, foot_pitch_tra_, foot_yaw_tra_;
  robotis_framework::FifthOrderPolynomialTrajectory foot_z_swap_tra_;
  robotis_framework::FifthOrderPolynomialTrajectory body_z_tra_, body_roll_tra_, body_pitch_tra_, body_yaw_tra_;
  robotis_framework::FifthOrderPolynomialTrajectory waist_yaw_tra_;
  robotis_framework::FifthOrderPolynomialTrajectory body_z_swap_tra_;
  robotis_framework::FifthOrderPolynomialTrajectory hip_roll_swap_tra_;

  double present_waist_yaw_angle_rad_;
  double previous_step_waist_yaw_angle_rad_;
  double r_init_shoulder_angle_rad_, r_init_elbow_angle_rad_;
  double l_init_shoulder_angle_rad_, l_init_elbow_angle_rad_;
  double r_shoulder_dir_, r_elbow_dir_;
  double l_shoulder_dir_, l_elbow_dir_;
  double shouler_swing_gain_, elbow_swing_gain_;

  Eigen::MatrixXd mat_rfoot_to_rft_, mat_lfoot_to_lft_;
  Eigen::MatrixXd rot_x_pi_3d_, rot_z_pi_3d_;

  Eigen::VectorXi step_idx_data_;
  boost::mutex step_data_mutex_lock_;
  boost::mutex imu_data_mutex_lock_;

  //Time for Preview Control and Dynamics Regulator
  double preview_time_;
  int preview_size_;

  //These matrix and parameters are for preview control
  Eigen::MatrixXd A_, b_, c_;
  Eigen::MatrixXd k_x_;
  Eigen::MatrixXd f_;
  double k_s_;
  double sum_of_zmp_x_ ;
  double sum_of_zmp_y_ ;
  double sum_of_cx_ ;
  double sum_of_cy_ ;
  Eigen::MatrixXd u_x, u_y;
  Eigen::MatrixXd x_lipm_, y_lipm_;

  int current_start_idx_for_ref_zmp_;

  double ref_zmp_x_at_this_time_, ref_zmp_y_at_this_time_;
  Eigen::MatrixXd reference_zmp_x_, reference_zmp_y_;

  bool real_running, ctrl_running;

  double walking_time_;    //Absolute Time
  double reference_time_;  //Absolute Time
  int balancing_index_;
  int current_step_data_status_;

  int step_counter;

  std::FILE* file;
  bool isClosed;
  bool firstTime;
};

}

#endif /* THORMANG3_WALKING_MODULE_THORMANG3_ONLINEL_WALKING_H_ */
