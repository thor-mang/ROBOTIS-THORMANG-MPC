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
 * robotis_onlinel_walking.h
 *
 *  Created on: 2016. 6. 10.
 *      Author: Jay Song
 */


#ifndef THORMANG3_WALKING_MODULE_ROBOTIS_ONLINEL_WALKING_H_
#define THORMANG3_WALKING_MODULE_ROBOTIS_ONLINEL_WALKING_H_

#include <vector>
#include <pthread.h>

#include "robotis_framework_common/singleton.h"
#include "math/walking_module_math.h"
#include "thormang3_kinematics_dynamics/kinematics_dynamics.h"


namespace thormang3
{

class RobotisOnlineWalking : public robotis_framework::Singleton<RobotisOnlineWalking>
{
private:
  KinematicsDynamics* thormang3_kd_;

  std::vector<StepData> step_data_;

  double goal_waist_yaw_angle_rad_;
  StepData current_step_data_;
  StepData reference_step_data_for_addition_;
  Pose3D initial_right_foot_pose_, initial_left_foot_pose_, initial_body_pose_;
  Pose3D present_right_foot_pose_, present_left_foot_pose_, present_body_Pose_;
  Pose3D previous_step_right_foot_pose_, previous_step_left_foot_pose_, previous_step_body_pose_;
  double present_waist_yaw_angle_rad_;
  double previous_step_waist_yaw_angle_rad_;

  Eigen::MatrixXd matRFtoRFT, matLFtoLFT;

  Eigen::VectorXi step_idx_data_;
  pthread_mutex_t m_mutex_lock;

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
  double x_zmp_center_shift_;
  double y_zmp_convergence_, y_zmp_center_shift_;

  bool real_running, ctrl_running;

  double walking_time_;    //Absolute Time
  double reference_time_;  //Absolute Time
  int balancing_index_;
  int current_step_data_status_;

  int dir_[16];
  int dir_output_[16];

  double init_angle_[16];
  double r_leg_[6], r_arm_[6], r_arm_init_[6];
  double l_leg_[6], l_arm_[6], l_arm_init_[6];
  double shouler_swing_gain_, elbow_swing_gain_;

  //private methods
  void calcStepIdxData();
  void calcRefZMP();
  double wsin(double time, double period, double period_shift, double mag, double mag_shift);
  double wsigmoid(double time, double period, double time_shift, double mag, double mag_shift, double sigmoid_ratio, double distortion_ratio);


  double GetDampingControllerOutput(double desired, double present, double previous_output, double goal_settling_time);

  double GetHipRollDampingControllerOutput(double desired, double present, double goal_settling_time, double gain);

  double GetHipPitchDampingControllerOutput(double desired, double present, double goal_settling_time, double gain);

  double GetAnkleRollDampingControllerOutput(double desired, double present, double goal_settling_time, double gain);
  double GetAnklePitchDampingControllerOutput(double desired, double present, double goal_settling_time, double gain);

  double GetZDampingControllOutput(double desired, double present, double goal_settling_time, double gain);

  double GetRightXDampingControlOutput(double desired, double present, double goal_settling_time, double gain);
  double GetRightYDampingControlOutput(double desired, double present, double goal_settling_time, double gain);
  double GetLeftXDampingControlOutput(double desired, double present, double goal_settling_time, double gain);
  double GetLeftYDampingControlOutput(double desired, double present, double goal_settling_time, double gain);

  double GetRightAnkleRollMomentDampingControllerOutput(double desired, double present, double goal_settling_time, double gain);
  double GetRightAnklePitchMomentDampingControllerOutput(double desired, double present, double goal_settling_time, double gain);

  double GetLeftAnkleRollMomentDampingControllerOutput(double desired, double present,  double goal_settling_time, double gain);
  double GetLeftAnklePitchMomentDampingControllerOutput(double desired, double present,  double goal_settling_time, double gain);

public:
  Eigen::MatrixXd matCOBtoG, matGtoCOB;
  Eigen::MatrixXd matCOBtoRH, matRHtoCOB;
  Eigen::MatrixXd matCOBtoLH, matLHtoCOB;
  Eigen::MatrixXd matRHtoRF;
  Eigen::MatrixXd matLHtoLF;
  Eigen::MatrixXd matGtoRF, matGtoLF;

  double out_angle_rad_[16];

  RobotisOnlineWalking();
  virtual ~RobotisOnlineWalking();

  int p_gain_;
  int i_gain_;
  int d_gain_;


  double HIP_ROLL_FEEDFORWARD_ANGLE_RAD;
  //OFFSET
  double COB_X_MANUAL_ADJUSTMENT_M;
  double COB_Y_MANUAL_ADJUSTMENT_M;
  double COB_Z_MANUAL_ADJUSTMENT_M;

  //Balancing
  bool BALANCE_ENABLE;
  bool DEBUG_PRINT;
  double HIP_PITCH_OFFSET;
  double ANKLE_PITCH_OFFSET;

  double WALK_STABILIZER_GAIN_RATIO;
  double IMU_GYRO_GAIN_RATIO;
  double FORCE_MOMENT_DISTRIBUTION_RATIO;

  double BALANCE_X_GAIN, BALANCE_Y_GAIN, BALANCE_Z_GAIN;
  double BALANCE_PITCH_GAIN, BALANCE_ROLL_GAIN;

  ////////// Damping Controller
  double BALANCE_HIP_ROLL_GAIN;
  double BALANCE_HIP_PITCH_GAIN;

  double BALANCE_ANKLE_ROLL_GAIN_BY_IMU;
  double BALANCE_ANKLE_PITCH_GAIN_BY_IMU;

  double BALANCE_X_GAIN_BY_FT;
  double BALANCE_Y_GAIN_BY_FT;
  double BALANCE_Z_GAIN_BY_FT;


  double BALANCE_RIGHT_ROLL_GAIN_BY_FT;
  double BALANCE_RIGHT_PITCH_GAIN_BY_FT;

  double BALANCE_LEFT_ROLL_GAIN_BY_FT;
  double BALANCE_LEFT_PITCH_GAIN_BY_FT;


  double BALANCE_HIP_ROLL_TIME_CONSTANT;
  double BALANCE_HIP_PITCH_TIME_CONSTANT;

  double BALANCE_ANKLE_ROLL_TIME_CONSTANT_BY_IMU;
  double BALANCE_ANKLE_PITCH_TIME_CONSTANT_BY_IMU;

  double BALANCE_X_TIME_CONSTANT;
  double BALANCE_Y_TIME_CONSTANT;
  double BALANCE_Z_TIME_CONSTANT;

  double BALANCE_RIGHT_ANKLE_ROLL_TIME_CONSTANT;
  double BALANCE_RIGHT_ANKLE_PITCH_TIME_CONSTANT;

  double BALANCE_LEFT_ANKLE_ROLL_TIME_CONSTANT;
  double BALANCE_LEFT_ANKLE_PITCH_TIME_CONSTANT;


  double AXIS_CONTROLLER_GAIN;
  double AXIS_CONTROLLER_TIME_CONSTANT;


  double LANDING_CONTROLLER_GAIN;
  double LANDING_CONTROLLER_TIME_CONSTANT;

  ////////// Damping Controller

  double FOOT_LANDING_OFFSET_GAIN;
  double FOOT_LANDING_DETECT_N;

  double SYSTEM_CONTROL_UNIT_TIME_SEC;
  double FOOT_LANDING_DETECTION_TIME_MAX_SEC;

  double COB_X_ADJUSTMENT_ABS_MAX_MM, COB_Y_ADJUSTMENT_ABS_MAX_MM, COB_Z_ADJUSTMENT_ABS_MAX_MM;
  double FOOT_ROLL_ADJUSTMENT_ABS_MAX_RAD, FOOT_PITCH_ADJUSTMENT_ABS_MAX_RAD;
  double FOOT_ROLL_ADJUSTMENT_ABS_MAX_RAD_BY_FT, FOOT_PITCH_ADJUSTMENT_ABS_MAX_RAD_BY_FT;

  double X_ADJUSTMENT_ABS_MAX_MM ;
  double Y_ADJUSTMENT_ABS_MAX_MM ;
  double Z_ADJUSTMENT_ABS_MAX_MM ;
  double ROLL_ADJUSTMENT_ABS_MAX_RAD ;
  double PITCH_ADJUSTMENT_ABS_MAX_RAD;

  //sensor value
  double current_right_fx_N_,  current_right_fy_N_,  current_right_fz_N_;
  double current_right_tx_Nm_, current_right_ty_Nm_, current_right_tz_Nm_;
  double current_left_fx_N_,  current_left_fy_N_,  current_left_fz_N_;
  double current_left_tx_Nm_, current_left_ty_Nm_, current_left_tz_Nm_;

  double current_imu_roll_rad_, current_imu_pitch_rad_;
  double current_gyro_roll_rad_per_sec, current_gyro_pitch_rad_per_sec;


  //for balancing
private:
  double left_fz_sigmoid_start_time_;
  double left_fz_sigmoid_end_time_;
  double left_fz_sigmoid_target_;
  double left_fz_sigmoid_shift_;

  double foot_landing_detection_time_sec_;

  double foot_r_roll_landing_offset_rad_,  foot_l_roll_landing_offset_rad_;
  double foot_r_pitch_landing_offset_rad_, foot_l_pitch_landing_offset_rad_;

  double foot_r_roll_adjustment_rad, foot_l_roll_adjustment_rad;
  double foot_r_pitch_adjustment_rad, foot_l_pitch_adjustment_rad;
  double cob_x_adjustment_mm, cob_y_adjustment_mm, cob_z_adjustment_mm;

  double m_gyro_roll_rad_per_sec;
  double m_gyro_pitch_rad_per_sec;

  ////////// Damping Controller
  double hip_roll_adjustment_deg_;
  double hip_pitch_adjustment_deg_;
  double r_x_adjustment_mm_by_ft_, r_y_adjustment_mm_by_ft_;
  double l_x_adjustment_mm_by_ft_, l_y_adjustment_mm_by_ft_;
  double z_adjustment_mm_by_ft_;

  Pose3D epr_present_, epr_adjustment_by_axis_controller_;
  Pose3D epl_present_, epl_adjustment_by_axis_controller_;

  double foot_r_roll_adjustment_rad_by_ft_;
  double foot_r_pitch_adjustment_rad_by_ft_;
  double foot_l_roll_adjustment_rad_by_ft_;
  double foot_l_pitch_adjustment_rad_by_ft_;

  double foot_r_roll_adjustment_rad_by_imu_;
  double foot_r_pitch_adjustment_rad_by_imu_;
  double foot_l_roll_adjustment_rad_by_imu_;
  double foot_l_pitch_adjustment_rad_by_imu_;

  double cob_x_adjustment_mm_by_landing_controller_;
  double cob_y_adjustment_mm_by_landing_controller_;

  ////////// Damping Controller
  double init_right_fx_N_,  init_right_fy_N_,  init_right_fz_N_;
  double init_right_tx_Nm_, init_right_ty_Nm_, init_right_tz_Nm_;

  double init_left_fx_N_,  init_left_fy_N_,  init_left_fz_N_;
  double init_left_tx_Nm_, init_left_ty_Nm_, init_left_tz_Nm_;

  double right_ft_scale_factor_, left_ft_scale_factor_;
  double right_dsp_fz_N_, right_ssp_fz_N_;
  double left_dsp_fz_N_,  left_ssp_fz_N_;

  double gyro_roll_init_rad_per_sec_, gyro_pitch_init_rad_per_sec_, gyro_yaw_init_rad_per_sec_;
  double m_iu_roll_init_rad_, m_iu_pitch_init_rad_, m_iu_yaw_init_rad_;

public:
  void initialize();
  void LocalizeAllWalkingParameter();
  void start();
  void stop();
  void process();
  bool isRunning();

  bool addStepData(StepData step_data);
  void eraseLastStepData();
  int  getNumofRemainingUnreservedStepData();

  void getReferenceStepDatafotAddition(StepData *ref_step_data_for_addition);

  void setRefZMPDecisionParameter(double X_ZMP_CenterShift, double Y_ZMP_CenterShift, double Y_ZMP_Convergence);
  bool setInitialPose(double r_foot_x, double r_foot_y, double r_foot_z, double r_foot_roll, double r_foot_pitch, double r_foot_yaw,
      double l_foot_x, double l_foot_y, double l_foot_z, double l_foot_roll, double l_foot_pitch, double l_foot_yaw,
      double center_of_body_x, double center_of_body_y, double center_of_body_z,
      double center_of_body_roll, double center_of_body_pitch, double center_of_body_yaw);

  void setInitalWaistYawAngle(double waist_yaw_angle_rad);

  void setInitForceTorque(double init_right_fx_N,  double init_right_fy_N,  double init_right_fz_N,
      double init_right_Tx_Nm,double init_right_Ty_Nm,double  init_right_Tz_Nm,
      double init_left_fx_N, double init_left_fy_N, double init_left_fz_N,
      double init_left_Tx_Nm, double init_left_Ty_Nm, double init_left_Tz_Nm);

  void setInitForceOntheGround(    double right_fx_on_gnd_N, double  right_fy_on_gnd_N,double  right_fz_on_gnd_N,
      double right_tx_on_gnd_N ,double right_ty_on_gnd_N , double right_tz_on_gnd_N,
      double left_fx_on_gnd_N ,  double left_fy_on_gnd_N ,  double left_fz_on_gnd_N,
      double left_tx_on_gnd_N , double left_ty_on_gnd_N ,double left_tz_on_gnd_N);

  void setFTScaleFactor(double right_ft_scale_factor, double left_ft_scale_factor);

};
}

#endif /* THORMANG3_WALKING_MODULE_ROBOTIS_ONLINEL_WALKING_H_ */
