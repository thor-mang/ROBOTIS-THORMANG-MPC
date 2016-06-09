/*
 *   Walking.h
 *
 *   Author: ROBOTIS
 *
 */

#ifndef _PREVIEW_CONTROL_WALKING__H_
#define _PREVIEW_CONTROL_WALKING__H_

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
  Eigen::MatrixXd A, b, c;
  Eigen::MatrixXd K_x_;
  Eigen::MatrixXd f_;
  double K_s_;
  double sum_of_zmp_x_ ;
  double sum_of_zmp_y_ ;
  double sum_of_cx_ ;
  double sum_of_cy_ ;
  Eigen::MatrixXd u_x, u_y;
  Eigen::MatrixXd x_LIPM, y_LIPM;
  Eigen::MatrixXd x_MPMM, y_MPMM;
  int m_CurrentStartIdxforZMPRef;

  double zmp_ref_x_at_this_time, zmp_ref_y_at_this_time;
  Eigen::MatrixXd m_ZMP_Reference_X, m_ZMP_Reference_Y;
  double m_X_ZMP_CenterShift;
  double m_Y_ZMP_Convergence, m_Y_ZMP_CenterShift;

  bool m_Real_Running, m_Ctrl_Running;

  double m_WalkingTime;    //Absolute Time
  double m_ReferenceTime;  //Absolute Time
  int m_Balancing_Index;
  int m_CurrentStepDataStatus;

  int dir[16];
  int dir_output[16];

  double m_InitAngle[16];
  double r_leg[6], r_arm[6], r_arm_init[6];
  double l_leg[6], l_arm[6], l_arm_init[6];
  double m_ShoulerSwingGain, m_ElbowSwingGain;

  //private methods
  void CalcStepIdxData();
  void CalcRefZMP();
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

  double m_OutAngleRad[16];

  RobotisOnlineWalking();
  virtual ~RobotisOnlineWalking();

  int P_GAIN;
  int I_GAIN;
  int D_GAIN;


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
  double current_right_fx_N,  current_right_fy_N,  current_right_fz_N;
  double current_right_Tx_Nm, current_right_Ty_Nm, current_right_Tz_Nm;
  double current_left_fx_N,  current_left_fy_N,  current_left_fz_N;
  double current_left_Tx_Nm, current_left_Ty_Nm, current_left_Tz_Nm;

  double current_imu_roll_rad, current_imu_pitch_rad;
  double current_gyro_roll_rad_per_sec, current_gyro_pitch_rad_per_sec;


  //for balancing
private:
  double m_Left_Fz_Sigmoid_StartTime;
  double m_Left_Fz_Sigmoid_EndTime;
  double m_Left_Fz_Sigmoid_Target;
  double m_Left_Fz_Sigmoid_Shift;

  double foot_landing_detection_time_sec;

  double foot_r_roll_landing_offset_rad, foot_l_roll_landing_offset_rad;
  double foot_r_pitch_landing_offset_rad, foot_l_pitch_landing_offset_rad;

  double foot_r_roll_adjustment_rad, foot_l_roll_adjustment_rad;
  double foot_r_pitch_adjustment_rad, foot_l_pitch_adjustment_rad;
  double cob_x_adjustment_mm, cob_y_adjustment_mm, cob_z_adjustment_mm;

  double m_gyro_roll_rad_per_sec;
  double m_gyro_pitch_rad_per_sec;

  ////////// Damping Controller
  double hip_roll_adjustment_deg;
  double hip_pitch_adjustment_deg;
  double r_x_adjustment_mm_by_ft, r_y_adjustment_mm_by_ft;
  double l_x_adjustment_mm_by_ft, l_y_adjustment_mm_by_ft;
  double z_adjustment_mm_by_ft;

  Pose3D epr_present, epr_adjustment_by_axis_controller;
  Pose3D epl_present, epl_adjustment_by_axis_controller;

  double foot_r_roll_adjustment_rad_by_ft;
  double foot_r_pitch_adjustment_rad_by_ft;
  double foot_l_roll_adjustment_rad_by_ft;
  double foot_l_pitch_adjustment_rad_by_ft;

  double foot_r_roll_adjustment_rad_by_imu;
  double foot_r_pitch_adjustment_rad_by_imu;
  double foot_l_roll_adjustment_rad_by_imu;
  double foot_l_pitch_adjustment_rad_by_imu;

  double cob_x_adjustment_mm_by_landing_controller;
  double cob_y_adjustment_mm_by_landing_controller;

  ////////// Damping Controller
  double m_init_right_fx_N,  m_init_right_fy_N,  m_init_right_fz_N;
  double m_init_right_Tx_Nm, m_init_right_Ty_Nm, m_init_right_Tz_Nm;

  double m_init_left_fx_N,  m_init_left_fy_N,  m_init_left_fz_N;
  double m_init_left_Tx_Nm, m_init_left_Ty_Nm, m_init_left_Tz_Nm;

  double m_right_ft_scale_factor, m_left_ft_scale_factor;
  double m_right_dsp_fz_N, m_right_ssp_fz_N;
  double m_left_dsp_fz_N, m_left_ssp_fz_N;

  double gyro_roll_init_rad_per_sec, gyro_pitch_init_rad_per_sec, gyro_yaw_init_rad_per_sec;
  double m_iu_roll_init_rad, m_iu_pitch_init_rad, m_iu_yaw_init_rad;

public:
  void initialize();
  void LocalizeAllWalkingParameter();
  void start();
  void stop();
  void process();
  bool isRunning();

  bool AddStepData(StepData step_data);
  void EraseLastStepData();
  int  GetNumofRemainingUnreservedStepData();

  void GetReferenceStepDatafotAddition(StepData *ref_step_data_for_addition);

  void SetRefZMPDecisionParameter(double X_ZMP_CenterShift, double Y_ZMP_CenterShift, double Y_ZMP_Convergence);
  bool SetInitialPose(double r_foot_x, double r_foot_y, double r_foot_z, double r_foot_roll, double r_foot_pitch, double r_foot_yaw,
      double l_foot_x, double l_foot_y, double l_foot_z, double l_foot_roll, double l_foot_pitch, double l_foot_yaw,
      double center_of_body_x, double center_of_body_y, double center_of_body_z,
      double center_of_body_roll, double center_of_body_pitch, double center_of_body_yaw);

  void SetInitalWaistYawAngle(double waist_yaw_angle_rad);

  void SetInitForceTorque(double init_right_fx_N,  double init_right_fy_N,  double init_right_fz_N,
      double init_right_Tx_Nm,double init_right_Ty_Nm,double  init_right_Tz_Nm,
      double init_left_fx_N, double init_left_fy_N, double init_left_fz_N,
      double init_left_Tx_Nm, double init_left_Ty_Nm, double init_left_Tz_Nm);

  void SetInitForceOntheGround(    double right_fx_on_gnd_N, double  right_fy_on_gnd_N,double  right_fz_on_gnd_N,
      double right_tx_on_gnd_N ,double right_ty_on_gnd_N , double right_tz_on_gnd_N,
      double left_fx_on_gnd_N ,  double left_fy_on_gnd_N ,  double left_fz_on_gnd_N,
      double left_tx_on_gnd_N , double left_ty_on_gnd_N ,double left_tz_on_gnd_N);

  void SetFTScaleFactor(double right_ft_scale_factor, double left_ft_scale_factor);

};
}

#endif
