/*
 * StepDataDefine.h
 *
 *  Created on: 2013. 12. 7.
 *      Author: hjsong
 */

#ifndef STEPDATADEFINE_H_
#define STEPDATADEFINE_H_

namespace thormang3
{

typedef struct
{
  double x, y, z;
} Position3D;

typedef struct
{
  double x, y, z, roll, pitch, yaw;
} Pose3D;

typedef struct
{
  int    moving_foot;
  double foot_z_swap, body_z_swap;
  double shoulder_swing_gain, elbow_swing_gain;
  double waist_roll_angle, waist_pitch_angle, waist_yaw_angle;
  Pose3D left_foot_pose;
  Pose3D right_foot_pose;
  Pose3D body_pose;
} StepPositionData;

typedef struct
{
  int    walking_state;
  double abs_step_time, dsp_ratio;
  double sigmoid_ratio_x, sigmoid_ratio_y, sigmoid_ratio_z, sigmoid_ratio_roll, sigmoid_ratio_pitch, sigmoid_ratio_yaw;
  double sigmoid_distortion_x, sigmoid_distortion_y, sigmoid_distortion_z, sigmoid_distortion_roll, sigmoid_distortion_pitch, sigmoid_distortion_yaw;
} StepTimeData;

typedef struct
{
  StepPositionData position_data;
  StepTimeData     time_data;
} StepData;

}


#endif /* STEPDATADEFINE_H_ */
