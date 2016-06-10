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
 * StepDataDefine.h
 *
 *  Created on: 2013. 12. 7.
 *      Author: Jay Song
 */

#ifndef THORMANG3_WALKING_MODULE_STEP_DATA_DEFINE_H_
#define THORMANG3_WALKING_MODULE_STEP_DATA_DEFINE_H_

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

#endif /* THORMANG3_WALKING_MODULE_STEP_DATA_DEFINE_H_ */
