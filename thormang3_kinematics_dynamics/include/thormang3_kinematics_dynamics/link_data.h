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
 * link_data.h
 *
 *  Created on: June 7, 2016
 *      Author: sch
 */

#ifndef THORMANG3_KINEMATICS_DYNAMICS_LINK_DATA_H_
#define THORMANG3_KINEMATICS_DYNAMICS_LINK_DATA_H_

#include "robotis_math/robotis_math.h"

namespace thormang3
{

class LinkData
{
public:
  LinkData();
  ~LinkData();

  std::string name;

  int parent;
  int sibling;
  int child;

  double mass;

  Eigen::MatrixXd relative_position;
  Eigen::MatrixXd joint_axis;
  Eigen::MatrixXd center_of_mass;
  Eigen::MatrixXd inertia;

  double joint_limit_max;
  double joint_limit_min;

  double joint_angle;
  double joint_velocity;
  double joint_acceleration;

  Eigen::MatrixXd position;
  Eigen::MatrixXd orientation;
  Eigen::MatrixXd transformation;
};

}

#endif /* THORMANG3_KINEMATICS_DYNAMICS_LINK_DATA_H_ */
