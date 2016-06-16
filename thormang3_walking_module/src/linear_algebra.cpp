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
 * linear_algebra.cpp
 *
 *  Created on: 2013. 12. 3.
 *      Author: Jay Song
 */

#include <iostream>
#include "../include/thormang3_walking_module/math/linear_algebra.h"

namespace thormang3
{

Pose3D getPose3DfromTransformMatrix(Eigen::MatrixXd matTransform)
{
  Pose3D pose_3d;

  pose_3d.x     = matTransform.coeff(0, 3);
  pose_3d.y     = matTransform.coeff(1, 3);
  pose_3d.z     = matTransform.coeff(2, 3);
  pose_3d.roll  = atan2( matTransform.coeff(2,1), matTransform.coeff(2,2));
  pose_3d.pitch = atan2(-matTransform.coeff(2,0), sqrt(matTransform.coeff(2,1)*matTransform.coeff(2,1) + matTransform.coeff(2,2)*matTransform.coeff(2,2)) );
  pose_3d.yaw   = atan2( matTransform.coeff(1,0), matTransform.coeff(0,0));

  return pose_3d;
}

}
