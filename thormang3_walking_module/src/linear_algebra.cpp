/*
 * LinearAlgebra.cpp
 *
 *  Created on: 2013. 12. 3.
 *      AuROBOTIS: hjsong
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
