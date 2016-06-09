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


matd GetOrientationMatrix(double Roll, double Pitch, double Yaw)
{
  double sr = sin(Roll), cr = cos(Roll);
  double sp = sin(Pitch), cp = cos(Pitch);
  double sy = sin(Yaw), cy = cos(Yaw);

  matd matRoll(4,4);
  matd matPitch(4,4);
  matd matYaw(4,4);

  matRoll << 1,   0,   0,  0,
      0,  cr, -sr,  0,
      0,  sr,  cr,  0,
      0,   0,   0,  1;

  matPitch << cp,  0,  sp,  0,
      0,  1,   0,  0,
      -sp,  0,  cp,  0,
      0,  0,   0,  1;

  matYaw << cy, -sy,  0,  0,
      sy,  cy,  0,  0,
      0,   0,  1,  0,
      0,   0,  0,  1;


  return (matYaw*matPitch)*matRoll;

}

vecd GetEulerRollPitchYaw(matd T)
{
  vecd rpy(3);
  rpy(0) = atan2( T(2,1), T(2,2));
  rpy(1) = atan2(-T(2,0), sqrt(T(2,1)*T(2,1) + T(2,2)*T(2,2)) );
  rpy(2) = atan2( T(1,0), T(0,0));

  return rpy;
}


matd GetTranslationMatrix(double x, double y, double z)
{
  matd matTranslation;
  matTranslation.setIdentity(4,4);

  matTranslation(0, 3) = x;
  matTranslation(1, 3) = y;
  matTranslation(2, 3) = z;

  return matTranslation;
}

matd GetTransformMatrix(double x, double y, double z, double Roll, double Pitch, double Yaw)
{
  //    matd matTransform = GetOrientationMatrix(Roll, Pitch, Yaw);
  //    matTransform(0, 3) = x;
  //    matTransform(1, 3) = y;
  //    matTransform(2, 3) = z;

  //return matTransform;


  return robotis_framework::getTransformationXYZRPY(x, y, z, Roll, Pitch, Yaw);
}

matd GetTransformMatrixInverse(matd T)
{
  vecd vecBOA(3); //If T is Transform Matrix A from B, the BOA is translation component coordi. B to coordi. A
  vecd vec_x(3); vecd vec_y(3); vecd vec_z(3);
  matd invT(4,4);

  vecBOA(0) = -T(0,3); vecBOA(1) = -T(1,3); vecBOA(2) = -T(2,3);
  vec_x(0) = T(0,0); vec_x(1) = T(1,0); vec_x(2) = T(2,0);
  vec_y(0) = T(0,1); vec_y(1) = T(1,1); vec_y(2) = T(2,1);
  vec_z(0) = T(0,2); vec_z(1) = T(1,2); vec_z(2) = T(2,2);


  // inv = [   x'   | -AtoB��x ]
  //       [   y'   | -AtoB��y ]
  //       [   z'   | -AtoB��z ]
  //       [  0 0 0  |       1 ]

  invT << vec_x(0), vec_x(1), vec_x(2), robotis_framework::calcInner(vecBOA, vec_x),
      vec_y(0), vec_y(1), vec_y(2), robotis_framework::calcInner(vecBOA, vec_y),
      vec_z(0), vec_z(1), vec_z(2), robotis_framework::calcInner(vecBOA, vec_z),
      0,        0,        0,                   1;

  return invT;
}

Pose3D GetPose3DfromTransformMatrix(matd matTransform)
{
  Pose3D _tempPose3D;

  _tempPose3D.x     = matTransform(0, 3);
  _tempPose3D.y     = matTransform(1, 3);
  _tempPose3D.z     = matTransform(2, 3);
  _tempPose3D.roll  = atan2( matTransform(2,1), matTransform(2,2));
  _tempPose3D.pitch = atan2(-matTransform(2,0), sqrt(matTransform(2,1)*matTransform(2,1) + matTransform(2,2)*matTransform(2,2)) );
  _tempPose3D.yaw   = atan2( matTransform(1,0), matTransform(0,0));

  return _tempPose3D;
}

vecd Cross(vecd v1, vecd v2)
{
  vecd result(3);
  result(0) = v1(1) * v2(2) - v1(2) * v2(1);
  result(1) = v1(2) * v2(0) - v1(0) * v2(2);
  result(2) = v1(0) * v2(1) - v1(1) * v2(0);

  return result;
}

}
