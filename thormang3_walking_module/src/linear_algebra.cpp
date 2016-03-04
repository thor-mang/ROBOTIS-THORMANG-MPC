/*
 * LinearAlgebra.cpp
 *
 *  Created on: 2013. 12. 3.
 *      AuROBOTIS: hjsong
 */
#include "thormang3_walking_module/math/linear_algebra.h"

#include <iostream>

using namespace ROBOTIS;

double ROBOTIS::sign(double a)
{
	if(a < 0.0)
		return -1.0;
	else
		return 1.0;

}

matd ROBOTIS::GetOrientationMatrix(double Roll, double Pitch, double Yaw)
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


	return matYaw*matPitch*matRoll;

}

vecd ROBOTIS::GetEulerRollPitchYaw(matd T)
{
	vecd rpy(3);
	rpy(0) = atan2( T(2,1), T(2,2));
	rpy(1) = atan2(-T(2,0), sqrt(T(2,1)*T(2,1) + T(2,2)*T(2,2)) );
	rpy(2) = atan2( T(1,0), T(0,0));

	return rpy;
}

Pose3D ROBOTIS::GetPose3DfromPose3DwithQuaternion(Pose3DwithQuaternion pose_with_quaternion)
{
	Pose3D pose;
	matd R = pose_with_quaternion.quaternion.toRotationMatrix();

	pose.x     = pose_with_quaternion.x;
	pose.y     = pose_with_quaternion.y;
	pose.z     = pose_with_quaternion.z;
	pose.roll  = atan2( R(2,1), R(2,2));
	pose.pitch = atan2(-R(2,0), sqrt(R(2,1)*R(2,1) + R(2,2)*R(2,2)) );
	pose.yaw   = atan2( R(1,0), R(0,0));

	return pose;
}

Pose3DwithQuaternion ROBOTIS::GetPose3DwithQuaternionfromPose3D(Pose3D pose)
{
	Pose3DwithQuaternion pose_with_quaternion;
	matd R4d = GetOrientationMatrix(pose.roll, pose.pitch, pose.yaw);

	Eigen::Matrix3d R3d;
	R3d << R4d(0,0), R4d(0,1), R4d(0,2),
			R4d(1,0), R4d(1,1), R4d(1,2),
			R4d(2,0), R4d(2,1), R4d(2,2);

//	std::cout<<"R4d" <<std::endl;
//	std::cout<< R4d <<std::endl;
//	std::cout<<"R3d" <<std::endl;
//	std::cout<< R3d <<std::endl;

	pose_with_quaternion.x = pose.x;
	pose_with_quaternion.y = pose.y;
	pose_with_quaternion.z = pose.z;


	pose_with_quaternion.quaternion = R3d;

//	std::cout<<"Qua" <<std::endl;
//	std::cout<< pose_with_quaternion.quaternion.toRotationMatrix() <<std::endl;

	return pose_with_quaternion;
}


void ROBOTIS::EulerRollPitchYawToQuaternion(double roll, double pitch, double yaw, double* quat_x, double* quat_y, double* quat_z, double* quat_w)
{
//	matd R4d = GetOrientationMatrix(roll, pitch, yaw);
//	Eigen::Matrix3d R3d;
//	R3d << R4d(0,0), R4d(0,1), R4d(0,2),
//			R4d(1,0), R4d(1,1), R4d(1,2),
//			R4d(2,0), R4d(2,1), R4d(2,2);


	double sr = sin(roll), cr = cos(roll);
	double sp = sin(pitch), cp = cos(pitch);
	double sy = sin(yaw), cy = cos(yaw);

	matd matRoll(3,3);
	matd matPitch(3,3);
	matd matYaw(3,3);

	matRoll << 1,   0,   0,
			0,  cr, -sr,
			0,  sr,  cr;

	matPitch << cp,  0,  sp,
				 0,  1,   0,
			   -sp,  0,  cp;

	matYaw << cy, -sy,  0,
			  sy,  cy,  0,
			   0,   0,  1;

	//Eigen::Matrix3d R3D = (matPitch*matRoll)*matYaw;
	Eigen::Matrix3d R3D = matYaw*(matPitch*matRoll);

	Eigen::Quaterniond _quaternion = Eigen::Quaterniond(R3D);

	*quat_x = _quaternion.x();
	*quat_y = _quaternion.y();
	*quat_z = _quaternion.z();
	*quat_w = _quaternion.w();
}

matd ROBOTIS::GetTranslationMatrix(double x, double y, double z)
{
	matd matTranslation;
	matTranslation.setIdentity(4,4);

	matTranslation(0, 3) = x;
	matTranslation(1, 3) = y;
	matTranslation(2, 3) = z;

	return matTranslation;
}

matd ROBOTIS::GetTransformMatrix(double x, double y, double z, double Roll, double Pitch, double Yaw)
{
	matd matTransform = ROBOTIS::GetOrientationMatrix(Roll, Pitch, Yaw);
	matTransform(0, 3) = x;
	matTransform(1, 3) = y;
	matTransform(2, 3) = z;

	return matTransform;
}

matd ROBOTIS::GetTransformMatrixInverse(matd T)
{
	vecd vecBOA(3); //If T is Transform Matrix A from B, the BOA is translation component coordi. B to coordi. A
	vecd vec_x(3); vecd vec_y(3); vecd vec_z(3);
	matd invT(4,4);

	vecBOA(0) = -T(0,3); vecBOA(1) = -T(1,3); vecBOA(2) = -T(2,3);
	vec_x(0) = T(0,0); vec_x(1) = T(1,0); vec_x(2) = T(2,0);
	vec_y(0) = T(0,1); vec_y(1) = T(1,1); vec_y(2) = T(2,1);
	vec_z(0) = T(0,2); vec_z(1) = T(1,2); vec_z(2) = T(2,2);


	// inv = [   x'   | -AtoB¡¤x ]
	//       [   y'   | -AtoB¡¤y ]
	//       [   z'   | -AtoB¡¤z ]
	//       [  0 0 0  |       1 ]

	invT << vec_x(0), vec_x(1), vec_x(2), Dot(vecBOA, vec_x),
			vec_y(0), vec_y(1), vec_y(2), Dot(vecBOA, vec_y),
			vec_z(0), vec_z(1), vec_z(2), Dot(vecBOA, vec_z),
			      0,        0,        0,                   1;

	return invT;
}

Pose3D ROBOTIS::GetPose3DfromTransformMatrix(matd matTransform)
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

vecd ROBOTIS::Cross(vecd v1, vecd v2)
{
	vecd result(3);
	result(0) = v1(1) * v2(2) - v1(2) * v2(1);
	result(1) = v1(2) * v2(0) - v1(0) * v2(2);
	result(2) = v1(0) * v2(1) - v1(1) * v2(0);

	return result;
}

void ROBOTIS::GetPositionWRTotherCoordinate(matd transform, double position_x, double position_y, double position_z, double* _position_x, double* _position_y, double* _position_z)
{
	matd position;
	position.resize(4,1);
	position.fill(1.0);

	position(0,0) = position_x;
	position(1,0) = position_y;
	position(2,0) = position_z;

	position = transform*position;

	*_position_x = position(0,0);
	*_position_y = position(1,0);
	*_position_z = position(2,0);
}

double ROBOTIS::Dot(vecd v1, vecd v2)
{

	double result = 0;
	if(v1.size() != v2.size() )
	{
		//fprintf(stderr, "The vector size Must be the same.\n");
		return result;
	}
	else
	{
		for(int vec_idx = 0; vec_idx < v1.size(); vec_idx++) {
			result += v1(vec_idx)*v2(vec_idx);
		}

		return result;
	}
}
