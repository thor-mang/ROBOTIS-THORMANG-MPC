/*
 * Algebra.h
 *
 *  Created on: 2013. 12. 3.
 *      Author: hjsong
 */

#ifndef LINEAR_ALGEBRA_H_
#define LINEAR_ALGEBRA_H_

#include <math.h>
#include <stdio.h>

#include "step_data_define.h"

#define EIGEN_NO_DEBUG
#define EIGEN_NO_STATIC_ASSERT
//#define EIGEN_DONT_ALIGN_STATICALLY //for 32bit

#include <Eigen/Dense>
#include <Eigen/StdVector>

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Quaterniond)

namespace ROBOTIS
{
	typedef Eigen::MatrixXd matd;
	typedef Eigen::MatrixXi mati;

	typedef Eigen::VectorXd vecd;
	typedef Eigen::VectorXi veci;

	typedef struct
	{
		double x, y, z;
		Eigen::Quaterniond quaternion;
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	}Pose3DwithQuaternion;



	matd GetOrientationMatrix(double Roll, double Pitch, double Yaw);
	matd GetTranslationMatrix(double x, double y, double z);
	matd GetTransformMatrix(double x, double y, double z, double Roll, double Pitch, double Yaw);
	matd GetTransformMatrixInverse(matd T);
	Pose3D GetPose3DfromTransformMatrix(matd matTransform);
	vecd Cross(vecd v1, vecd v2);
	vecd GetEulerRollPitchYaw(matd T);

	Pose3D GetPose3DfromPose3DwithQuaternion(Pose3DwithQuaternion pose_with_quaternion);
	Pose3DwithQuaternion GetPose3DwithQuaternionfromPose3D(Pose3D pose);

	void EulerRollPitchYawToQuaternion(double roll, double pitch, double yaw, double* quat_x, double* quat_y, double* quat_z, double* quat_w);

	void GetPositionWRTotherCoordinate(matd transform, double position_x, double position_y, double position_z, double* _position_x, double* _position_y, double* _position_z);

	double Dot(vecd v1, vecd v2);

	double sign(double a);

}

#endif /* ALGEBRA_H_ */
