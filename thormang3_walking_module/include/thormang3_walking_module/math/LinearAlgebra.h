/*
 * Algebra.h
 *
 *  Created on: 2013. 12. 3.
 *      Author: hjsong
 */

#ifndef LINEAR_ALGEBRA_H_
#define LINEAR_ALGEBRA_H_

#include <stdio.h>
#include "robotis_math/RobotisMath.h"
#include "StepDataDefine.h"

namespace ROBOTIS
{
	typedef Eigen::MatrixXd matd;
	typedef Eigen::MatrixXi mati;

	typedef Eigen::VectorXd vecd;
	typedef Eigen::VectorXi veci;


	matd GetOrientationMatrix(double Roll, double Pitch, double Yaw);
	matd GetTranslationMatrix(double x, double y, double z);
	matd GetTransformMatrix(double x, double y, double z, double Roll, double Pitch, double Yaw);
	matd GetTransformMatrixInverse(matd T);
	Pose3D GetPose3DfromTransformMatrix(matd matTransform);
	vecd Cross(vecd v1, vecd v2);
	vecd GetEulerRollPitchYaw(matd T);
}

#endif /* ALGEBRA_H_ */
