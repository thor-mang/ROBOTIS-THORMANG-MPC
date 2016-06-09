/*
 * Algebra.h
 *
 *  Created on: 2013. 12. 3.
 *      Author: hjsong
 */

#ifndef LINEAR_ALGEBRA_H_
#define LINEAR_ALGEBRA_H_

#include <stdio.h>
#include "robotis_math/robotis_math.h"
#include "step_data_define.h"

namespace thormang3
{

Pose3D getPose3DfromTransformMatrix(Eigen::MatrixXd  matTransform);

}

#endif /* ALGEBRA_H_ */
