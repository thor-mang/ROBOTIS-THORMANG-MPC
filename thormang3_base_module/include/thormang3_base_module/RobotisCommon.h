#ifndef BASE_MODULE_ROBOTISCOMMON_H_
#define BASE_MODULE_ROBOTISCOMMON_H_

#define EIGEN_NO_DEBUG
#define EIGEN_NO_STATIC_ASSERT

#include <Eigen/Dense>

namespace ROBOTIS_BASE
{

#define PI	3.141592

#define MAX_JOINT_ID    31 // 29 + 2
#define ALL_JOINT_ID    46

#define MAX_ARM_ID      7
#define MAX_LEG_ID      6
#define MAX_ITER        5

#define id_head_end     29
#define id_cob          44
#define id_torso        27

#define id_r_arm_start   1
#define id_l_arm_start   2

#define id_r_arm_end    35
#define id_l_arm_end    34

#define id_r_leg_end    45
#define id_l_leg_end    46

#define deg2rad 	(M_PI / 180.0)
#define rad2deg 	(180.0 / M_PI)

#define PRINT_MAT(X) std::cout << #X << ":\n" << X << std::endl << std::endl

}

#endif /* BASE_MODULE_ROBOTISCOMMON_H_ */
