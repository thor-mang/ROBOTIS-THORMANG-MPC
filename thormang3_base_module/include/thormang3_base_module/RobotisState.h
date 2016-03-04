
#ifndef BASE_MODULE_ROBOTISSTATE_H_
#define BASE_MODULE_ROBOTISSTATE_H_

#define EIGEN_NO_DEBUG
#define EIGEN_NO_STATIC_ASSERT

#include <std_msgs/Float64.h>
#include <std_msgs/String.h>

#include "RobotisCommon.h"

namespace ROBOTIS_BASE
{

class RobotisState
{
public:

    RobotisState();
    ~RobotisState();

    bool is_moving;

	int cnt; // counter number

	double mov_time; // movement time
	double smp_time; // sampling time

    int all_time_steps; // all time steps of movement time

    Eigen::MatrixXd calc_joint_tra; // calculated joint trajectory

    Eigen::MatrixXd joint_ini_pose;
    Eigen::MatrixXd joint_pose;

    int via_num;

    Eigen::MatrixXd joint_via_pose;
    Eigen::MatrixXd joint_via_dpose;
    Eigen::MatrixXd joint_via_ddpose;

    Eigen::MatrixXd via_time;

    // msgs
    std_msgs::String send_tra_msg;


};

}

#endif /* BASE_MODULE_ROBOTISSTATE_H_ */
