
#ifndef MANIPULATION_MODULE_ROBOTISSTATE_H_
#define MANIPULATION_MODULE_ROBOTISSTATE_H_

#define EIGEN_NO_DEBUG
#define EIGEN_NO_STATIC_ASSERT

#include <std_msgs/Float64.h>
#include <std_msgs/String.h>

#include "robotis_math/RobotisMath.h"
#include "thormang3_kinematics_dynamics/ThorMang3KinematicsDynamics.h"

#include "thormang3_manipulation_module_msgs/JointPose.h"
#include "thormang3_manipulation_module_msgs/KinematicsPose.h"

namespace ROBOTIS_MANIPULATION
{

class RobotisState
{
public:

    RobotisState();
    ~RobotisState();

    bool is_moving;

    /* trajectory */
	int cnt; // counter number

	double mov_time; // movement time
	double smp_time; // sampling time

    int all_time_steps; // all time steps of movement time

    Eigen::MatrixXd calc_joint_tra; // calculated joint trajectory
    Eigen::MatrixXd calc_task_tra; // calculated task trajectory

    Eigen::MatrixXd joint_ini_pose;

    /* msgs */
    thormang3_manipulation_module_msgs::JointPose goal_joint_pose_msg;
    thormang3_manipulation_module_msgs::KinematicsPose goal_kinematics_pose_msg;

    /* ik */
    bool ik_solve;
    Eigen::MatrixXd ik_target_position;
    Eigen::MatrixXd ik_start_rotation , ik_target_rotation;
    int ik_id_start, ik_id_end;

    Eigen::MatrixXd ik_weight;

    void setInverseKinematics(int cnt , Eigen::MatrixXd start_rotation);
};

}

#endif /* MANIPULATION_MODULE_ROBOTISSTATE_H_ */
