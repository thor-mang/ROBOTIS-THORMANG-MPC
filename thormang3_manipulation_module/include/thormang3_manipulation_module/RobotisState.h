
#ifndef MANIPULATION_MODULE_ROBOTISSTATE_H_
#define MANIPULATION_MODULE_ROBOTISSTATE_H_

#define EIGEN_NO_DEBUG
#define EIGEN_NO_STATIC_ASSERT

#include <std_msgs/Float64.h>
#include <std_msgs/String.h>

#include "RobotisCommon.h"

#include "thormang3_manipulation_module_msgs/DemoPose.h"
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

	int cnt; // counter number

	double mov_time; // movement time
	double smp_time; // sampling time

    int all_time_steps; // all time steps of movement time

    Eigen::MatrixXd calc_joint_tra; // calculated joint trajectory
    Eigen::MatrixXd calc_task_tra; // calculated task trajectory

    Eigen::MatrixXd joint_ini_pose;

    int via_num;
    Eigen::MatrixXd via_time;

    Eigen::MatrixXd task_via_pose;
    Eigen::MatrixXd task_via_dpose;
    Eigen::MatrixXd task_via_ddpose;

    Eigen::MatrixXd l_arm_task_via_pose , r_arm_task_via_pose;
    Eigen::MatrixXd l_arm_task_via_dpose , r_arm_task_via_dpose;
    Eigen::MatrixXd l_arm_task_via_ddpose , r_arm_task_via_ddpose;

    bool demo_solve;

    // for gui
    thormang3_manipulation_module_msgs::JointPose goal_joint_pose_msg;
    thormang3_manipulation_module_msgs::KinematicsPose goal_kinematics_pose_msg;
    thormang3_manipulation_module_msgs::DemoPose goal_demo_pose_msg;

    // inverse kinematics
    bool ik_solve;
    Eigen::MatrixXd ik_target_position;
    Eigen::MatrixXd ik_start_rotation , ik_target_rotation;
    int ik_id_start;
    int ik_id_end;

    bool ik_bi_solve;
    Eigen::MatrixXd ik_l_arm_target_position , ik_r_arm_target_position;
    Eigen::MatrixXd ik_l_arm_start_rotation , ik_r_arm_start_rotation;
    Eigen::MatrixXd ik_l_arm_end_rotation , ik_r_arm_end_rotation;

    Eigen::MatrixXd ik_l_arm_target_rotation , ik_r_arm_target_rotation;

    Eigen::MatrixXd calc_l_arm_task_tra , calc_r_arm_task_tra; // calculated task trajectory

    // msgs
    std_msgs::String send_tra_msg;

    //
    Eigen::MatrixXd ik_weight;

    void setInverseKinematics(int cnt , Eigen::MatrixXd start_rotation);
    void setInverseKinematicsDemo(int cnt , Eigen::MatrixXd start_rotation);

    void setInverseKinematicsBiManual(int cnt ,
                                      Eigen::MatrixXd l_arm_start_rotation , Eigen::MatrixXd r_arm_start_rotation ,
                                      Eigen::MatrixXd l_arm_target_rotation , Eigen::MatrixXd r_arm_target_rotation );
};

}

#endif /* MANIPULATION_MODULE_ROBOTISSTATE_H_ */
