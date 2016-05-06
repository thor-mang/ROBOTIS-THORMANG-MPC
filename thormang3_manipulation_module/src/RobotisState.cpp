/*
 * Link.cpp
 *
 *  Created on: Jan 11, 2016
 *      Author: sch
 */

#include "thormang3_manipulation_module/RobotisState.h"

using namespace ROBOTIS;

namespace ROBOTIS_MANIPULATION
{

RobotisState::RobotisState()
{
    is_moving = false;

    /* trajectory */
    cnt = 0;

    mov_time = 1.0;
    smp_time = 0.008;
    all_time_steps = int( mov_time / smp_time ) + 1;

    calc_joint_tra = Eigen::MatrixXd::Zero( all_time_steps , MAX_JOINT_ID + 1 );
    calc_task_tra = Eigen::MatrixXd::Zero( all_time_steps , 3 );

    joint_ini_pose = Eigen::MatrixXd::Zero( MAX_JOINT_ID + 1 , 1 );

    /* ik */
    ik_solve = false;

    ik_target_position = transitionXYZ( 0.0 , 0.0 , 0.0 );

    ik_start_rotation = rpy2rotation( 0.0 , 0.0 , 0.0 );
    ik_target_rotation = rpy2rotation( 0.0 , 0.0 , 0.0 );

    ik_id_start = 0;
    ik_id_end = 0;

    ik_weight = Eigen::MatrixXd::Zero( MAX_JOINT_ID + 1 , 1 );
    ik_weight.fill( 1.0 );
}

RobotisState::~RobotisState(){}

void RobotisState::setInverseKinematics( int cnt ,Eigen::MatrixXd start_rotation )
{
    for ( int dim = 0; dim < 3; dim++ )
        ik_target_position.coeffRef( dim , 0 ) = calc_task_tra.coeff( cnt , dim );

    Eigen::Quaterniond _start_quaternion = rotation2quaternion( start_rotation );

    Eigen::Quaterniond _target_quaternion( goal_kinematics_pose_msg.pose.orientation.w ,
                                           goal_kinematics_pose_msg.pose.orientation.x ,
                                           goal_kinematics_pose_msg.pose.orientation.y ,
                                           goal_kinematics_pose_msg.pose.orientation.z );

    double _cnt = ( double ) cnt / ( double ) all_time_steps;

    Eigen::Quaterniond _quaternion = _start_quaternion.slerp( _cnt , _target_quaternion );

    ik_target_rotation = quaternion2rotation( _quaternion );
}

}
