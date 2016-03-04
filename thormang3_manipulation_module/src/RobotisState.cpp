/*
 * Link.cpp
 *
 *  Created on: Jan 11, 2016
 *      Author: sch
 */

#include "thormang3_manipulation_module/RobotisCommon.h"
#include "thormang3_manipulation_module/RobotisState.h"
#include "thormang3_manipulation_module/Transformation.h"

namespace ROBOTIS_MANIPULATION
{

RobotisState::RobotisState()
{
    is_moving = false;

    cnt = 0;

    mov_time = 1.0;
    smp_time = 0.008;
    all_time_steps = int( mov_time / smp_time ) + 1;

    calc_joint_tra = Eigen::MatrixXd::Zero( all_time_steps , MAX_JOINT_ID + 1 );
    calc_task_tra = Eigen::MatrixXd::Zero( all_time_steps , 3 );

    joint_ini_pose = Eigen::MatrixXd::Zero( MAX_JOINT_ID + 1 , 1 );

    via_num = 1;
    via_time = Eigen::MatrixXd::Zero( via_num , 1 );

    task_via_pose = Eigen::MatrixXd::Zero( via_num , 3 );
    task_via_dpose = Eigen::MatrixXd::Zero( via_num , 3 );
    task_via_ddpose = Eigen::MatrixXd::Zero( via_num , 3 );

    l_arm_task_via_pose = Eigen::MatrixXd::Zero( via_num , 3 );
    r_arm_task_via_pose = Eigen::MatrixXd::Zero( via_num , 3 );
    l_arm_task_via_dpose = Eigen::MatrixXd::Zero( via_num , 3 );
    r_arm_task_via_dpose = Eigen::MatrixXd::Zero( via_num , 3 );
    l_arm_task_via_ddpose = Eigen::MatrixXd::Zero( via_num , 3 );
    r_arm_task_via_ddpose = Eigen::MatrixXd::Zero( via_num , 3 );

    // for gui
    goal_joint_pose_msg.name = "";
    goal_joint_pose_msg.value = 0.0;

    goal_kinematics_pose_msg.name = "";
    goal_kinematics_pose_msg.pose.position.x = 0.0;
    goal_kinematics_pose_msg.pose.position.y = 0.0;
    goal_kinematics_pose_msg.pose.position.z = 0.0;
    goal_kinematics_pose_msg.pose.orientation.x = 0.0;
    goal_kinematics_pose_msg.pose.orientation.y = 0.0;
    goal_kinematics_pose_msg.pose.orientation.z = 0.0;
    goal_kinematics_pose_msg.pose.orientation.w = 1.0;

    goal_demo_pose_msg.name = "";
    goal_demo_pose_msg.demo = "";
    goal_demo_pose_msg.pose.position.x = 0.0;
    goal_demo_pose_msg.pose.position.y = 0.0;
    goal_demo_pose_msg.pose.position.z = 0.0;
    goal_demo_pose_msg.pose.orientation.x = 0.0;
    goal_demo_pose_msg.pose.orientation.y = 0.0;
    goal_demo_pose_msg.pose.orientation.z = 0.0;
    goal_demo_pose_msg.pose.orientation.w = 1.0;

    demo_solve = false;

    // for inverse kinematics;
    ik_solve = false;

    ik_target_position = transitionXYZ( 0.0 , 0.0 , 0.0 );

    ik_start_rotation = rpy2rotation( 0.0 , 0.0 , 0.0 );
    ik_target_rotation = rpy2rotation( 0.0 , 0.0 , 0.0 );

    ik_id_start = 0;
    ik_id_end = 0;

    ik_bi_solve = false;
    ik_l_arm_target_position = transitionXYZ( 0.0 , 0.0 , 0.0 );
    ik_r_arm_target_position = transitionXYZ( 0.0 , 0.0 , 0.0 );

    ik_l_arm_start_rotation = rpy2rotation( 0.0 , 0.0 , 0.0 );
    ik_r_arm_start_rotation = rpy2rotation( 0.0 , 0.0 , 0.0 );
    ik_l_arm_target_rotation = rpy2rotation( 0.0 , 0.0 , 0.0 );
    ik_r_arm_target_rotation = rpy2rotation( 0.0 , 0.0 , 0.0 );

    ik_l_arm_end_rotation = rpy2rotation( 0.0 , 0.0 , 0.0 );
    ik_r_arm_end_rotation = rpy2rotation( 0.0 , 0.0 , 0.0 );

    calc_l_arm_task_tra = Eigen::MatrixXd::Zero( all_time_steps , 3 );
    calc_r_arm_task_tra = Eigen::MatrixXd::Zero( all_time_steps , 3 );

    // msgs
    send_tra_msg.data = "";

    //
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

void RobotisState::setInverseKinematicsDemo( int cnt ,Eigen::MatrixXd start_rotation )
{
    for ( int dim = 0; dim < 3; dim++ )
        ik_target_position.coeffRef( dim , 0 ) = calc_task_tra.coeff( cnt , dim );

    Eigen::Quaterniond _start_quaternion = rotation2quaternion( start_rotation );

    Eigen::Quaterniond _target_quaternion( goal_demo_pose_msg.pose.orientation.w ,
                                           goal_demo_pose_msg.pose.orientation.x ,
                                           goal_demo_pose_msg.pose.orientation.y ,
                                           goal_demo_pose_msg.pose.orientation.z );

    double _cnt = ( double ) cnt / ( double ) all_time_steps;

    Eigen::Quaterniond _quaternion = _start_quaternion.slerp( _cnt , _target_quaternion );

    ik_target_rotation = quaternion2rotation( _quaternion );
}

void RobotisState::setInverseKinematicsBiManual( int cnt ,
                                                 Eigen::MatrixXd l_arm_start_rotation , Eigen::MatrixXd r_arm_start_rotation ,
                                                 Eigen::MatrixXd l_arm_target_rotation , Eigen::MatrixXd r_arm_target_rotation )
{
    for ( int dim = 0; dim < 3; dim++ )
    {
        ik_l_arm_target_position.coeffRef( dim , 0 ) = calc_l_arm_task_tra.coeff( cnt , dim );
        ik_r_arm_target_position.coeffRef( dim , 0 ) = calc_r_arm_task_tra.coeff( cnt , dim );
    }

    Eigen::Quaterniond _l_arm_start_quaternion = rotation2quaternion( l_arm_start_rotation );
    Eigen::Quaterniond _l_arm_target_quaternion = rotation2quaternion( l_arm_target_rotation );

    Eigen::Quaterniond _r_arm_start_quaternion = rotation2quaternion( r_arm_start_rotation );
    Eigen::Quaterniond _r_arm_target_quaternion = rotation2quaternion( r_arm_target_rotation );

    double _cnt = ( double ) cnt / ( double ) all_time_steps;

    Eigen::Quaterniond _l_arm_quaternion = _l_arm_start_quaternion.slerp( _cnt , _l_arm_target_quaternion );
    Eigen::Quaterniond _r_arm_quaternion = _r_arm_start_quaternion.slerp( _cnt , _r_arm_target_quaternion );

    ik_l_arm_target_rotation = quaternion2rotation( _l_arm_quaternion );
    ik_r_arm_target_rotation = quaternion2rotation( _r_arm_quaternion );
}

}
