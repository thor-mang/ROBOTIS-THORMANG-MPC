/*
 * Link.cpp
 *
 *  Created on: Jan 11, 2016
 *      Author: sch
 */

#include "thormang3_manipulation_module/RobotisCommon.h"
#include "thormang3_manipulation_module/RobotisLink.h"
#include "thormang3_manipulation_module/RobotisData.h"
#include "thormang3_manipulation_module/Transformation.h"

namespace ROBOTIS_MANIPULATION
{

RobotisData::RobotisData() {}
RobotisData::~RobotisData() {}

RobotisData::RobotisData(TREE_SELECT tree)
{
    for ( int id = 0; id <= ALL_JOINT_ID; id++ )
        robotis_joint[ id ] = new RobotisLink();

    if ( tree == WHOLE_BODY )
    {
        robotis_joint[0]->name                  =   "base";
        robotis_joint[0]->parent                =   -1;
        robotis_joint[0]->sibling               =   -1;
        robotis_joint[0]->child                 =   38;
        robotis_joint[0]->mass                  =   0.0;
        robotis_joint[0]->relative_position     =   transitionXYZ( 0.0 , 0.0 , 0.0 );
        robotis_joint[0]->joint_axis            =   transitionXYZ( 0.0 , 0.0 , 0.0 );
        robotis_joint[0]->center_of_mass        =   transitionXYZ( 0.0 , 0.0 , 0.0 );
        robotis_joint[0]->joint_limit_max       =   100.0;
        robotis_joint[0]->joint_limit_min       =   -100.0;
        robotis_joint[0]->inertia               =   inertiaXYZ( 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 );

        /* ----- passive joint -----*/

        robotis_joint[38]->name                  =   "passive_x";
        robotis_joint[38]->parent                =   0;
        robotis_joint[38]->sibling               =   -1;
        robotis_joint[38]->child                 =   39;
        robotis_joint[38]->mass                  =   0.0;
        robotis_joint[38]->relative_position     =   transitionXYZ( 0.0 , 0.0 , 0.0 );
        robotis_joint[38]->joint_axis            =   transitionXYZ( 0.0 , 0.0 , 0.0 );
        robotis_joint[38]->center_of_mass        =   transitionXYZ( 0.0 , 0.0 , 0.0 );
        robotis_joint[38]->joint_limit_max       =   100.0;
        robotis_joint[38]->joint_limit_min       =   -100.0;
        robotis_joint[38]->inertia               =   inertiaXYZ( 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 );

        robotis_joint[39]->name                  =   "passive_y";
        robotis_joint[39]->parent                =   38;
        robotis_joint[39]->sibling               =   -1;
        robotis_joint[39]->child                 =   40;
        robotis_joint[39]->mass                  =   0.0;
        robotis_joint[39]->relative_position     =   transitionXYZ( 0.0 , 0.0 , 0.0 );
        robotis_joint[39]->joint_axis            =   transitionXYZ( 0.0 , 0.0 , 0.0 );
        robotis_joint[39]->center_of_mass        =   transitionXYZ( 0.0 , 0.0 , 0.0 );
        robotis_joint[39]->joint_limit_max       =   100.0;
        robotis_joint[39]->joint_limit_min       =   -100.0;
        robotis_joint[39]->inertia               =   inertiaXYZ( 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 );

        robotis_joint[40]->name                  =   "passive_z";
        robotis_joint[40]->parent                =   39;
        robotis_joint[40]->sibling               =   -1;
        robotis_joint[40]->child                 =   41;
        robotis_joint[40]->mass                  =   0.0;
        robotis_joint[40]->relative_position     =   transitionXYZ( 0.0 , 0.0 , 0.801 );
        robotis_joint[40]->joint_axis            =   transitionXYZ( 0.0 , 0.0 , 0.0 );
        robotis_joint[40]->center_of_mass        =   transitionXYZ( 0.0 , 0.0 , 0.0 );
        robotis_joint[40]->joint_limit_max       =   100.0;
        robotis_joint[40]->joint_limit_min       =   -100.0;
        robotis_joint[40]->inertia               =   inertiaXYZ( 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 );

        robotis_joint[41]->name                  =   "passive_roll";
        robotis_joint[41]->parent                =   40;
        robotis_joint[41]->sibling               =   -1;
        robotis_joint[41]->child                 =   42;
        robotis_joint[41]->mass                  =   0.0;
        robotis_joint[41]->relative_position     =   transitionXYZ( 0.0 , 0.0 , 0.0 );
        robotis_joint[41]->joint_axis            =   transitionXYZ( 1.0 , 0.0 , 0.0 );
        robotis_joint[41]->center_of_mass        =   transitionXYZ( 0.0 , 0.0 , 0.0 );
        robotis_joint[41]->joint_limit_max       =   100.0;
        robotis_joint[41]->joint_limit_min       =   -100.0;
        robotis_joint[41]->inertia               =   inertiaXYZ( 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 );

        robotis_joint[42]->name                  =   "passive_pitch";
        robotis_joint[42]->parent                =   41;
        robotis_joint[42]->sibling               =   -1;
        robotis_joint[42]->child                 =   43;
        robotis_joint[42]->mass                  =   0.0;
        robotis_joint[42]->relative_position     =   transitionXYZ( 0.0 , 0.0 , 0.0 );
        robotis_joint[42]->joint_axis            =   transitionXYZ( 0.0 , 1.0 , 0.0 );
        robotis_joint[42]->center_of_mass        =   transitionXYZ( 0.0 , 0.0 , 0.0 );
        robotis_joint[42]->joint_limit_max       =   100.0;
        robotis_joint[42]->joint_limit_min       =   -100.0;
        robotis_joint[42]->inertia               =   inertiaXYZ( 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 );

        robotis_joint[43]->name                  =   "passive_yaw";
        robotis_joint[43]->parent                =   42;
        robotis_joint[43]->sibling               =   -1;
        robotis_joint[43]->child                 =   44;
        robotis_joint[43]->mass                  =   0.0;
        robotis_joint[43]->relative_position     =   transitionXYZ( 0.0 , 0.0 , 0.0 );
        robotis_joint[43]->joint_axis            =   transitionXYZ( 0.0 , 0.0 , 1.0 );
        robotis_joint[43]->center_of_mass        =   transitionXYZ( 0.0 , 0.0 , 0.0 );
        robotis_joint[43]->joint_limit_max       =   100.0;
        robotis_joint[43]->joint_limit_min       =   -100.0;
        robotis_joint[43]->inertia               =   inertiaXYZ( 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 );

        /* ----- body -----*/

        // pelvis_link
        robotis_joint[44]->name                  =   "pelvis";
        robotis_joint[44]->parent                =   43;
        robotis_joint[44]->sibling               =   -1;
        robotis_joint[44]->child                 =   27;
        robotis_joint[44]->mass                  =   6.869;
        robotis_joint[44]->relative_position     =   transitionXYZ( 0.0 , 0.0 , 0.0 );
        robotis_joint[44]->joint_axis            =   transitionXYZ( 0.0 , 0.0 , 0.0 );
        robotis_joint[44]->center_of_mass        =   transitionXYZ( -0.011 , 0.000 , 0.058 );
        robotis_joint[44]->joint_limit_max       =   100.0;
        robotis_joint[44]->joint_limit_min       =   -100.0;
        robotis_joint[44]->inertia               =   inertiaXYZ( 0.03603 , 0.00000 , 0.00016 , 0.02210 , 0.00000 , 0.03830 );

        // chest_link
        robotis_joint[27]->name                  =   "torso_y";
        robotis_joint[27]->parent                =   44;
        robotis_joint[27]->sibling               =   15;
        robotis_joint[27]->child                 =   28;
        robotis_joint[27]->mass                  =   5.383;
        robotis_joint[27]->relative_position     =   transitionXYZ( 0.0 , 0.0 , 0.171 );
        robotis_joint[27]->joint_axis            =   transitionXYZ( 0.0 , 0.0 , 1.0 );
        robotis_joint[27]->center_of_mass        =   transitionXYZ( -0.007 , 0.000 , 0.109 );
        robotis_joint[27]->joint_limit_max       =   0.6 * M_PI;
        robotis_joint[27]->joint_limit_min       =   -0.6 * M_PI;
        robotis_joint[27]->inertia               =   inertiaXYZ( 0.04710 , 0.00000 , 0.00036 , 0.02554 , 0.00000 , 0.03094 );

        /* ----- head -----*/

        // head_yaw
        robotis_joint[28]->name                  =   "head_y";
        robotis_joint[28]->parent                =   27;
        robotis_joint[28]->sibling               =   1;
        robotis_joint[28]->child                 =   29;
        robotis_joint[28]->mass                  =   0.087;
        robotis_joint[28]->relative_position     =   transitionXYZ( 0.0 , 0.0 , 0.229 );
        robotis_joint[28]->joint_axis            =   transitionXYZ( 0.0 , 0.0 , 1.0 );
        robotis_joint[28]->center_of_mass        =   transitionXYZ( 0.000 , -0.002 , 0.010 );
        robotis_joint[28]->joint_limit_max       =   0.5 * M_PI;
        robotis_joint[28]->joint_limit_min       =   -0.5 * M_PI;
        robotis_joint[28]->inertia               =   inertiaXYZ( 0.00011 , 0.00000 , 0.00000 , 0.00003 , 0.00000 , 0.00012 );

        // head_pitch
        robotis_joint[29]->name                  =   "head_p";
        robotis_joint[29]->parent                =   28;
        robotis_joint[29]->sibling               =   -1;
        robotis_joint[29]->child                 =   -1;
        robotis_joint[29]->mass                  =   0.724;
        robotis_joint[29]->relative_position     =   transitionXYZ( 0.0 , -0.04500 , 0.03900 );
        robotis_joint[29]->joint_axis            =   transitionXYZ( 0.0 , 1.0 , 0.0 );
        robotis_joint[29]->center_of_mass        =   transitionXYZ( 0.009 , 0.046 , 0.022 );
        robotis_joint[29]->joint_limit_max       =   0.5 * M_PI;
        robotis_joint[29]->joint_limit_min       =   -0.5 * M_PI;
        robotis_joint[29]->inertia               =   inertiaXYZ( 0.00113 , 0.00001 , -0.00005 , 0.00114 , 0.00002 , 0.00084 );

        /*----- right arm -----*/

        // right arm shoulder pitch 1
        robotis_joint[1]->name                  =   "r_arm_sh_p1";
        robotis_joint[1]->parent                =   27;
        robotis_joint[1]->sibling               =   2;
        robotis_joint[1]->child                 =   3;
        robotis_joint[1]->mass                  =   0.194;
        robotis_joint[1]->relative_position     =   transitionXYZ( 0.000 , -0.152 , 0.160 );
        robotis_joint[1]->joint_axis            =   transitionXYZ( 0.0 , -1.0 , 0.0 );
        robotis_joint[1]->center_of_mass        =   transitionXYZ( -0.003 , -0.020 , -0.005 );
        robotis_joint[1]->joint_limit_max       =   0.5 * M_PI;
        robotis_joint[1]->joint_limit_min       =   -0.5 * M_PI;
        robotis_joint[1]->inertia               =   inertiaXYZ( 0.00018 , 0.0 , 0.0 , 0.00058 , -0.00004 , 0.00057 );

        // right arm shoulder roll
        robotis_joint[3]->name                  =   "r_arm_sh_r";
        robotis_joint[3]->parent                =   1;
        robotis_joint[3]->sibling               =   -1;
        robotis_joint[3]->child                 =   5;
        robotis_joint[3]->mass                  =   0.875;
        robotis_joint[3]->relative_position     =   transitionXYZ( 0.057 , -0.060 , -0.039 );
        robotis_joint[3]->joint_axis            =   transitionXYZ( -1.0 , 0.0 , 0.0 );
        robotis_joint[3]->center_of_mass        =   transitionXYZ( -0.060 , -0.002 , 0.000 );
        robotis_joint[3]->joint_limit_max       =   0.3 * M_PI;
        robotis_joint[3]->joint_limit_min       =   -0.5 * M_PI;
        robotis_joint[3]->inertia               =   inertiaXYZ( 0.00043 , 0.00000 , 0.00000 , 0.00112 , 0.00000 , 0.00113 );

        // right arm shoulder pitch 2
        robotis_joint[5]->name                  =   "r_arm_sh_p2";
        robotis_joint[5]->parent                =   3;
        robotis_joint[5]->sibling               =   -1;
        robotis_joint[5]->child                 =   7;
        robotis_joint[5]->mass                  =   1.122;
        robotis_joint[5]->relative_position     =   transitionXYZ( -0.057 , -0.033 , 0.000 );
        robotis_joint[5]->joint_axis            =   transitionXYZ( 0.0 , -1.0 , 0.0 );
        robotis_joint[5]->center_of_mass        =   transitionXYZ( 0.000 , -0.073 , 0.000 );
        robotis_joint[5]->joint_limit_max       =   0.5 * M_PI;
        robotis_joint[5]->joint_limit_min       =   -0.5 * M_PI;
        robotis_joint[5]->inertia               =   inertiaXYZ( 0.00277 , 0.00002 , -0.00001 , 0.00090 , 0.00004 , 0.00255 );

        // right arm elbow yaw
        robotis_joint[7]->name                  =   "r_arm_el_y";
        robotis_joint[7]->parent                =   5;
        robotis_joint[7]->sibling               =   -1;
        robotis_joint[7]->child                 =   9;
        robotis_joint[7]->mass                  =   1.357;
        robotis_joint[7]->relative_position     =   transitionXYZ( 0.03000 , -0.18700 , 0.05700 );
        robotis_joint[7]->joint_axis            =   transitionXYZ( 0.0 , 0.0 , -1.0 );
        robotis_joint[7]->center_of_mass        =   transitionXYZ( 0.042 , -0.012 , -0.058 );
        robotis_joint[7]->joint_limit_max       =   0.4 * M_PI;
        robotis_joint[7]->joint_limit_min       =   -0.4 * M_PI;
        robotis_joint[7]->inertia               =   inertiaXYZ( 0.00152 , 0.00100 , -0.00006 , 0.00560 , 0.00002 , 0.00528 );

        // right arm wrist roll
        robotis_joint[9]->name                  =   "r_arm_wr_r";
        robotis_joint[9]->parent                =   7;
        robotis_joint[9]->sibling               =   -1;
        robotis_joint[9]->child                 =   11;
        robotis_joint[9]->mass                  =   0.087;
        robotis_joint[9]->relative_position     =   transitionXYZ( 0.171 , -0.030 , -0.057 );
        robotis_joint[9]->joint_axis            =   transitionXYZ( 1.0 , 0.0 , 0.0 );
        robotis_joint[9]->center_of_mass        =   transitionXYZ( 0.010 , 0.000 , -0.002 );
        robotis_joint[9]->joint_limit_max       =   0.5 * M_PI;
        robotis_joint[9]->joint_limit_min       =   -0.5 * M_PI;
        robotis_joint[9]->inertia               =   inertiaXYZ( 0.00012 , 0.00000 , 0.00000 , 0.00011 , 0.00000 , 0.00003 );

        // right arm wrist yaw
        robotis_joint[11]->name                  =   "r_arm_wr_y";
        robotis_joint[11]->parent                =   9;
        robotis_joint[11]->sibling               =   -1;
        robotis_joint[11]->child                 =   13;
        robotis_joint[11]->mass                  =   0.768;
        robotis_joint[11]->relative_position     =   transitionXYZ( 0.039 , 0.000 , 0.045 );
        robotis_joint[11]->joint_axis            =   transitionXYZ( 0.0 , 0.0 , -1.0 );
        robotis_joint[11]->center_of_mass        =   transitionXYZ( 0.023 , -0.001 , -0.046 );
        robotis_joint[11]->joint_limit_max       =   0.45 * M_PI;
        robotis_joint[11]->joint_limit_min       =   -0.45 * M_PI;
        robotis_joint[11]->inertia               =   inertiaXYZ( 0.00059 , 0.00002 , -0.00002 , 0.00078 , 0.00000 , 0.00078 );

        // right arm wrist pitch
        robotis_joint[13]->name                  =   "r_arm_wr_p";
        robotis_joint[13]->parent                =   11;
        robotis_joint[13]->sibling               =   -1;
        robotis_joint[13]->child                 =   31;
        robotis_joint[13]->mass                  =   0.565;
        robotis_joint[13]->relative_position     =   transitionXYZ( 0.045 , 0.045 , -0.045 );
        robotis_joint[13]->joint_axis            =   transitionXYZ( 0.0 , 1.0 , 0.0 );
        robotis_joint[13]->center_of_mass        =   transitionXYZ( 0.065 , -0.045 , 0.000 );
        robotis_joint[13]->joint_limit_max       =   0.45 * M_PI;
        robotis_joint[13]->joint_limit_min       =   -0.45 * M_PI;
        robotis_joint[13]->inertia               =   inertiaXYZ( 0.00047 , 0.00001 , 0.00000 , 0.00042 , 0.00000 , 0.00058 );

        // right arm gripper
        robotis_joint[31]->name                  =   "r_arm_grip";
        robotis_joint[31]->parent                =   13;
        robotis_joint[31]->sibling               =   33;
        robotis_joint[31]->child                 =   -1;
        robotis_joint[31]->mass                  =   0.013;
        robotis_joint[31]->relative_position     =   transitionXYZ( 0.088 , -0.058 , 0.000 );
        robotis_joint[31]->joint_axis            =   transitionXYZ( 0.0 , 0.0 , 1.0 );
        robotis_joint[31]->center_of_mass        =   transitionXYZ( 0.0 , 0.0 , 0.0 );
        robotis_joint[31]->joint_limit_max       =   0.45 * M_PI;
        robotis_joint[31]->joint_limit_min       =   -0.45 * M_PI;
        robotis_joint[31]->inertia               =   inertiaXYZ( 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 );

        // right arm gripper 1
        robotis_joint[33]->name                  =   "r_arm_grip_1";
        robotis_joint[33]->parent                =   13;
        robotis_joint[33]->sibling               =   35;
        robotis_joint[33]->child                 =   -1;
        robotis_joint[33]->mass                  =   0.013;
        robotis_joint[33]->relative_position     =   transitionXYZ( 0.088 , -0.032 , 0.000 );
        robotis_joint[33]->joint_axis            =   transitionXYZ( 0.0 , 0.0 , 1.0 );
        robotis_joint[33]->center_of_mass        =   transitionXYZ( 0.0 , 0.0 , 0.0 );
        robotis_joint[33]->joint_limit_max       =   0.45 * M_PI;
        robotis_joint[33]->joint_limit_min       =   -0.45 * M_PI;
        robotis_joint[33]->inertia               =   inertiaXYZ( 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 );

        // right arm end effector
        robotis_joint[35]->name                  =   "r_arm_end";
        robotis_joint[35]->parent                =   13;
        robotis_joint[35]->sibling               =   -1;
        robotis_joint[35]->child                 =   -1;
        robotis_joint[35]->mass                  =   0.0;
        robotis_joint[35]->relative_position     =   transitionXYZ( 0.145 , -0.045 , 0.0 );
        robotis_joint[35]->joint_axis            =   transitionXYZ( 0.0 , 0.0 , 0.0 );
        robotis_joint[35]->center_of_mass        =   transitionXYZ( 0.0 , 0.0 , 0.0 );
        robotis_joint[35]->joint_limit_max       =   100.0;
        robotis_joint[35]->joint_limit_min       =   -100.0;
        robotis_joint[35]->inertia               =   inertiaXYZ( 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 );

        /*----- left arm -----*/

        // left arm shoulder pitch 1
        robotis_joint[2]->name                  =   "l_arm_sh_p1";
        robotis_joint[2]->parent                =   27;
        robotis_joint[2]->sibling               =   -1;
        robotis_joint[2]->child                 =   4;
        robotis_joint[2]->mass                  =   0.194;
        robotis_joint[2]->relative_position     =   transitionXYZ( 0.000 , 0.152 , 0.160 );
        robotis_joint[2]->joint_axis            =   transitionXYZ( 0.0 , 1.0 , 0.0 );
        robotis_joint[2]->center_of_mass        =   transitionXYZ( -0.003 , 0.020 , -0.005 );
        robotis_joint[2]->joint_limit_max       =   0.5 * M_PI;
        robotis_joint[2]->joint_limit_min       =   -0.5 * M_PI;
        robotis_joint[2]->inertia               =   inertiaXYZ( 0.00018 , 0.00000 , 0.00000 , 0.00058 , 0.00004 , 0.00057 );

        // left arm shoulder roll
        robotis_joint[4]->name                  =   "l_arm_sh_r";
        robotis_joint[4]->parent                =   2;
        robotis_joint[4]->sibling               =   -1;
        robotis_joint[4]->child                 =   6;
        robotis_joint[4]->mass                  =   0.875;
        robotis_joint[4]->relative_position     =   transitionXYZ( 0.057 , 0.060 , -0.039 );
        robotis_joint[4]->joint_axis            =   transitionXYZ( -1.0 , 0.0 , 0.0 );
        robotis_joint[4]->center_of_mass        =   transitionXYZ( -0.060 , 0.002 , 0.000 );
        robotis_joint[4]->joint_limit_max       =   0.5 * M_PI;
        robotis_joint[4]->joint_limit_min       =   -0.3 * M_PI;
        robotis_joint[4]->inertia               =   inertiaXYZ( 0.00043 , 0.00000 , 0.00000 , 0.00112 , 0.00000 , 0.00113 );

        // left arm shoulder pitch 2
        robotis_joint[6]->name                  =   "l_arm_sh_p2";
        robotis_joint[6]->parent                =   4;
        robotis_joint[6]->sibling               =   -1;
        robotis_joint[6]->child                 =   8;
        robotis_joint[6]->mass                  =   1.122;
        robotis_joint[6]->relative_position     =   transitionXYZ( -0.057 , 0.033 , 0.000 );
        robotis_joint[6]->joint_axis            =   transitionXYZ( 0.0 , 1.0 , 0.0 );
        robotis_joint[6]->center_of_mass        =   transitionXYZ( 0.000 , 0.073 , 0.000 );
        robotis_joint[6]->joint_limit_max       =   0.5 * M_PI;
        robotis_joint[6]->joint_limit_min       =   -0.5 * M_PI;
        robotis_joint[6]->inertia               =   inertiaXYZ( 0.00277 , -0.00002 , -0.00001 , 0.00090 , -0.00004 , 0.00255 );

        // left arm elbow yaw
        robotis_joint[8]->name                  =   "l_arm_el_y";
        robotis_joint[8]->parent                =   6;
        robotis_joint[8]->sibling               =   -1;
        robotis_joint[8]->child                 =   10;
        robotis_joint[8]->mass                  =   1.357;
        robotis_joint[8]->relative_position     =   transitionXYZ( 0.030 , 0.187 , 0.057 );
        robotis_joint[8]->joint_axis            =   transitionXYZ( 0.0 , 0.0 , -1.0 );
        robotis_joint[8]->center_of_mass        =   transitionXYZ( 0.042 , 0.012 , -0.058 );
        robotis_joint[8]->joint_limit_max       =   0.4 * M_PI;
        robotis_joint[8]->joint_limit_min       =   -0.4 * M_PI;
        robotis_joint[8]->inertia               =   inertiaXYZ( 0.00152 , -0.00100 , -0.00006 , 0.00560 , -0.00002 , 0.00528 );

        // left arm wrist roll
        robotis_joint[10]->name                  =   "l_arm_wr_r";
        robotis_joint[10]->parent                =   8;
        robotis_joint[10]->sibling               =   -1;
        robotis_joint[10]->child                 =   12;
        robotis_joint[10]->mass                  =   0.087;
        robotis_joint[10]->relative_position     =   transitionXYZ( 0.171 , 0.030 , -0.057 );
        robotis_joint[10]->joint_axis            =   transitionXYZ( 1.0 , 0.0 , 0.0 );
        robotis_joint[10]->center_of_mass        =   transitionXYZ( 0.010 , 0.000 , 0.002 );
        robotis_joint[10]->joint_limit_max       =   0.5 * M_PI;
        robotis_joint[10]->joint_limit_min       =   -0.5 * M_PI;
        robotis_joint[10]->inertia               =   inertiaXYZ( 0.00012 , 0.00000 , 0.00000 , 0.00011 , 0.00000 , 0.00003 );

        // left arm wrist yaw
        robotis_joint[12]->name                  =   "l_arm_wr_y";
        robotis_joint[12]->parent                =   10;
        robotis_joint[12]->sibling               =   -1;
        robotis_joint[12]->child                 =   14;
        robotis_joint[12]->mass                  =   0.768;
        robotis_joint[12]->relative_position     =   transitionXYZ( 0.039 , 0.000 , 0.045 );
        robotis_joint[12]->joint_axis            =   transitionXYZ( 0.0 , 0.0 , -1.0 );
        robotis_joint[12]->center_of_mass        =   transitionXYZ( 0.023 , 0.001 , -0.046 );
        robotis_joint[12]->joint_limit_max       =   0.45 * M_PI;
        robotis_joint[12]->joint_limit_min       =   -0.45 * M_PI;
        robotis_joint[12]->inertia               =   inertiaXYZ( 0.00059 , -0.00002 , -0.00002 , 0.00078 , 0.00000 , 0.00078 );

        // left arm wrist pitch
        robotis_joint[14]->name                  =   "l_arm_wr_p";
        robotis_joint[14]->parent                =   12;
        robotis_joint[14]->sibling               =   -1;
        robotis_joint[14]->child                 =   30;
        robotis_joint[14]->mass                  =   0.08709;
        robotis_joint[14]->relative_position     =   transitionXYZ( 0.045 , -0.045 , -0.045 );
        robotis_joint[14]->joint_axis            =   transitionXYZ( 0.0 , -1.0 , 0.0 );
        robotis_joint[14]->center_of_mass        =   transitionXYZ( 0.065 , 0.045 , 0.000 );
        robotis_joint[14]->joint_limit_max       =   0.45 * M_PI;
        robotis_joint[14]->joint_limit_min       =   -0.45 * M_PI;
        robotis_joint[14]->inertia               =   inertiaXYZ( 0.00047 , -0.00001 , 0.00000 , 0.00042 , 0.00000 , 0.00058 );

        // left arm gripper
        robotis_joint[30]->name                  =   "l_arm_grip";
        robotis_joint[30]->parent                =   14;
        robotis_joint[30]->sibling               =   32;
        robotis_joint[30]->child                 =   -1;
        robotis_joint[30]->mass                  =   0.013;
        robotis_joint[30]->relative_position     =   transitionXYZ( 0.088 , 0.058 , 0.000 );
        robotis_joint[30]->joint_axis            =   transitionXYZ( 0.0 , 0.0 , 1.0 );
        robotis_joint[30]->center_of_mass        =   transitionXYZ( 0.0 , 0.0 , 0.0 );
        robotis_joint[30]->joint_limit_max       =   0.45 * M_PI;
        robotis_joint[30]->joint_limit_min       =   -0.45 * M_PI;
        robotis_joint[30]->inertia               =   inertiaXYZ( 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 );

        // left arm gripper_1
        robotis_joint[32]->name                  =   "l_arm_grip_1";
        robotis_joint[32]->parent                =   14;
        robotis_joint[32]->sibling               =   34;
        robotis_joint[32]->child                 =   -1;
        robotis_joint[32]->mass                  =   0.013;
        robotis_joint[32]->relative_position     =   transitionXYZ( 0.088 , 0.032 , 0.000 );
        robotis_joint[32]->joint_axis            =   transitionXYZ( 0.0 , 0.0 , 1.0 );
        robotis_joint[32]->center_of_mass        =   transitionXYZ( 0.0 , 0.0 , 0.0 );
        robotis_joint[32]->joint_limit_max       =   0.45 * M_PI;
        robotis_joint[32]->joint_limit_min       =   -0.45 * M_PI;
        robotis_joint[32]->inertia               =   inertiaXYZ( 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 );

        // left arm end effector
        robotis_joint[34]->name                  =   "l_arm_end";
        robotis_joint[34]->parent                =   14;
        robotis_joint[34]->sibling               =   -1;
        robotis_joint[34]->child                 =   -1;
        robotis_joint[34]->mass                  =   0.0;
        robotis_joint[34]->relative_position     =   transitionXYZ( 0.145 , 0.045 , 0.0 );
        robotis_joint[34]->joint_axis            =   transitionXYZ( 0.0 , 0.0 , 0.0 );
        robotis_joint[34]->center_of_mass        =   transitionXYZ( 0.0 , 0.0 , 0.0 );
        robotis_joint[34]->joint_limit_max       =   100.0;
        robotis_joint[34]->joint_limit_min       =   -100.0;
        robotis_joint[34]->inertia               =   inertiaXYZ( 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 );

        /* ----- right leg -----*/

        // right leg hip yaw
        robotis_joint[15]->name                  =   "r_leg_hip_y";
        robotis_joint[15]->parent                =   44;
        robotis_joint[15]->sibling               =   16;
        robotis_joint[15]->child                 =   17;
        robotis_joint[15]->mass                  =   0.243;
        robotis_joint[15]->relative_position     =   transitionXYZ( 0.000 , -0.093 , -0.018 );
        robotis_joint[15]->joint_axis            =   transitionXYZ( 0.0 , 0.0 , -1.0 );
        robotis_joint[15]->center_of_mass        =   transitionXYZ( -0.012 , 0.000 , -0.025 );
        robotis_joint[15]->joint_limit_max       =   0.45 * M_PI;
        robotis_joint[15]->joint_limit_min       =   -0.45 * M_PI;
        robotis_joint[15]->inertia               =   inertiaXYZ( 0.00024 , 0.00000 , 0.00000 , 0.00101 , 0.00000 , 0.00092 );

        // right leg hip roll
        robotis_joint[17]->name                  =   "r_leg_hip_r";
        robotis_joint[17]->parent                =   15;
        robotis_joint[17]->sibling               =   -1;
        robotis_joint[17]->child                 =   19;
        robotis_joint[17]->mass                  =   1.045;
        robotis_joint[17]->relative_position     =   transitionXYZ( 0.057 , 0.000 , -0.075 );
        robotis_joint[17]->joint_axis            =   transitionXYZ( 1.0 , 0.0 , 0.0 );
        robotis_joint[17]->center_of_mass        =   transitionXYZ( -0.068 , 0.000 , 0.000 );
        robotis_joint[17]->joint_limit_max       =   0.3 * M_PI;
        robotis_joint[17]->joint_limit_min       =   -0.3 * M_PI;
        robotis_joint[17]->inertia               =   inertiaXYZ( 0.00056 , 0.00000 , 0.00000 , 0.00168 , 0.00000 , 0.00171 );

        // right leg hip pitch
        robotis_joint[19]->name                  =   "r_leg_hip_p";
        robotis_joint[19]->parent                =   17;
        robotis_joint[19]->sibling               =   -1;
        robotis_joint[19]->child                 =   21;
        robotis_joint[19]->mass                  =   3.095;
        robotis_joint[19]->relative_position     =   transitionXYZ( -0.057 , 0.033 , 0.000 );
        robotis_joint[19]->joint_axis            =   transitionXYZ( 0.0 , -1.0 , 0.0 );
        robotis_joint[19]->center_of_mass        =   transitionXYZ( 0.022 , 0.007 , -0.168 );
        robotis_joint[19]->joint_limit_max       =   0.4 * M_PI;
        robotis_joint[19]->joint_limit_min       =   -0.4 * M_PI;
        robotis_joint[19]->inertia               =   inertiaXYZ( 0.04329 , -0.00027 , 0.00286 , 0.04042 , 0.00203 , 0.00560 );

        // right leg knee pitch
        robotis_joint[21]->name                  =   "r_leg_kn_p";
        robotis_joint[21]->parent                =   19;
        robotis_joint[21]->sibling               =   -1;
        robotis_joint[21]->child                 =   23;
        robotis_joint[21]->mass                  =   2.401;
        robotis_joint[21]->relative_position     =   transitionXYZ( 0.000 , -0.060 , -0.300 );
        robotis_joint[21]->joint_axis            =   transitionXYZ( 0.0 , -1.0 , 0.0 );
        robotis_joint[21]->center_of_mass        =   transitionXYZ( -0.002 , 0.066 , -0.183 );
        robotis_joint[21]->joint_limit_max       =   0.1 * M_PI;
        robotis_joint[21]->joint_limit_min       =   -0.7 * M_PI;
        robotis_joint[21]->inertia               =   inertiaXYZ( 0.01971 , -0.00031 , -0.00294 , 0.01687 , -0.00140 , 0.00574 );

        // right leg ankle pitch
        robotis_joint[23]->name                  =   "r_leg_an_p";
        robotis_joint[23]->parent                =   21;
        robotis_joint[23]->sibling               =   -1;
        robotis_joint[23]->child                 =   25;
        robotis_joint[23]->mass                  =   1.045;
        robotis_joint[23]->relative_position     =   transitionXYZ( 0.000 , 0.060 , -0.300 );
        robotis_joint[23]->joint_axis            =   transitionXYZ( 0.0 , 1.0 , 0.0 );
        robotis_joint[23]->center_of_mass        =   transitionXYZ( -0.011 , 0.033 , 0.000 );
        robotis_joint[23]->joint_limit_max       =   0.45 * M_PI;
        robotis_joint[23]->joint_limit_min       =   -0.45 * M_PI;
        robotis_joint[23]->inertia               =   inertiaXYZ( 0.00056 , 0.00000 , 0.00000 , 0.00168 , 0.00000 , 0.00171 );

        // right leg ankle roll
        robotis_joint[25]->name                  =   "r_leg_an_r";
        robotis_joint[25]->parent                =   23;
        robotis_joint[25]->sibling               =   -1;
        robotis_joint[25]->child                 =   37;
        robotis_joint[25]->mass                  =   0.223;
        robotis_joint[25]->relative_position     =   transitionXYZ( 0.057 , 0.033 , 0.000 );
        robotis_joint[25]->joint_axis            =   transitionXYZ( 1.0 , 0.0 , 0.0 );
        robotis_joint[25]->center_of_mass        =   transitionXYZ( -0.070 , 0.000 , -0.048 );
        robotis_joint[25]->joint_limit_max       =   0.45 * M_PI;
        robotis_joint[25]->joint_limit_min       =   -0.45 * M_PI;
        robotis_joint[25]->inertia               =   inertiaXYZ( 0.00022 , 0.00000 , -0.00001 , 0.00099 , 0.00000 , 0.00091 );

        // right leg ft
        robotis_joint[37]->name                  =   "r_leg_ft";
        robotis_joint[37]->parent                =   25;
        robotis_joint[37]->sibling               =   -1;
        robotis_joint[37]->child                 =   45;
        robotis_joint[37]->mass                  =   1.689;
        robotis_joint[37]->relative_position     =   transitionXYZ( -0.057 , 0.000 , -0.087 );
        robotis_joint[37]->joint_axis            =   transitionXYZ( 0.0 , 0.0 , 0.0 );
        robotis_joint[37]->center_of_mass        =   transitionXYZ( 0.000 , -0.009 , -0.013 );
        robotis_joint[37]->joint_limit_max       =   100.0;
        robotis_joint[37]->joint_limit_min       =   -100.0;
        robotis_joint[37]->inertia               =   inertiaXYZ( 0.00219 , 0.00000 , 0.00000 , 0.00433 , -0.00011 , 0.00609 );

        // right leg end
        robotis_joint[45]->name                  =   "r_leg_end";
        robotis_joint[45]->parent                =   37;
        robotis_joint[45]->sibling               =   -1;
        robotis_joint[45]->child                 =   -1;
        robotis_joint[45]->mass                  =   0.0;
        robotis_joint[45]->relative_position     =   transitionXYZ( 0.0 , 0.0 , -0.021 );
        robotis_joint[45]->joint_axis            =   transitionXYZ( 0.0 , 0.0 , 0.0 );
        robotis_joint[45]->center_of_mass        =   transitionXYZ( 0.0 , 0.0 , 0.0 );
        robotis_joint[45]->joint_limit_max       =   100.0;
        robotis_joint[45]->joint_limit_min       =   -100.0;
        robotis_joint[45]->inertia               =   inertiaXYZ( 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 );

        /* ----- left leg -----*/

        // left leg hip yaw
        robotis_joint[16]->name                  =   "l_leg_hip_y";
        robotis_joint[16]->parent                =   44;
        robotis_joint[16]->sibling               =   -1;
        robotis_joint[16]->child                 =   18;
        robotis_joint[16]->mass                  =   0.243;
        robotis_joint[16]->relative_position     =   transitionXYZ( 0.000 , 0.093 , -0.018 );
        robotis_joint[16]->joint_axis            =   transitionXYZ( 0.0 , 0.0 , -1.0 );
        robotis_joint[16]->center_of_mass        =   transitionXYZ( 0.012 , 0.000 , -0.025 );
        robotis_joint[16]->joint_limit_max       =   0.45 * M_PI;
        robotis_joint[16]->joint_limit_min       =   -0.45 * M_PI;
        robotis_joint[16]->inertia               =   inertiaXYZ( 0.00024 , 0.00000 , 0.00000 , 0.00101 , 0.00000 , 0.00092 );

        // left leg hip roll
        robotis_joint[18]->name                  =   "l_leg_hip_r";
        robotis_joint[18]->parent                =   16;
        robotis_joint[18]->sibling               =   -1;
        robotis_joint[18]->child                 =   20;
        robotis_joint[18]->mass                  =   1.045;
        robotis_joint[18]->relative_position     =   transitionXYZ( 0.057 , 0.000 , -0.075 );
        robotis_joint[18]->joint_axis            =   transitionXYZ( 1.0 , 0.0 , 0.0 );
        robotis_joint[18]->center_of_mass        =   transitionXYZ( -0.068 , 0.000 , 0.000 );
        robotis_joint[18]->joint_limit_max       =   0.3 * M_PI;
        robotis_joint[18]->joint_limit_min       =   -0.3 * M_PI;
        robotis_joint[18]->inertia               =   inertiaXYZ( 0.00056 , 0.00000 , 0.00000 , 0.00168 , 0.00000 , 0.00171 );

        // left leg hip pitch
        robotis_joint[20]->name                  =   "l_leg_hip_p";
        robotis_joint[20]->parent                =   18;
        robotis_joint[20]->sibling               =   -1;
        robotis_joint[20]->child                 =   22;
        robotis_joint[20]->mass                  =   3.095;
        robotis_joint[20]->relative_position     =   transitionXYZ( -0.057 , 0.033 , 0.000 );
        robotis_joint[20]->joint_axis            =   transitionXYZ( 0.0 , 1.0 , 0.0 );
        robotis_joint[20]->center_of_mass        =   transitionXYZ( 0.022 , -0.007 , -0.168 );
        robotis_joint[20]->joint_limit_max       =   0.4 * M_PI;
        robotis_joint[20]->joint_limit_min       =   -0.4 * M_PI;
        robotis_joint[20]->inertia               =   inertiaXYZ( 0.04328 , 0.00028 , 0.00288 , 0.04042 , -0.00202 , 0.00560 );

        // left leg knee pitch
        robotis_joint[22]->name                  =   "l_leg_kn_p";
        robotis_joint[22]->parent                =   20;
        robotis_joint[22]->sibling               =   -1;
        robotis_joint[22]->child                 =   24;
        robotis_joint[22]->mass                  =   2.401;
        robotis_joint[22]->relative_position     =   transitionXYZ( 0.000 , 0.060 , -0.300 );
        robotis_joint[22]->joint_axis            =   transitionXYZ( 0.0 , 1.0 , 0.0 );
        robotis_joint[22]->center_of_mass        =   transitionXYZ( -0.002 , -0.066 , -0.183 );
        robotis_joint[22]->joint_limit_max       =   0.7 * M_PI;
        robotis_joint[22]->joint_limit_min       =   -0.1 * M_PI;
        robotis_joint[22]->inertia               =   inertiaXYZ( 0.01971 , 0.00031 , -0.00294 , 0.01687 , 0.00140 , 0.00574 );

        // left leg ankle pitch
        robotis_joint[24]->name                  =   "l_leg_an_p";
        robotis_joint[24]->parent                =   22;
        robotis_joint[24]->sibling               =   -1;
        robotis_joint[24]->child                 =   26;
        robotis_joint[24]->mass                  =   1.045;
        robotis_joint[24]->relative_position     =   transitionXYZ( 0.000 , -0.060 , -0.300 );
        robotis_joint[24]->joint_axis            =   transitionXYZ( 0.0 , -1.0 , 0.0 );
        robotis_joint[24]->center_of_mass        =   transitionXYZ( -0.011 , -0.033 , 0.000 );
        robotis_joint[24]->joint_limit_max       =   0.45 * M_PI;
        robotis_joint[24]->joint_limit_min       =   -0.45 * M_PI;
        robotis_joint[24]->inertia               =   inertiaXYZ( 0.00056 , 0.00000 , 0.00000 , 0.00168 , 0.00000 , 0.00171 );

        // left leg ankle pitch
        robotis_joint[26]->name                  =   "l_leg_an_r";
        robotis_joint[26]->parent                =   24;
        robotis_joint[26]->sibling               =   -1;
        robotis_joint[26]->child                 =   36;
        robotis_joint[26]->mass                  =   0.223;
        robotis_joint[26]->relative_position     =   transitionXYZ( 0.057 , -0.033 , 0.000 );
        robotis_joint[26]->joint_axis            =   transitionXYZ( 1.0 , 0.0 , 0.0 );
        robotis_joint[26]->center_of_mass        =   transitionXYZ( -0.070 , 0.000 , -0.048 );
        robotis_joint[26]->joint_limit_max       =   0.45 * M_PI;
        robotis_joint[26]->joint_limit_min       =   -0.45 * M_PI;
        robotis_joint[26]->inertia               =   inertiaXYZ( 0.00022 , 0.00000 , -0.00001 , 0.00099 , 0.00000 , 0.00091 );

        // left leg ft
        robotis_joint[36]->name                  =   "r_leg_ft";
        robotis_joint[36]->parent                =   26;
        robotis_joint[36]->sibling               =   -1;
        robotis_joint[36]->child                 =   46;
        robotis_joint[36]->mass                  =   0.0;
        robotis_joint[36]->relative_position     =   transitionXYZ( -0.057 , 0.000 , -0.087 );
        robotis_joint[36]->joint_axis            =   transitionXYZ( 0.0 , 0.0 , 0.0 );
        robotis_joint[36]->center_of_mass        =   transitionXYZ( 0.000 , 0.009 , -0.013 );
        robotis_joint[36]->joint_limit_max       =   100.0;
        robotis_joint[36]->joint_limit_min       =   -100.0;
        robotis_joint[36]->inertia               =   inertiaXYZ( 0.00219 , 0.00000 , 0.00000 , 0.00433 , 0.00011 , 0.00609 );

        // left leg end
        robotis_joint[46]->name                  =   "r_leg_end";
        robotis_joint[46]->parent                =   36;
        robotis_joint[46]->sibling               =   -1;
        robotis_joint[46]->child                 =   -1;
        robotis_joint[46]->mass                  =   0.0;
        robotis_joint[46]->relative_position     =   transitionXYZ( 0.0 , 0.0 , -0.021 );
        robotis_joint[46]->joint_axis            =   transitionXYZ( 0.0 , 0.0 , 0.0 );
        robotis_joint[46]->center_of_mass        =   transitionXYZ( 0.0 , 0.0 , 0.0 );
        robotis_joint[46]->joint_limit_max       =   100.0;
        robotis_joint[46]->joint_limit_min       =   -100.0;
        robotis_joint[46]->inertia               =   inertiaXYZ( 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 );
    }
}

std::vector<int> RobotisData::findRoute( int to )
{
    int _id = robotis_joint[ to ]->parent;

    std::vector<int> _idx;

    if( _id == 0 )
    {
        _idx.push_back(0);
        _idx.push_back( to );
    }
    else
    {
        _idx = findRoute( _id );
        _idx.push_back( to );
    }

    return _idx;
}

std::vector<int> RobotisData::findRoute( int from , int to )
{
    int _id = robotis_joint[ to ]->parent;

    std::vector<int> _idx;

    if( _id == from )
    {
        _idx.push_back( from );
        _idx.push_back( to );
    }
    else if ( _id != 0 )
    {
        _idx = findRoute( from , _id );
        _idx.push_back( to );
    }

    return _idx;
}

double RobotisData::totalMass( int joint_ID )
{
    double _mass;

    if ( joint_ID == -1 )
        _mass = 0.0;
    else
        _mass = robotis_joint[ joint_ID ]->mass + totalMass( robotis_joint[ joint_ID ]->sibling ) + totalMass( robotis_joint[ joint_ID ]->child );

    return _mass;
}

Eigen::MatrixXd RobotisData::calcMC( int joint_ID )
{
    Eigen::MatrixXd _mc(3,1);

    if ( joint_ID == -1 )
        _mc = Eigen::MatrixXd::Zero(3,1);
    else
    {
        _mc = robotis_joint[ joint_ID ]->mass * ( robotis_joint[ joint_ID ]->orientation * robotis_joint[ joint_ID ]->center_of_mass + robotis_joint[ joint_ID ]->position );
        _mc = _mc + calcMC( robotis_joint[ joint_ID ]->sibling ) + calcMC( robotis_joint[ joint_ID ]->child );
    }

    return _mc;
}

Eigen::MatrixXd RobotisData::calcCOM( Eigen::MatrixXd MC )
{
    double _mass ;
    Eigen::MatrixXd _COM( 3 , 1 );

    _mass = totalMass( 0 );

    _COM = MC / _mass;

    return _COM;
}

void RobotisData::forwardKinematics( int joint_ID )
{
    if ( joint_ID == -1 )
        return;

//    robotis_joint[0]->position.block( 0 , 0 , 3 , 1 )       = Eigen::MatrixXd::Zero(3,1);
//    robotis_joint[0]->orientation.block( 0 , 0 , 3 , 3 )    = Rodrigues( hatto( robotis_joint[0]->joint_axis ), robotis_joint[ 0 ]->joint_angle );

//    robotis_joint[0]->position.block<3, 1>( 0 , 0)       = Eigen::MatrixXd::Zero(3,1);
//    robotis_joint[0]->orientation.block<3, 3>( 0 , 0)    = Rodrigues( hatto( robotis_joint[0]->joint_axis ), robotis_joint[ 0 ]->joint_angle );

    robotis_joint[0]->position       = Eigen::MatrixXd::Zero(3,1);
    robotis_joint[0]->orientation    = Rodrigues( hatto( robotis_joint[0]->joint_axis ), robotis_joint[ 0 ]->joint_angle );

    if ( joint_ID != 0 )
    {
        int _parent = robotis_joint[ joint_ID ]->parent;

//        robotis_joint[ joint_ID ]->position.block( 0 , 0 , 3 , 1 ) =
//                robotis_joint[ _parent ]->orientation * robotis_joint[ joint_ID ]->relative_position + robotis_joint[ _parent ]->position;
//        robotis_joint[ joint_ID ]->orientation.block( 0 , 0 , 3 , 3 ) =
//                robotis_joint[ _parent ]->orientation * Rodrigues( hatto( robotis_joint[ joint_ID ]->joint_axis ), robotis_joint[ joint_ID ]->joint_angle );
//
//        robotis_joint[ joint_ID ]->transformation.block ( 0 , 3 , 3 , 1 ) = robotis_joint[ joint_ID ]->position;
//        robotis_joint[ joint_ID ]->transformation.block ( 0 , 0 , 3 , 3 ) = robotis_joint[ joint_ID ]->orientation;

//        robotis_joint[ joint_ID ]->position.block<3, 1>( 0 , 0 ) =
//                robotis_joint[ _parent ]->orientation * robotis_joint[ joint_ID ]->relative_position + robotis_joint[ _parent ]->position;
//        robotis_joint[ joint_ID ]->orientation.block<3, 3>( 0 , 0 ) =
//                robotis_joint[ _parent ]->orientation * Rodrigues( hatto( robotis_joint[ joint_ID ]->joint_axis ), robotis_joint[ joint_ID ]->joint_angle );
//
//        robotis_joint[ joint_ID ]->transformation.block<3, 1> ( 0 , 3) = robotis_joint[ joint_ID ]->position;
//        robotis_joint[ joint_ID ]->transformation.block<3, 3> ( 0 , 0) = robotis_joint[ joint_ID ]->orientation;

        robotis_joint[ joint_ID ]->position =
                robotis_joint[ _parent ]->orientation * robotis_joint[ joint_ID ]->relative_position + robotis_joint[ _parent ]->position;
        robotis_joint[ joint_ID ]->orientation =
                robotis_joint[ _parent ]->orientation * Rodrigues( hatto( robotis_joint[ joint_ID ]->joint_axis ), robotis_joint[ joint_ID ]->joint_angle );

        robotis_joint[ joint_ID ]->transformation.block<3, 1> ( 0 , 3) = robotis_joint[ joint_ID ]->position;
        robotis_joint[ joint_ID ]->transformation.block<3, 3> ( 0 , 0) = robotis_joint[ joint_ID ]->orientation;
    }

    forwardKinematics( robotis_joint[ joint_ID ]->sibling );
    forwardKinematics( robotis_joint[ joint_ID ]->child );
}

Eigen::MatrixXd RobotisData::calcJacobian( std::vector<int> idx )
{
    int _idx_size 	= 	idx.size();
    int _end 		= 	_idx_size - 1;

    Eigen::MatrixXd _tar_position    = 	robotis_joint[ idx[ _end ] ]->position;
    Eigen::MatrixXd _Jacobian        = 	Eigen::MatrixXd::Zero( 6 , _idx_size );

    for ( int id = 0; id < _idx_size; id++)
    {
        int _id 					= 	idx[ id ];

        Eigen::MatrixXd _tar_orientation      = 	robotis_joint[ _id ]->orientation * robotis_joint[ _id ]->joint_axis;

        _Jacobian.block( 0 , id , 3 , 1 ) 	= 	cross( _tar_orientation , _tar_position - robotis_joint[ _id ]->position );
        _Jacobian.block( 3 , id , 3 , 1 ) 	= 	_tar_orientation;
    }

    return _Jacobian;
}

Eigen::MatrixXd RobotisData::calcJacobianCOM(std::vector<int> idx)
{
    int _idx_size	=	idx.size();
    int _end 		=	_idx_size - 1;

    Eigen::MatrixXd _target_position    = 	robotis_joint[ idx[ _end ] ]->position;
    Eigen::MatrixXd _jacobianCOM        = 	Eigen::MatrixXd::Zero( 6 , _idx_size );

    for ( int id = 0; id < _idx_size; id++ )
    {
        int _id 			= 	idx[ id ];
        double _mass 	= 	totalMass( _id );

        Eigen::MatrixXd _og                     =	calcMC( _id ) / _mass - robotis_joint[ _id ]->position;
        Eigen::MatrixXd _target_orientation 	= 	robotis_joint[ _id ]->orientation * robotis_joint[ _id ]->joint_axis;

        _jacobianCOM.block( 0 , id , 3 , 1 ) 	= 	cross( _target_orientation , _og );
        _jacobianCOM.block( 3 , id , 3 , 1 ) 	= 	_target_orientation;
    }

    return _jacobianCOM;
}

Eigen::MatrixXd RobotisData::calcVWerr( Eigen::MatrixXd tar_position, Eigen::MatrixXd curr_position, Eigen::MatrixXd tar_orientation, Eigen::MatrixXd curr_orientation )
{
    Eigen::MatrixXd _pos_err        =	tar_position - curr_position;
    Eigen::MatrixXd _ori_err        =	curr_orientation.inverse() * tar_orientation;
    Eigen::MatrixXd __ori_err      =	curr_orientation * rot2omega( _ori_err );

    Eigen::MatrixXd _err 	= 	Eigen::MatrixXd::Zero( 6 , 1 );
    _err.block(0,0,3,1) 		= 	_pos_err;
    _err.block(3,0,3,1) 		= 	__ori_err;

    return _err;
}

bool RobotisData::inverseKinematics( int to, Eigen::MatrixXd tar_position, Eigen::MatrixXd tar_orientation, int max_iter, double ik_err )
{
    bool ik_success = false;
    bool limit_success = false;

    forwardKinematics(0);

    std::vector<int> _idx   = 	findRoute( to );

    for ( int iter = 0; iter < max_iter; iter++ )
    {
        Eigen::MatrixXd _Jacobian		= 	calcJacobian( _idx );

        Eigen::MatrixXd _curr_position      = 	robotis_joint[ to ]->position;
        Eigen::MatrixXd _curr_orientation 	= 	robotis_joint[ to ]->orientation;

        Eigen::MatrixXd _err 	= 	calcVWerr( tar_position,_curr_position, tar_orientation, _curr_orientation );

        if ( _err.norm() < ik_err )
        {
            ik_success = true;
            break;
        }
        else
            ik_success = false;


        Eigen::MatrixXd __Jacobian      = 	_Jacobian * _Jacobian.transpose();
        Eigen::MatrixXd ___Jacobian     = 	_Jacobian.transpose() * __Jacobian.inverse();

        Eigen::MatrixXd _delta_angle 		= 	___Jacobian * _err ;

        for ( int id = 0; id < _idx.size(); id++ )
        {
            int _joint_num      = 	_idx[ id ];
            robotis_joint[ _joint_num ]->joint_angle    += 	_delta_angle.coeff( id );
        }

        forwardKinematics(0);
    }

    for ( int id = 0; id < _idx.size(); id++ )
    {
        int _joint_num      = 	_idx[ id ];

        if ( robotis_joint[ _joint_num ]->joint_angle >= robotis_joint[ _joint_num ]->joint_limit_max )
        {
            limit_success = false;
            break;
        }
        else if ( robotis_joint[ _joint_num ]->joint_angle <= robotis_joint[ _joint_num ]->joint_limit_min )
        {
            limit_success = false;
            break;
        }
        else
            limit_success = true;
    }

    if ( ik_success == true && limit_success == true )
        return true;
    else
        return false;
}

bool RobotisData::inverseKinematics( int from, int to, Eigen::MatrixXd tar_position, Eigen::MatrixXd tar_orientation, int max_iter, double ik_err )
{
    bool ik_success = false;
    bool limit_success = false;

    forwardKinematics(0);

    std::vector<int> _idx   = 	findRoute( from , to );

    for ( int iter = 0; iter < max_iter; iter++ )
    {
        Eigen::MatrixXd _Jacobian		= 	calcJacobian( _idx );

        Eigen::MatrixXd _curr_position      = 	robotis_joint[ to ]->position;
        Eigen::MatrixXd _curr_orientation 	= 	robotis_joint[ to ]->orientation;

        Eigen::MatrixXd _err 	= 	calcVWerr( tar_position,_curr_position, tar_orientation, _curr_orientation );

        if ( _err.norm() < ik_err )
        {
            ik_success = true;
            break;
        }
        else
            ik_success = false;


        Eigen::MatrixXd __Jacobian      = 	_Jacobian * _Jacobian.transpose();
        Eigen::MatrixXd ___Jacobian     = 	_Jacobian.transpose() * __Jacobian.inverse();

        Eigen::MatrixXd _delta_angle 		= 	___Jacobian * _err ;

        for ( int id = 0; id < _idx.size(); id++ )
        {
            int _joint_num      = 	_idx[ id ];
            robotis_joint[ _joint_num ]->joint_angle    += 	_delta_angle.coeff( id );
        }

        forwardKinematics(0);
    }

    for ( int id = 0; id < _idx.size(); id++ )
    {
        int _joint_num      = 	_idx[ id ];

        if ( robotis_joint[ _joint_num ]->joint_angle >= robotis_joint[ _joint_num ]->joint_limit_max )
        {
            limit_success = false;
            break;
        }
        else if ( robotis_joint[ _joint_num ]->joint_angle <= robotis_joint[ _joint_num ]->joint_limit_min )
        {
            limit_success = false;
            break;
        }
        else
            limit_success = true;
    }

    if ( ik_success == true && limit_success == true )
        return true;
    else
        return false;
}

bool RobotisData::inverseKinematics( int to, Eigen::MatrixXd tar_position, Eigen::MatrixXd tar_orientation, int max_iter, double ik_err , Eigen::MatrixXd weight )
{
    bool ik_success = false;
    bool limit_success = false;

    forwardKinematics(0);

    std::vector<int> _idx   = 	findRoute( to );

    /* weight */
    Eigen::MatrixXd _weight = Eigen::MatrixXd::Identity( _idx.size() , _idx.size() );

    for ( int _ix = 0; _ix < _idx.size(); _ix++ )
        _weight.coeffRef( _ix , _ix ) = weight.coeff( _idx[ _ix ], 0 );

    /* damping */
    Eigen::MatrixXd _eval 		= 	Eigen::MatrixXd::Zero( 6 , 6 );

    double p_damping 		= 	1e-5;
    double R_damping 		= 	1e-5;

    for ( int _ix = 0; _ix < 3; _ix++ )
    {
        _eval.coeffRef( _ix , _ix )              =	p_damping;
        _eval.coeffRef( _ix + 3 , _ix + 3 ) 	=	R_damping;
    }

    /* ik */
    for ( int iter = 0; iter < max_iter; iter++ )
    {
        Eigen::MatrixXd _Jacobian		= 	calcJacobian( _idx );

        Eigen::MatrixXd _curr_position      = 	robotis_joint[ to ]->position;
        Eigen::MatrixXd _curr_orientation 	= 	robotis_joint[ to ]->orientation;

        Eigen::MatrixXd _err 	= 	calcVWerr( tar_position,_curr_position, tar_orientation, _curr_orientation );

        if ( _err.norm() < ik_err )
        {
            ik_success = true;
            break;
        }
        else
            ik_success = false;

        Eigen::MatrixXd __Jacobian      = 	( _Jacobian * _weight * _Jacobian.transpose() + _eval );
        Eigen::MatrixXd ___Jacobian     = 	_weight * _Jacobian.transpose() * __Jacobian.inverse();

        Eigen::MatrixXd _delta_angle 		= 	___Jacobian * _err ;

        for ( int id = 0; id < _idx.size(); id++ )
        {
            int _joint_num      = 	_idx[ id ];
            robotis_joint[ _joint_num ]->joint_angle    += 	_delta_angle.coeff( id );
        }

        forwardKinematics(0);
    }

    /* check joint limit */
    for ( int id = 0; id < _idx.size(); id++ )
    {
        int _joint_num      = 	_idx[ id ];

        if ( robotis_joint[ _joint_num ]->joint_angle >= robotis_joint[ _joint_num ]->joint_limit_max )
        {
            limit_success = false;
            break;
        }
        else if ( robotis_joint[ _joint_num ]->joint_angle <= robotis_joint[ _joint_num ]->joint_limit_min )
        {
            limit_success = false;
            break;
        }
        else
            limit_success = true;
    }

    if ( ik_success == true && limit_success == true )
        return true;
    else
        return false;
}

bool RobotisData::inverseKinematics( int from, int to, Eigen::MatrixXd tar_position, Eigen::MatrixXd tar_orientation, int max_iter, double ik_err , Eigen::MatrixXd weight )
{
    bool ik_success = false;
    bool limit_success = false;

    forwardKinematics(0);

    std::vector<int> _idx   = 	findRoute( from , to );

    /* weight */
    Eigen::MatrixXd _weight = Eigen::MatrixXd::Identity( _idx.size() , _idx.size() );

    for ( int _ix = 0; _ix < _idx.size(); _ix++ )
        _weight.coeffRef( _ix , _ix ) = weight.coeff( _idx[ _ix ], 0 );

    /* damping */
    Eigen::MatrixXd _eval 		= 	Eigen::MatrixXd::Zero( 6 , 6 );

    double p_damping 		= 	1e-5;
    double R_damping 		= 	1e-5;

    for ( int _ix = 0; _ix < 3; _ix++ )
    {
        _eval.coeffRef( _ix , _ix )              =	p_damping;
        _eval.coeffRef( _ix + 3 , _ix + 3 ) 	=	R_damping;
    }

    /* ik */
    for ( int iter = 0; iter < max_iter; iter++ )
    {
        Eigen::MatrixXd _Jacobian		= 	calcJacobian( _idx );

        Eigen::MatrixXd _curr_position      = 	robotis_joint[ to ]->position;
        Eigen::MatrixXd _curr_orientation 	= 	robotis_joint[ to ]->orientation;

        Eigen::MatrixXd _err 	= 	calcVWerr( tar_position,_curr_position, tar_orientation, _curr_orientation );

        if ( _err.norm() < ik_err )
        {
            ik_success = true;
            break;
        }
        else
            ik_success = false;


        Eigen::MatrixXd __Jacobian      = 	( _Jacobian * _weight * _Jacobian.transpose() + _eval );
        Eigen::MatrixXd ___Jacobian     = 	_weight * _Jacobian.transpose() * __Jacobian.inverse();

        Eigen::MatrixXd _delta_angle 		= 	___Jacobian * _err ;

        for ( int id = 0; id < _idx.size(); id++ )
        {
            int _joint_num      = 	_idx[ id ];
            robotis_joint[ _joint_num ]->joint_angle    += 	_delta_angle.coeff( id );
        }

        forwardKinematics(0);
    }

    for ( int id = 0; id < _idx.size(); id++ )
    {
        int _joint_num      = 	_idx[ id ];

        if ( robotis_joint[ _joint_num ]->joint_angle >= robotis_joint[ _joint_num ]->joint_limit_max )
        {
            limit_success = false;
            break;
        }
        else if ( robotis_joint[ _joint_num ]->joint_angle <= robotis_joint[ _joint_num ]->joint_limit_min )
        {
            limit_success = false;
            break;
        }
        else
            limit_success = true;
    }

    if ( ik_success == true && limit_success == true )
        return true;
    else
        return false;
}

}
