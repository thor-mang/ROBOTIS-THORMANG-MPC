/*
 * jointstate.cpp
 *
 *  Created on: Jul 10, 2015
 *      Author: sch
 */

#include "thormang3_manipulation_module/RobotisCommon.h"
#include "thormang3_manipulation_module/JointState.h"

namespace ROBOTIS_MANIPULATION
{

JointData JointState::goal_joint_state[ MAX_JOINT_ID + 1 ]; // goal state
JointData JointState::curr_joint_state[ MAX_JOINT_ID + 1 ]; // current state
JointData JointState::fake_joint_state[ MAX_JOINT_ID + 1 ]; // current state

}

