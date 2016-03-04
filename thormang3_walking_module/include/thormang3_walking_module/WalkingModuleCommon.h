/*
 * WalkingModuleCommon.h
 *
 *  Created on: Feb 15, 2016
 *      Author: jaysbc
 */

#ifndef SRC_ROBOTIS_THORMANG_MPC_THORMANG3_WALKING_MODULE_INCLUDE_THORMANG3_WALKING_MODULE_WALKINGMODULECOMMON_H_
#define SRC_ROBOTIS_THORMANG_MPC_THORMANG3_WALKING_MODULE_INCLUDE_THORMANG3_WALKING_MODULE_WALKINGMODULECOMMON_H_

#include <string>

namespace ROBOTIS {
class WalkingStateFlag
{
public:
	static const int InWalkingStarting	= 0;
	static const int InWalking 			= 1;
	static const int InWalkingEnding	= 2;
};

class MovingFootFlag
{
public:
	static const int LFootMove = 1;
	static const int RFootMove = 2;
	static const int NFootMove = 3;
};


class STEP_DATA_ERR {
public:
	static const int NO_ERROR					= 0;
	static const int NOT_ENABLED_WALKING_MODULE	= 2;
	static const int PROBLEM_IN_POSITION_DATA	= 4;
	static const int PROBLEM_IN_TIME_DATA		= 8;
	static const int ROBOT_IS_WALKING_NOW 		= 1024;
};

class WALKING_START_ERR {
public:
	static const int NO_ERROR					= 0;
	static const int NOT_ENABLED_WALKING_MODULE	= 2;
	static const int NO_STEP_DATA				= 16;
	static const int ROBOT_IS_WALKING_NOW 		= 1024;
};

class BALANCE_PARAM_ERR {
public:
	static const int NO_ERROR 				    	= 0;
	static const int NOT_ENABLED_WALKING_MODULE		= 2;
	static const int PREV_REQUEST_IS_NOT_FINISHED	= 32;
	static const int TIME_CONST_IS_ZERO_OR_NEGATIVE	= 64;
};

class REMOVE_STEP_DATA_ERR {
public:
	static const int NO_ERROR				= 0;
	static const int ROBOT_IS_WALKING_NOW	= 1024;
};


class WALKING_STATUS_MSG {
public:
	static const std::string FAILED_TO_ADD_STEP_DATA_MSG;
	static const std::string BALANCE_PARAM_SETTING_START_MSG;
	static const std::string BALANCE_PARAM_SETTING_FINISH_MSG;
	static const std::string WALKING_MODULE_IS_ENABLED_MSG;
	static const std::string WALKING_MODULE_IS_DISABLED_MSG;
	static const std::string WALKING_START_MSG;
	static const std::string WALKING_FINISH_MSG;
};

const std::string WALKING_STATUS_MSG::FAILED_TO_ADD_STEP_DATA_MSG = "Failed_to_add_Step_Data";
const std::string WALKING_STATUS_MSG::BALANCE_PARAM_SETTING_START_MSG = "Balance_Param_Setting_Started";
const std::string WALKING_STATUS_MSG::BALANCE_PARAM_SETTING_FINISH_MSG = "Balance_Param_Setting_Finished";
const std::string WALKING_STATUS_MSG::WALKING_MODULE_IS_ENABLED_MSG = "Walking_Module_is_enabled";
const std::string WALKING_STATUS_MSG::WALKING_MODULE_IS_DISABLED_MSG = "Walking_Module_is_enabled";
const std::string WALKING_STATUS_MSG::WALKING_START_MSG = "Walking_Started";
const std::string WALKING_STATUS_MSG::WALKING_FINISH_MSG = "Walking_Finished";
}

#endif /* SRC_ROBOTIS_THORMANG_MPC_THORMANG3_WALKING_MODULE_INCLUDE_THORMANG3_WALKING_MODULE_WALKINGMODULECOMMON_H_ */
