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
	static const int InWalkingStarting;
	static const int InWalking;
	static const int InWalkingEnding;
};

class MovingFootFlag
{
public:
	static const int LFootMove;
	static const int RFootMove;
	static const int NFootMove;
};


class STEP_DATA_ERR {
public:
	static const int NO_ERROR;
	static const int NOT_ENABLED_WALKING_MODULE;
	static const int PROBLEM_IN_POSITION_DATA;
	static const int PROBLEM_IN_TIME_DATA;
	static const int ROBOT_IS_WALKING_NOW;
};

class WALKING_START_ERR {
public:
	static const int NO_ERROR;
	static const int NOT_ENABLED_WALKING_MODULE;
	static const int NO_STEP_DATA;
	static const int ROBOT_IS_WALKING_NOW;
};

class BALANCE_PARAM_ERR {
public:
	static const int NO_ERROR;
	static const int NOT_ENABLED_WALKING_MODULE;
	static const int PREV_REQUEST_IS_NOT_FINISHED;
	static const int TIME_CONST_IS_ZERO_OR_NEGATIVE;
};

class REMOVE_STEP_DATA_ERR {
public:
	static const int NO_ERROR;
	static const int ROBOT_IS_WALKING_NOW;
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
}

#endif /* SRC_ROBOTIS_THORMANG_MPC_THORMANG3_WALKING_MODULE_INCLUDE_THORMANG3_WALKING_MODULE_WALKINGMODULECOMMON_H_ */
