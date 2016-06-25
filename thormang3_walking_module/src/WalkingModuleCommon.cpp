#include <thormang3_walking_module/WalkingModuleCommon.h>

namespace ROBOTIS {
const int WalkingStateFlag::InWalkingStarting	= 0;
const int WalkingStateFlag::InWalking 			= 1;
const int WalkingStateFlag::InWalkingEnding	= 2;

const int MovingFootFlag::LFootMove = 1;
const int MovingFootFlag::RFootMove = 2;
const int MovingFootFlag::NFootMove = 3;

const int STEP_DATA_ERR::NO_ERROR					= 0;
const int STEP_DATA_ERR::NOT_ENABLED_WALKING_MODULE	= 2;
const int STEP_DATA_ERR::PROBLEM_IN_POSITION_DATA	= 4;
const int STEP_DATA_ERR::PROBLEM_IN_TIME_DATA		= 8;
const int STEP_DATA_ERR::ROBOT_IS_WALKING_NOW 		= 1024;

const int WALKING_START_ERR::NO_ERROR					= 0;
const int WALKING_START_ERR::NOT_ENABLED_WALKING_MODULE	= 2;
const int WALKING_START_ERR::NO_STEP_DATA				= 16;
const int WALKING_START_ERR::ROBOT_IS_WALKING_NOW 		= 1024;

const int BALANCE_PARAM_ERR::NO_ERROR 				    	= 0;
const int BALANCE_PARAM_ERR::NOT_ENABLED_WALKING_MODULE		= 2;
const int BALANCE_PARAM_ERR::PREV_REQUEST_IS_NOT_FINISHED	= 32;
const int BALANCE_PARAM_ERR::TIME_CONST_IS_ZERO_OR_NEGATIVE	= 64;

const int REMOVE_STEP_DATA_ERR::NO_ERROR				= 0;
const int REMOVE_STEP_DATA_ERR::ROBOT_IS_WALKING_NOW	= 1024;

const std::string WALKING_STATUS_MSG::FAILED_TO_ADD_STEP_DATA_MSG = "Failed_to_add_Step_Data";
const std::string WALKING_STATUS_MSG::BALANCE_PARAM_SETTING_START_MSG = "Balance_Param_Setting_Started";
const std::string WALKING_STATUS_MSG::BALANCE_PARAM_SETTING_FINISH_MSG = "Balance_Param_Setting_Finished";
const std::string WALKING_STATUS_MSG::WALKING_MODULE_IS_ENABLED_MSG = "Walking_Module_is_enabled";
const std::string WALKING_STATUS_MSG::WALKING_MODULE_IS_DISABLED_MSG = "Walking_Module_is_enabled";
const std::string WALKING_STATUS_MSG::WALKING_START_MSG = "Walking_Started";
const std::string WALKING_STATUS_MSG::WALKING_FINISH_MSG = "Walking_Finished";
}
