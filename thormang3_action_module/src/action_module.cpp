/*
 * motion_module_tutorial.cpp
 *
 *  Created on: 2016. 2. 23.
 *      Author: zerom
 */

#include <stdio.h>
#include <sstream>

#include "thormang3_action_module/action_module.h"


using namespace ROBOTIS;

std::string ActionModule::convertIntToString(int _n) {
	std::ostringstream ostr;
	ostr << _n;
	return ostr.str();
}



ActionModule::ActionModule()
    : control_cycle_msec_(8)
{
    enable          = false;
    module_name     = "action_module"; // set unique module name
    control_mode    = POSITION_CONTROL;

    //////////////////////////////////
	action_file_ = 0;
	playing_ = false;
	first_driving_start_ = false;
	playing_finished_ = true;
	page_step_count_ = 0;
	play_page_idx_ = 0;
	stop_playing_ = true;

	previous_enable_  = false;
	present_enable_   = false;
	previous_running_ = false;
	present_running_  = false;

}

ActionModule::~ActionModule()
{
    queue_thread_.join();

    ////////////////////////////////////////
	if(action_file_ != 0)
		fclose( action_file_ );
}

void ActionModule::Initialize(const int control_cycle_msec, Robot *robot)
{
    control_cycle_msec_ = control_cycle_msec;
    queue_thread_       = boost::thread(boost::bind(&ActionModule::QueueThread, this));

    for(std::map<std::string, Dynamixel*>::iterator it = robot->dxls.begin(); it != robot->dxls.end(); it++)
    {
        std::string joint_name = it->first;
        Dynamixel*  dxl_info   = it->second;

        joint_name_to_id_[joint_name]   = dxl_info->id;
        joint_id_to_name_[dxl_info->id] = joint_name;
        result[joint_name] = new DynamixelState();
        result[joint_name]->goal_position = dxl_info->dxl_state->goal_position;
    }


    ros::NodeHandle ros_node;

    std::string path = ros::package::getPath("thormang3_action_module") + "/data/motion_4095.bin";
    std::string action_file_path = ros_node.param<std::string>("action_file_path", path);

    loadFile(action_file_path);


    playing_ = false;
}

void ActionModule::QueueThread()
{
    ros::NodeHandle     ros_node;
    ros::CallbackQueue  callback_queue;

    ros_node.setCallbackQueue(&callback_queue);

    /* subscriber */
    action_page_sub_ = ros_node.subscribe("/robotis/action/page_num", 1, &ActionModule::pageNumberCallback, this);

    /* publisher */
    status_msg_pub_ = ros_node.advertise<robotis_controller_msgs::StatusMsg>("/robotis/status", 0);

    while(ros_node.ok())
    {
        callback_queue.callAvailable();
        usleep(100);
    }
}


void ActionModule::pageNumberCallback(const std_msgs::Int32::ConstPtr& msg)
{

	if(msg->data == -1) {
		Stop();
	}
	else if(msg->data == -2) {
		brake();
	}
	else {
		if(start(msg->data) == true) {
	    	std::string status_msg = "Succeed to start page " + convertIntToString(msg->data);
	    	ROS_INFO_STREAM(status_msg);
			publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, status_msg);
		}
		else {
	    	std::string status_msg = "Failed to start page " + convertIntToString(msg->data);
	    	ROS_ERROR_STREAM(status_msg);
			publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, status_msg);
		}
	}
}


void ActionModule::Process(std::map<std::string, Dynamixel *> dxls, std::map<std::string, double> sensors)
{
	previous_enable_ = present_enable_;
	present_enable_  = enable;

    if(enable == false)
        return;

    if((present_enable_ == true) && (present_enable_ != previous_enable_)) {
    	for(std::map<std::string, Dynamixel *>::iterator it = dxls.begin() ; it != dxls.end(); it++)
    	{
    		std::string joint_name = it->first;

    		if(result.find(joint_name) == result.end())
    			continue;
    		else {
    			result[joint_name]->goal_position = it->second->dxl_state->goal_position;
    		}
    	}
    }


	previous_running_ = present_running_;
	present_running_  = IsRunning();

	if(present_running_ != previous_running_) {
		if(present_running_ == true) {
	    	std::string status_msg = "Action_Start";
	    	ROS_INFO_STREAM(status_msg);
			publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, status_msg);
		}
		else {
	    	std::string status_msg = "Action_Finish";
	    	ROS_INFO_STREAM(status_msg);
			publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, status_msg);
		}
	}

    actionPlayProcess(dxls);
}


void ActionModule::OnModuleEnable()
{
    previous_enable_ = false;
    present_enable_ = false;
}

void ActionModule::OnModuleDisable()
{
    previous_enable_ = false;
    present_enable_ = false;
}


void ActionModule::Stop()
{
	stop_playing_ = true;
}

bool ActionModule::IsRunning()
{
	return playing_;
}

int  ActionModule::convertRadTow4095(double rad)
{
	return (int)((rad + M_PI)*2048.0/M_PI);
}

double ActionModule::convertw4095ToRad(int w4095)
{
	return (w4095 - 2048)*M_PI/2048.0;
}


bool ActionModule::verifyChecksum( action_file_define::Page* page )
{
	unsigned char  checksum = 0x00;
    unsigned char* pt = (unsigned char*)page;

    for(unsigned int i = 0; i < sizeof(action_file_define::Page); i++)
    {
        checksum += *pt;
        pt++;
    }

    if(checksum != 0xff)
        return false;

	return true;
}

void ActionModule::setChecksum( action_file_define::Page* page )
{
	unsigned char  checksum = 0x00;
    unsigned char* pt = (unsigned char*)page;

    page->header.checksum = 0x00;

    for(unsigned int i=0; i<sizeof(action_file_define::Page); i++)
    {
        checksum += *pt;
        pt++;
    }

    page->header.checksum = (unsigned char)(0xff - checksum);
}



bool ActionModule::loadFile(std::string file_name)
{
	FILE* action = fopen( file_name.c_str(), "r+b" );
    if( action == 0 )
	{
    	std::string status_msg = "Can not open Action file!";
    	ROS_ERROR_STREAM(status_msg);
		publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, status_msg);
        return false;
	}

    fseek( action, 0, SEEK_END );
    if( ftell(action) != (long)(sizeof(action_file_define::Page) * action_file_define::MAXNUM_PAGE) )
    {
    	std::string status_msg = "It's not an Action file!";
    	ROS_ERROR_STREAM(status_msg);
		publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, status_msg);
        fclose( action );
        return false;
    }

	if(action_file_ != 0)
		fclose( action_file_ );

	action_file_ = action;
	return true;
}

bool ActionModule::createFile(std::string file_name)
{
	FILE* action = fopen( file_name.c_str(), "ab" );
	if( action == 0 )
	{
		std::string status_msg = "Can not create Action file!";
		ROS_ERROR_STREAM(status_msg);
		publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, status_msg);
		return false;
	}

	action_file_define::Page page;
	resetPage(&page);

	for(int i=0; i < action_file_define::MAXNUM_PAGE; i++)
		fwrite((const void *)&page, 1, sizeof(action_file_define::Page), action);

	if(action_file_ != 0)
		fclose( action_file_ );

	action_file_ = action;

	return true;
}

bool ActionModule::start(int page_number)
{
	if( page_number < 1 || page_number >= action_file_define::MAXNUM_PAGE )
	{

		std::string _status_msg = "Can not play page.(" + convertIntToString(page_number) + " is invalid index)";
		ROS_ERROR_STREAM(_status_msg);
		publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, _status_msg);
        return false;
	}

	action_file_define::Page page;
	if( loadPage(page_number, &page) == false )
        return false;

	return start(page_number, &page);
}

bool ActionModule::start(std::string page_name)
{
	int index;
	action_file_define::Page page;

	for(index=1; index < action_file_define::MAXNUM_PAGE; index++)
	{
		if(loadPage(index, &page) == false)
			return false;

		if(strcmp(page_name.c_str(), (char*)page.header.name) == 0)
			break;
	}

	if(index == action_file_define::MAXNUM_PAGE) {
		std::string _str_name_page = page_name;
		std::string _status_msg = "Can not play page.(" + _str_name_page + " is invalid name)\n";
		ROS_ERROR_STREAM(_status_msg);
		publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, _status_msg);
        return false;
	}
	else
		return start(index, &page);
}

bool ActionModule::start(int page_number, action_file_define::Page* page)
{
	if(enable == false)	{
		std::string status_msg = "Action Module is disabled";
		ROS_ERROR_STREAM(status_msg);
		publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, status_msg);
		return false;
	}

	if(playing_ == true)
	{
		std::string status_msg = "Can not play page " + convertIntToString(page_number) + ".(Now playing)";
		ROS_ERROR_STREAM(status_msg);
		publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, status_msg);
        return false;
	}

	play_page_ = *page;

    if( play_page_.header.repeat == 0 || play_page_.header.stepnum == 0 )
	{
		std::string status_msg = "Page " + convertIntToString(page_number) + " has no action\n";
		ROS_ERROR_STREAM(status_msg);
		publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, status_msg);
        return false;
	}

    play_page_idx_ = page_number;
    first_driving_start_ = true;
    playing_ = true;

	return true;
}

void ActionModule::brake()
{
	playing_ = false;
}

bool ActionModule::isRunning(int* playing_page_num, int* playing_step_num)
{
	if(playing_page_num != 0)
		*playing_page_num = play_page_idx_;

	if(playing_step_num != 0)
		*playing_step_num = page_step_count_ - 1;

	return IsRunning();
}

bool ActionModule::loadPage(int page_number, action_file_define::Page* page)
{
	long position = (long)(sizeof(action_file_define::Page)*page_number);

    if( fseek( action_file_, position, SEEK_SET ) != 0 )
        return false;

    if( fread( page, 1, sizeof(action_file_define::Page), action_file_ ) != sizeof(action_file_define::Page) )
        return false;

    if( verifyChecksum( page ) == false )
        resetPage( page );

	return true;
}

bool ActionModule::savePage(int page_number, action_file_define::Page* page)
{
	long position = (long)(sizeof(action_file_define::Page)*page_number);

	if( verifyChecksum(page) == false )
        setChecksum(page);

    if( fseek( action_file_, position, SEEK_SET ) != 0 )
        return false;

    if( fwrite( page, 1, sizeof(action_file_define::Page), action_file_ ) != sizeof(action_file_define::Page) )
        return false;

	return true;
}

void ActionModule::resetPage(action_file_define::Page* page)
{
	unsigned char *pt = (unsigned char*)page;

    for(unsigned int i=0; i<sizeof(action_file_define::Page); i++)
    {
        *pt = 0x00;
        pt++;
    }

    page->header.schedule = action_file_define::TIME_BASE_SCHEDULE; // default time base
    page->header.repeat = 1;
    page->header.speed = 32;
    page->header.accel = 32;

	for(int i=0; i < 38; i++)
	    page->header.pgain[i] = 0x55;

    for(int i=0; i < action_file_define::MAXNUM_STEP; i++)
    {
        for(int j=0; j < 38; j++)
            page->step[i].position[j] = action_file_define::INVALID_BIT_MASK;

        page->step[i].pause = 0;
        page->step[i].time = 0;
    }

    setChecksum( page );
}

void ActionModule::actionPlayProcess(std::map<std::string, Dynamixel *> dxls)
{
    //////////////////// local Variable
    uint8_t bID;
    uint32_t ulTotalTime256T;
    uint32_t ulPreSectionTime256T;
    uint32_t ulMainTime256T;
    int32_t lStartSpeed1024_PreTime_256T;
    int32_t lMovingAngle_Speed1024Scale_256T_2T;
    int32_t lDivider1,lDivider2;
    //unsigned short
    int16_t wMaxAngle1024;
    int16_t wMaxSpeed256;
    int16_t wTmp;
    int16_t wPrevTargetAngle; // Start position
    int16_t wCurrentTargetAngle; // Target position
    int16_t wNextTargetAngle; // Next target position
    uint8_t bDirectionChanged;

    ///////////////// Static Variable
	static uint16_t wpStartAngle1024[action_file_define::MAXNUM_JOINTS];   // Start point of interpolation
    static uint16_t wpTargetAngle1024[action_file_define::MAXNUM_JOINTS];  // Target point of interpolation
    static int16_t  ipMovingAngle1024[action_file_define::MAXNUM_JOINTS];  // Total Moving Angle
    static int16_t  ipMainAngle1024[action_file_define::MAXNUM_JOINTS];    // Moving angle at Constant Velocity Section
    static int16_t  ipAccelAngle1024[action_file_define::MAXNUM_JOINTS];   // Moving angle at Acceleration Section
    static int16_t  ipMainSpeed1024[action_file_define::MAXNUM_JOINTS];    // Target constant velocity
    static int16_t  ipLastOutSpeed1024[action_file_define::MAXNUM_JOINTS]; // Velocity of Previous State
    static int16_t  ipGoalSpeed1024[action_file_define::MAXNUM_JOINTS];    // Target velocity
    static uint8_t  bpFinishType[action_file_define::MAXNUM_JOINTS];       // Desired State at Target angle
    int16_t iSpeedN;
    static uint16_t wUnitTimeCount;
    static uint16_t wUnitTimeNum;
    static uint16_t wPauseTime;
    static uint16_t wUnitTimeTotalNum;
    static uint16_t wAccelStep;
    static uint8_t  bSection;
    static uint8_t  bPlayRepeatCount;
    static uint16_t wNextPlayPage;

    /////////////// Enum Variable

    /**************************************
    * Section             /----\
    *                    /|    |\
    *        /+---------/ |    | \
    *       / |        |  |    |  \
    * -----/  |        |  |    |   \----
    *      PRE  MAIN   PRE MAIN POST PAUSE
    ***************************************/
    enum{ PRE_SECTION, MAIN_SECTION, POST_SECTION, PAUSE_SECTION };
    enum{ ZERO_FINISH, NONE_ZERO_FINISH};

    if( playing_ == false )
        return;

    if( first_driving_start_ == true ) // First of Start Process
    {
        first_driving_start_ = false;  //First Process end
        playing_finished_ = false;
		stop_playing_ = false;
        wUnitTimeCount = 0;
        wUnitTimeNum = 0;
        wPauseTime = 0;
        bSection = PAUSE_SECTION;
        page_step_count_ = 0;
        bPlayRepeatCount = play_page_.header.repeat;
        wNextPlayPage = 0;


		for(unsigned int _jointIndex = 0; _jointIndex < action_file_define::MAXNUM_JOINTS; _jointIndex++)
		{
			bID = _jointIndex;
			std::string _joint_name = "";

			if(joint_id_to_name_.find(bID) == joint_id_to_name_.end())
				continue;
			else
				_joint_name = joint_id_to_name_[bID];

			if(dxls.find(_joint_name) == dxls.end())
				continue;
			else {
				double _goal_joint_angle_rad = dxls[joint_id_to_name_[bID]]->dxl_state->goal_position;
				wpTargetAngle1024[bID] = convertRadTow4095(_goal_joint_angle_rad);
				ipLastOutSpeed1024[bID] = 0;
				ipMovingAngle1024[bID] = 0;
				ipGoalSpeed1024[bID] = 0;
			}
		}
    }


    if( wUnitTimeCount < wUnitTimeNum ) // If state is ongoing
    {
        wUnitTimeCount++;
        if( bSection == PAUSE_SECTION )
        {
        }
        else
        {
            for(unsigned int _jointIndex = 0; _jointIndex < action_file_define::MAXNUM_JOINTS; _jointIndex++ )
            {
				bID = _jointIndex;
				std::string _joint_name = "";

				if(joint_id_to_name_.find(bID) == joint_id_to_name_.end())
					continue;
				else
					_joint_name = joint_id_to_name_[bID];

				if(dxls.find(_joint_name) == dxls.end())
					continue;
				else {
					if( ipMovingAngle1024[bID] == 0 )
					{
						result[_joint_name]->goal_position = convertw4095ToRad(wpStartAngle1024[bID]);
					}
					else
					{
						if( bSection == PRE_SECTION )
						{
							iSpeedN = (short)( ( (long)(ipMainSpeed1024[bID] - ipLastOutSpeed1024[bID]) * wUnitTimeCount ) / wUnitTimeNum );
							ipGoalSpeed1024[bID] = ipLastOutSpeed1024[bID] + iSpeedN;
							ipAccelAngle1024[bID] =  (short)( ( ( (long)( ipLastOutSpeed1024[bID] + (iSpeedN >> 1) ) * wUnitTimeCount * 144 ) / 15 ) >> 9);

							result[_joint_name]->goal_position = convertw4095ToRad(wpStartAngle1024[bID] + ipAccelAngle1024[bID]);
						}
						else if( bSection == MAIN_SECTION )
						{
							result[_joint_name]->goal_position	= convertw4095ToRad(wpStartAngle1024[bID] + (short int)(((long)(ipMainAngle1024[bID])*wUnitTimeCount) / wUnitTimeNum));

							ipGoalSpeed1024[bID] = ipMainSpeed1024[bID];
						}
						else // POST_SECTION
						{
							if( wUnitTimeCount == (wUnitTimeNum-1) )
							{
							    // use target angle in order to reduce the last step error
								result[_joint_name]->goal_position	= convertw4095ToRad(wpTargetAngle1024[bID]);
							}
							else
							{
								if( bpFinishType[bID] == ZERO_FINISH )
								{
									iSpeedN = (short int)(((long)(0 - ipLastOutSpeed1024[bID]) * wUnitTimeCount) / wUnitTimeNum);
									ipGoalSpeed1024[bID] = ipLastOutSpeed1024[bID] + iSpeedN;

									result[_joint_name]->goal_position
									= convertw4095ToRad(wpStartAngle1024[bID] +  (short)((((long)(ipLastOutSpeed1024[bID] + (iSpeedN>>1)) * wUnitTimeCount * 144) / 15) >> 9));

								}
								else // NONE_ZERO_FINISH
								{
									// Same as MAIN Section
									// because some servos need to be rotate, others do not.
									result[_joint_name]->goal_position
									= convertw4095ToRad(wpStartAngle1024[bID] + (short int)(((long)(ipMainAngle1024[bID]) * wUnitTimeCount) / wUnitTimeNum));

									ipGoalSpeed1024[bID] = ipMainSpeed1024[bID];
								}
							}
						}
					}
					//result[_joint_name]->position_p_gain = ( 256 >> (play_page_.header.pgain[bID] >> 4) ) << 2 ;
				}
			}
		}
	}
    else if( wUnitTimeCount >= wUnitTimeNum )  // If current section is completed
    {
        wUnitTimeCount = 0;

        for(unsigned int _jointIndex = 0; _jointIndex < action_file_define::MAXNUM_JOINTS; _jointIndex++)
        {
			bID = _jointIndex;
			std::string _joint_name = "";

			if(joint_id_to_name_.find(bID) == joint_id_to_name_.end())
				continue;
			else
				_joint_name = joint_id_to_name_[bID];

			if(dxls.find(_joint_name) == dxls.end())
				continue;
			else {
				double _goal_joint_angle_rad = dxls[joint_id_to_name_[bID]]->dxl_state->goal_position;
				wpStartAngle1024[bID] = convertRadTow4095(_goal_joint_angle_rad);
				ipLastOutSpeed1024[bID] = ipGoalSpeed1024[bID];
			}
        }

        // Section will be updated ( PRE -> MAIN -> POST -> (PAUSE or PRE) ... )
        if( bSection == PRE_SECTION )
        {
            //preparations for MAIN section
            bSection = MAIN_SECTION;
            wUnitTimeNum =  wUnitTimeTotalNum - (wAccelStep << 1);

            for(unsigned int _jointIndex = 0; _jointIndex < action_file_define::MAXNUM_JOINTS; _jointIndex++)
			{
				bID = _jointIndex;

				if( bpFinishType[bID] == NONE_ZERO_FINISH )
				{
					if( (wUnitTimeTotalNum - wAccelStep) == 0 ) // �뜝�럥�맶�뜝�럥吏쀥뜝�럩援꿨뜝�럥�맶占쎈쐻�뜝占� �뜝�럥�맶�뜝�럥吏쀥뜝�럩援꿨뜝�럥�맶�뜝�럥吏쀥뜝�럩援꿨뜝�럥�맶�뜝�럥吏쀥뜝�럩援� �뜝�럥�맶�뜝�럥吏쀥뜝�럩援꿨뜝�럥�맶�뜝�럥吏쀥뜝�럩援� �뜝�럥�맶�뜝�럥吏쀥뜝�럩援꿨뜝�럥�맶�뜝�럥�냿嶺뚮ㅏ�뒩占쎄뎡
						ipMainAngle1024[bID] = 0;
					else
						ipMainAngle1024[bID] = (short)((((long)(ipMovingAngle1024[bID] - ipAccelAngle1024[bID])) * wUnitTimeNum) / (wUnitTimeTotalNum - wAccelStep));
				}
				else // ZERO_FINISH
					ipMainAngle1024[bID] = ipMovingAngle1024[bID] - ipAccelAngle1024[bID] - (short int)((((long)ipMainSpeed1024[bID] * wAccelStep * 12) / 5) >> 8);
			}
        }
        else if( bSection == MAIN_SECTION )
        {
            //preparations for POST Section
            bSection = POST_SECTION;
            wUnitTimeNum = wAccelStep;

            for(unsigned int _jointIndex = 0; _jointIndex < action_file_define::MAXNUM_JOINTS; _jointIndex++)
			{
				bID = _jointIndex;
				ipMainAngle1024[bID] = ipMovingAngle1024[bID] - ipMainAngle1024[bID] - ipAccelAngle1024[bID];
			}
        }
        else if( bSection == POST_SECTION )
        {
            //it will be decided by Pause time exist or not
            if( wPauseTime )
            {
                bSection = PAUSE_SECTION;
                wUnitTimeNum = wPauseTime;
            }
            else
            {
                bSection = PRE_SECTION;
            }
        }
        else if( bSection == PAUSE_SECTION )
        {
            //preparations for PRE Section
            bSection = PRE_SECTION;

            for(unsigned int _jointIndex = 0; _jointIndex < action_file_define::MAXNUM_JOINTS; _jointIndex++)
			{
            	bID = _jointIndex;
				ipLastOutSpeed1024[bID] = 0;
			}
        }

        // all is prepared in PRE Section
        if( bSection == PRE_SECTION )
        {
            if( playing_finished_ == true ) // If motion is finished
            {
                playing_ = false;
                return;
            }

            page_step_count_++;

            if( page_step_count_ > play_page_.header.stepnum ) // If motion playing for present page is finished
            {
                // copy next page to play page
                play_page_ = next_play_page_;
                if( play_page_idx_ != wNextPlayPage )
                    bPlayRepeatCount = play_page_.header.repeat;
                page_step_count_ = 1;
                play_page_idx_ = wNextPlayPage;
            }

            if( page_step_count_ == play_page_.header.stepnum ) // If this step is last
            {
                // load next page
                if( stop_playing_ == true ) // If there is STOP command
                {
                    wNextPlayPage = play_page_.header.exit; // Next page will be set to exit page
                }
                else
                {
                    bPlayRepeatCount--;
                    if( bPlayRepeatCount > 0 ) // If repeat number is remained
                        wNextPlayPage = play_page_idx_; // Next page will be set to current page
                    else // If repeat is finished
                        wNextPlayPage = play_page_.header.next; // Next page will be set to NEXT page
                }

                if( wNextPlayPage == 0 ) // If there is no NEXT page, the motion playing will be finished after current step.
                    playing_finished_ = true;
                else
                {
                    // load NEXT Page(If page is same, memory copy will be conducted. If not, read from file will be conducted)
                    if( play_page_idx_ != wNextPlayPage )
                        loadPage( wNextPlayPage, &next_play_page_ );
                    else
                        next_play_page_ = play_page_;

                    // If there is no playing information, the motion playing will be finished after current step.
                    if( next_play_page_.header.repeat == 0 || next_play_page_.header.stepnum == 0 )
                        playing_finished_ = true;
                }
            }

            //////// Calc Step Parameter
            wPauseTime = (((unsigned short)play_page_.step[page_step_count_-1].pause) << 5) / play_page_.header.speed;
            wMaxSpeed256 = ((unsigned short)play_page_.step[page_step_count_-1].time * (unsigned short)play_page_.header.speed) >> 5;
            if( wMaxSpeed256 == 0 )
                wMaxSpeed256 = 1;
            wMaxAngle1024 = 0;

            ////////// Calc parameter for each joint
            for(unsigned int _jointIndex = 0; _jointIndex < action_file_define::MAXNUM_JOINTS; _jointIndex++)
			{
				bID = _jointIndex;
				//Calc the trajectory using previous, present and future
				ipAccelAngle1024[bID] = 0;

				// Find current target angle
				if( play_page_.step[page_step_count_-1].position[bID] & action_file_define::INVALID_BIT_MASK )
					wCurrentTargetAngle = wpTargetAngle1024[bID];
				else
					wCurrentTargetAngle = play_page_.step[page_step_count_-1].position[bID];

				// Update start, prev_target, curr_target
				wpStartAngle1024[bID] = wpTargetAngle1024[bID];
				wPrevTargetAngle = wpTargetAngle1024[bID];
				wpTargetAngle1024[bID] = wCurrentTargetAngle;

				// Find Moving offset
				ipMovingAngle1024[bID] = (int)(wpTargetAngle1024[bID] - wpStartAngle1024[bID]);

				// Find Next target angle
				if( page_step_count_ == play_page_.header.stepnum ) // If current step is last step
				{
					if( playing_finished_ == true ) // If it will be finished
						wNextTargetAngle = wCurrentTargetAngle;
					else
					{
						if( next_play_page_.step[0].position[bID] & action_file_define::INVALID_BIT_MASK )
							wNextTargetAngle = wCurrentTargetAngle;
						else
							wNextTargetAngle = next_play_page_.step[0].position[bID];
					}
				}
				else
				{
					if( play_page_.step[page_step_count_].position[bID] & action_file_define::INVALID_BIT_MASK )
						wNextTargetAngle = wCurrentTargetAngle;
					else
						wNextTargetAngle = play_page_.step[page_step_count_].position[bID];
				}

				// Find direction change
				if( ((wPrevTargetAngle < wCurrentTargetAngle) && (wCurrentTargetAngle < wNextTargetAngle))
					|| ((wPrevTargetAngle > wCurrentTargetAngle) && (wCurrentTargetAngle > wNextTargetAngle)) )
				{
				    // if there is no discontinuous point
					bDirectionChanged = 0;
				}
				else
				{
					bDirectionChanged = 1;
				}

				// Find finish type
				if( bDirectionChanged || wPauseTime || playing_finished_ == true )
				{
					bpFinishType[bID] = ZERO_FINISH;
				}
				else
				{
					bpFinishType[bID] = NONE_ZERO_FINISH;
				}

				if( play_page_.header.schedule == action_file_define::SPEED_BASE_SCHEDULE )
				{
					//MaxAngle1024 update
					if( ipMovingAngle1024[bID] < 0 )
						wTmp = -ipMovingAngle1024[bID];
					else
						wTmp = ipMovingAngle1024[bID];

					if( wTmp > wMaxAngle1024 )
						wMaxAngle1024 = wTmp;
				}

			}

            // calculation the time. And, the calculated time will be divided by 7.8msec(<<7)- calculate there are how many 7.8msec
            // after unit conversion, calculate angle/velocity, and then calculate there are how many 7.8msec in the calculated time
            //unit conversion ---  angle :1024->300deg,  velocity: 256 ->720
            //wUnitTimeNum = ((wMaxAngle1024*300/1024) /(wMaxSpeed256 * 720/256)) /7.8msec;
            //             = ((128*wMaxAngle1024*300/1024) /(wMaxSpeed256 * 720/256)) ;    (/7.8msec == *128)
            //             = (wMaxAngle1024*40) /(wMaxSpeed256 *3);
            if( play_page_.header.schedule == action_file_define::TIME_BASE_SCHEDULE )
                wUnitTimeTotalNum  = wMaxSpeed256; //TIME BASE 051025
            else
                wUnitTimeTotalNum  = (wMaxAngle1024 * 40) / (wMaxSpeed256 * 3);

            wAccelStep = play_page_.header.accel;
            if( wUnitTimeTotalNum <= (wAccelStep << 1) )
            {
                if( wUnitTimeTotalNum == 0 )
                    wAccelStep = 0;
                else
                {
                    wAccelStep = (wUnitTimeTotalNum - 1) >> 1;
                    if( wAccelStep == 0 )
                        wUnitTimeTotalNum = 0; //If motor will be rotated, acceleration and constant velocity step should exist more than one step
                }
            }

            ulTotalTime256T = ((unsigned long)wUnitTimeTotalNum) << 1;// /128 * 256
            ulPreSectionTime256T = ((unsigned long)wAccelStep) << 1;// /128 * 256
            ulMainTime256T = ulTotalTime256T - ulPreSectionTime256T;
            lDivider1 = ulPreSectionTime256T + (ulMainTime256T << 1);
            lDivider2 = (ulMainTime256T << 1);

            if(lDivider1 == 0)
                lDivider1 = 1;

            if(lDivider2 == 0)
                lDivider2 = 1;

            for(unsigned int _jointIndex = 0; _jointIndex < action_file_define::MAXNUM_JOINTS; _jointIndex++)
			{
				bID = _jointIndex;
				lStartSpeed1024_PreTime_256T = (long)ipLastOutSpeed1024[bID] * ulPreSectionTime256T; //  *300/1024 * 1024/720 * 256 * 2
				lMovingAngle_Speed1024Scale_256T_2T = (((long)ipMovingAngle1024[bID]) * 2560L) / 12;

				if( bpFinishType[bID] == ZERO_FINISH )
					ipMainSpeed1024[bID] = (short int)((lMovingAngle_Speed1024Scale_256T_2T - lStartSpeed1024_PreTime_256T) / lDivider2);
				else
					ipMainSpeed1024[bID] = (short int)((lMovingAngle_Speed1024Scale_256T_2T - lStartSpeed1024_PreTime_256T) / lDivider1);

				if( ipMainSpeed1024[bID] > 1023 )
					ipMainSpeed1024[bID] = 1023;

				if( ipMainSpeed1024[bID] < -1023 )
					ipMainSpeed1024[bID] = -1023;

			}
            wUnitTimeNum = wAccelStep; //PreSection
        }
    }
}


void ActionModule::publishStatusMsg(unsigned int type, std::string msg)
{
    robotis_controller_msgs::StatusMsg status;
    status.header.stamp = ros::Time::now();
    status.type = type;
    status.module_name = "Action";
    status.status_msg = msg;

    status_msg_pub_.publish(status);
}

