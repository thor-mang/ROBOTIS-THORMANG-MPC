/*
 * motion_module_tutorial.h
 *
 *  Created on: 2016. 2. 23.
 *      Author: zerom
 */

#ifndef ACTION_MOTION_MODULE_H_
#define ACTION_MOTION_MODULE_H_

#define _USE_MATH_DEFINES
#include <cmath>
#include <map>

#include <ros/ros.h>
#include <ros/package.h>
#include <ros/callback_queue.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <boost/thread.hpp>

#include "robotis_framework_common/motion_module.h"
#include "robotis_controller_msgs/StatusMsg.h"

#include "thormang3_action_module_msgs/IsRunning.h"

#include "action_file_define.h"

namespace thormang3
{

class ActionModule : public robotis_framework::MotionModule, public robotis_framework::Singleton<ActionModule>
{
private:
    int             control_cycle_msec_;
    boost::thread   queue_thread_;

    ros::Subscriber action_page_sub_;
    ros::Publisher  status_msg_pub_;



    void QueueThread();


    /////////////////////////////////////////////////////////////////////////
    std::map<std::string, int> joint_name_to_id_;
    std::map<int, std::string> joint_id_to_name_;
    FILE* action_file_;
    action_file_define::Page play_page_;
    action_file_define::Page next_play_page_;
    action_file_define::Step current_step_;


	int  play_page_idx_;
	bool first_driving_start_;
	int  page_step_count_;

	bool playing_;
	bool stop_playing_;
	bool playing_finished_;

	bool verifyChecksum( action_file_define::Page* page );
	void setChecksum( action_file_define::Page* page );

	int convertRadTow4095(double rad);
	double convertw4095ToRad(int w4095);

	void publishStatusMsg(unsigned int type, std::string msg);

	bool IsRunningServiceCallback(thormang3_action_module_msgs::IsRunning::Request  &req,
	                              thormang3_action_module_msgs::IsRunning::Response &res);

    void pageNumberCallback(const std_msgs::Int32::ConstPtr& msg);

	std::string convertIntToString(int n);

	bool previous_enable_;
	bool present_enable_;
	bool previous_running_;
	bool present_running_;

//////////////////////////////////////////////////////////////
public:
    ActionModule();
    virtual ~ActionModule();

    void    initialize(const int control_cycle_msec, robotis_framework::Robot *robot);
    void    process(std::map<std::string, robotis_framework::Dynamixel *> dxls, std::map<std::string, double> sensors);

    void	stop();
    bool	isRunning();

	bool loadFile(std::string file_name);
	bool createFile(std::string file_name);

	bool start(int page_number);
	bool start(std::string page_name);
	bool start(int page_number, action_file_define::Page* page);

    void OnModuleEnable();
    void OnModuleDisable();

	void brake();
	bool isRunning(int* playing_page_num, int* playing_step_num);
    bool loadPage(int page_number, action_file_define::Page* page);
    bool savePage(int page_number, action_file_define::Page* page);
    void resetPage(action_file_define::Page* page);

	void actionPlayProcess(std::map<std::string, robotis_framework::Dynamixel *> dxls);
};

}



#endif /* THORMANG3_ACTION_MOTION_MODULE_H_ */
