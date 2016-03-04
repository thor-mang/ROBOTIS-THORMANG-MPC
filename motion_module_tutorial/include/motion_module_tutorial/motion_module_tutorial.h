/*
 * motion_module_tutorial.h
 *
 *  Created on: 2016. 2. 23.
 *      Author: zerom
 */

#ifndef ROBOTIS_THORMANG_MPC_MOTION_MODULE_TUTORIAL_INCLUDE_MOTION_MODULE_TUTORIAL_MOTION_MODULE_TUTORIAL_H_
#define ROBOTIS_THORMANG_MPC_MOTION_MODULE_TUTORIAL_INCLUDE_MOTION_MODULE_TUTORIAL_MOTION_MODULE_TUTORIAL_H_



#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <std_msgs/Int16.h>
#include <boost/thread.hpp>

#include "robotis_framework_common/MotionModule.h"

namespace ROBOTIS
{

class MotionModuleTutorial : public MotionModule
{
private:
    static MotionModuleTutorial *unique_instance_;

    int             control_cycle_msec_;
    boost::thread   queue_thread_;

    /* sample subscriber & publisher */
    ros::Subscriber sub1_;
    ros::Publisher  pub1_;

    MotionModuleTutorial();

    void QueueThread();

public:
    virtual ~MotionModuleTutorial();

    static MotionModuleTutorial *GetInstance() { return unique_instance_; }

    /* ROS Topic Callback Functions */
    void    TopicCallback(const std_msgs::Int16::ConstPtr &msg);

    void    Initialize(const int control_cycle_msec);
    void    Process(std::map<std::string, Dynamixel *> dxls);
};

}



#endif /* ROBOTIS_THORMANG_MPC_MOTION_MODULE_TUTORIAL_INCLUDE_MOTION_MODULE_TUTORIAL_MOTION_MODULE_TUTORIAL_H_ */
