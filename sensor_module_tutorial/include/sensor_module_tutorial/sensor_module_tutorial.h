/*
 * sensor_module_tutorial.h
 *
 *  Created on: 2016. 4. 20.
 *      Author: zerom
 */

#ifndef SENSOR_MODULE_TUTORIAL_INCLUDE_SENSOR_MODULE_TUTORIAL_SENSOR_MODULE_TUTORIAL_H_
#define SENSOR_MODULE_TUTORIAL_INCLUDE_SENSOR_MODULE_TUTORIAL_SENSOR_MODULE_TUTORIAL_H_


#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <std_msgs/Int16.h>
#include <boost/thread.hpp>

#include "robotis_framework_common/SensorModule.h"

namespace ROBOTIS
{

class SensorModuleTutorial : public SensorModule
{
private:
    static SensorModuleTutorial *unique_instance_;

    int             control_cycle_msec_;
    boost::thread   queue_thread_;

    /* sample subscriber & publisher */
    ros::Subscriber sub1_;
    ros::Publisher  pub1_;

    SensorModuleTutorial();

    void QueueThread();

public:
    virtual ~SensorModuleTutorial();

    static SensorModuleTutorial *GetInstance() { return unique_instance_; }

    /* ROS Topic Callback Functions */
    void    TopicCallback(const std_msgs::Int16::ConstPtr &msg);

    void    Initialize(const int control_cycle_msec, Robot *robot);
    void    Process(std::map<std::string, Dynamixel *> dxls);
};

}


#endif /* SENSOR_MODULE_TUTORIAL_INCLUDE_SENSOR_MODULE_TUTORIAL_SENSOR_MODULE_TUTORIAL_H_ */
