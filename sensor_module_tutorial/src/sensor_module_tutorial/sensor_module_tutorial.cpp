/*******************************************************************************
 * Copyright (c) 2016, ROBOTIS CO., LTD.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of ROBOTIS nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

/*
 * sensor_module_tutorial.cpp
 *
 *  Created on: 2016. 4. 20.
 *      Author: zerom
 */

#include <stdio.h>
#include "sensor_module_tutorial/sensor_module_tutorial.h"

using namespace ROBOTIS;

SensorModuleTutorial::SensorModuleTutorial()
    : control_cycle_msec_(8)
{
    module_name_    = "test_sensor_module"; // set unique module name

    result_["test_sensor"] = 0.0;
}

SensorModuleTutorial::~SensorModuleTutorial()
{
    queue_thread_.join();
}

void SensorModuleTutorial::initialize(const int control_cycle_msec, robotis_framework::Robot *robot)
{
    control_cycle_msec_ = control_cycle_msec;
    queue_thread_       = boost::thread(boost::bind(&SensorModuleTutorial::queueThread, this));
}

void SensorModuleTutorial::queueThread()
{
    ros::NodeHandle     _ros_node;
    ros::CallbackQueue  _callback_queue;

    _ros_node.setCallbackQueue(&_callback_queue);

    /* subscriber */
    sub1_ = _ros_node.subscribe("/tutorial_topic", 10, &SensorModuleTutorial::topicCallback, this);

    /* publisher */
    pub1_ = _ros_node.advertise<std_msgs::Int16>("/tutorial_publish", 1, true);

    while(_ros_node.ok())
    {
        _callback_queue.callAvailable();
        usleep(100);
    }
}

void SensorModuleTutorial::topicCallback(const std_msgs::Int16::ConstPtr &msg)
{
    std_msgs::Int16 _msg;
    _msg.data = msg->data;
    pub1_.publish(_msg);
}

void SensorModuleTutorial::process(std::map<std::string, robotis_framework::Dynamixel *> dxls,
                                   std::map<std::string, robotis_framework::Sensor *> sensors)
{
    uint16_t ext_port_data_1 = dxls["r_leg_an_p"]->dxl_state_->bulk_read_table_["external_port_data_1"];
    uint16_t ext_port_data_2 = dxls["r_leg_an_p"]->dxl_state_->bulk_read_table_["external_port_data_2"];

    // ...

    result_["test_sensor"] = 0.0;
}


