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
 * thormang3_manager.cpp
 *
 *  Created on: 2016. 1. 21.
 *      Author: zerom
 */

#include "robotis_controller/robotis_controller.h"

/* Sensor Module Header */
#include "thormang3_imu_module/imu_sensor_module.h"
#include "thormang3_diagnostic_module/diagnostic_sensor_module.h"
#include "thormang3_feet_ft_module/feet_force_torque_sensor_module.h"
#include "thormang3_wrist_ft_module/wrist_force_torque_sensor_module.h"

/* Motion Module Header */
#include "thormang3_base_module/base_module.h"
#include "thormang3_action_module/action_module.h"
#include "thormang3_head_control_module/head_control_module.h"
#include "thormang3_manipulation_module/manipulation_module.h"
#include "thormang3_walking_module/walking_module.h"
#include "thormang3_ros_control_module/ros_control_module.h"
#include "thormang3_step_control_module/step_control_module.h"


using namespace thormang3;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "THORMANG3_Manager");
    ros::NodeHandle nh;

    ROS_INFO("manager->init");
    robotis_framework::RobotisController *controller = robotis_framework::RobotisController::getInstance();

    /* Load ROS Parameter */
    std::string offset_file = nh.param<std::string>("offset_file_path", "");
    std::string robot_file  = nh.param<std::string>("robot_file_path", "");

    std::string init_file   = nh.param<std::string>("init_file_path", "");

    std::string gazebo_robot_name = nh.param<std::string>("gazebo_robot_name", controller->gazebo_robot_name_);
    controller->gazebo_robot_name_ = gazebo_robot_name;
    ImuSensor::getInstance()->gazebo_robot_name_ = gazebo_robot_name;
    FeetForceTorqueSensor::getInstance()->gazebo_robot_name_ = gazebo_robot_name;
    WristForceTorqueSensor::getInstance()->gazebo_robot_name_ = gazebo_robot_name;

    /* gazebo simulation */
    if(nh.param<bool>("gazebo", false))
    {
        controller->gazebo_mode_ = true;
        ImuSensor::getInstance()->gazebo_mode_ = true;
        FeetForceTorqueSensor::getInstance()->gazebo_mode_ = true;
        WristForceTorqueSensor::getInstance()->gazebo_mode_ = true;
        StepControlModule::getInstance()->gazebo_mode_ = true;
        ROS_WARN("SET TO GAZEBO MODE!");
    }

    if(robot_file == "")
    {
        ROS_ERROR("NO robot file path in the ROS parameters.");
        return -1;
    }

    if(controller->initialize(robot_file, init_file) == false)
    {
        ROS_ERROR("ROBOTIS Controller Initialize Fail!");
        return -1;
    }

    if(offset_file != "")
    {
      controller->offset_config_path_ = offset_file;
      controller->loadOffsets(offset_file);
    }

    sleep(1);

    /* Add Sensor Module */
    controller->addSensorModule((robotis_framework::SensorModule*)ImuSensor::getInstance());
    controller->addSensorModule((robotis_framework::SensorModule*)FeetForceTorqueSensor::getInstance());
    controller->addSensorModule((robotis_framework::SensorModule*)WristForceTorqueSensor::getInstance());
    controller->addSensorModule((robotis_framework::SensorModule*)DiagnosticSensor::getInstance());

    /* Add Motion Module */
    controller->addMotionModule((robotis_framework::MotionModule*)BaseModule::getInstance());
    controller->addMotionModule((robotis_framework::MotionModule*)ActionModule::getInstance());
    controller->addMotionModule((robotis_framework::MotionModule*)ManipulationModule::getInstance());
    controller->addMotionModule((robotis_framework::MotionModule*)HeadControlModule::getInstance());
    //controller->addMotionModule((robotis_framework::MotionModule*)OnlineWalkingModule::getInstance()); // Encaspulated by WalkControlModule
    controller->addMotionModule((robotis_framework::MotionModule*)StepControlModule::getInstance());
    controller->addMotionModule((robotis_framework::MotionModule*)RosControlModule::getInstance());

    controller->startTimer();

    while(ros::ok())
    {
      usleep(1000*1000);
    }

    return 0;
}
