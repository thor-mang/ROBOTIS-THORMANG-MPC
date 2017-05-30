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

#ifndef THORMANG3_DIAGNOSTIC_SENSOR_MODULE_H_
#define THORMANG3_DIAGNOSTIC_SENSOR_MODULE_H_

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <boost/thread.hpp>

#include <sensor_msgs/Imu.h>

#include <robotis_framework_common/sensor_module.h>

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>



namespace thormang3
{
class DiagnosticSensor
  : public robotis_framework::SensorModule
  , public robotis_framework::Singleton<DiagnosticSensor>
{
//  class DynamixelFunctionDiagnosticTask
//   : public diagnostic_updater::FunctionDiagnosticTask
//  {

//    DynamixelFunctionDiagnosticTask(const std::string &name, boost::function<void(T&)> fn) :

//  };
protected:
  class DynamixelDiagnosticTask
    : public diagnostic_updater::DiagnosticTask
  {
    public:
      DynamixelDiagnosticTask(const std::string name, robotis_framework::Dynamixel* dxl)
        : DiagnosticTask(name)
        , name_(name)
        , dxl_(dxl)
    {}

      void run(diagnostic_updater::DiagnosticStatusWrapper &stat) override
      {
        stat.name = name_;

        // do awesome stuff here

        stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "node is ok");
        stat.add("id", dxl_->id_);
        stat.add("port", dxl_->port_name_);
        stat.add("dxl_state_ ", dxl_->dxl_state_);
        stat.add("current_position", dxl_->present_position_item_ );
        stat.add("velocity",dxl_->present_velocity_item_);
        stat.add("current", dxl_->present_current_item_);
        stat.add("torque_enable_item_",dxl_->torque_enable_item_);
        stat.add("position_p_gain_item_",dxl_->position_p_gain_item_);
        stat.add("position_i_gain_item_",dxl_->position_i_gain_item_);
        stat.add("position_d_gain_item_",dxl_->position_d_gain_item_);
        stat.add("velocity_p_gain_item_",dxl_->velocity_p_gain_item_);
        stat.add("velocity_i_gain_item_",dxl_->velocity_i_gain_item_);
        stat.add("velocity_d_gain_item_",dxl_->velocity_d_gain_item_);

      }

    private:
      const std::string name_;
      robotis_framework::Dynamixel* dxl_;
  };

public:
  DiagnosticSensor();
  ~DiagnosticSensor();

  void initialize(const int control_cycle_msec, robotis_framework::Robot* robot) override;
  void process(std::map<std::string, robotis_framework::Dynamixel* > dxls, std::map<std::string, robotis_framework::Sensor*> sensors) override;

  void stop();
  bool isRunning();

  bool gazebo_mode_;
  std::string gazebo_robot_name_;
  boost::shared_ptr<diagnostic_updater::Updater> diagnostic_updater_;

private:
  void msgQueueThread();

  int control_cycle_msec_;
  boost::thread queue_thread_;

  diagnostic_updater::Updater updater_;
};
}


#endif /* THORMANG3_DIAGNOSTIC_SENSOR_MODULE_H_ */
