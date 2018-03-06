//=================================================================================================
// Copyright (c) 2018, Alexander Stumpf, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#ifndef L3_MODULE_H_
#define L3_MODULE_H_

#include <ros/ros.h>

#include <boost/thread.hpp>
#include <ros/callback_queue.h>
#include <std_msgs/Int16.h>

// robotis
#include <robotis_framework_common/motion_module.h>

// l3
#include <l3_walk_controller/walk_controller.h>

#include <thor_mang_l3_plugins/robotis_sensor_module.h>



namespace thormang3
{
class L3Module
  : public robotis_framework::Singleton<L3Module>
  , public robotis_framework::MotionModule
{
public:
  L3Module();
  virtual ~L3Module();

  void onModuleEnable() override;
  void onModuleDisable() override {}

  void initialize(const int control_cycle_msec, robotis_framework::Robot* robot) override;

  void process(std::map<std::string, robotis_framework::Dynamixel*> dxls, std::map<std::string, double> sensors) override;

  void stop() override;
  bool isRunning() override;

  bool gazebo_mode_;

private:
  void controllerManagerThread();
  void queueThread();

  int control_cycle_msec_;
  boost::thread queue_thread_;
  ros::Time last_time_stamp_;

  boost::mutex l3_mutex_;

  ros::NodeHandle queue_nh_;

  bool is_running_;
  std::vector<std::string> joint_names_;

  /** L3 */
  l3::WalkController::Ptr walk_controller_;
  std::vector<thormang3::RobotisSensorModule::Ptr> sensor_modules_;
};
}

#endif

