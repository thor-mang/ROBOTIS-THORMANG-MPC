//=================================================================================================
// Copyright (c) 2016, Alexander Stumpf, TU Darmstadt
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

#ifndef STEP_CONTROL_MODULE_H_
#define STEP_CONTROL_MODULE_H_

#include <ros/ros.h>

#include <boost/noncopyable.hpp>

#include <boost/thread.hpp>
#include <ros/callback_queue.h>

// robotis
#include <thormang3_walking_module/walking_module.h>

// vigir walk control
#include <vigir_step_control/step_controller.h>

// dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <thormang3_step_control_module/BalanceParametersConfig.h>



namespace thormang3
{
class StepControlModule
  : public WalkingMotionModule
  , private robotis_framework::Singleton<StepControlModule>
{
public:
  StepControlModule();

  ~StepControlModule();

  static StepControlModule* getInstance()
  {
     Singleton<StepControlModule>::getInstance();
  }

  void onModuleEnable() override;

  void initialize(const int control_cycle_msec, robotis_framework::Robot* robot) override;

  void process(std::map<std::string, robotis_framework::Dynamixel*> dxls, std::map<std::string, double> sensors) override;

  bool gazebo_mode_;

private:
  void queueThread();
  void dynamicReconfigureCallback(thormang3_step_control_module::BalanceParametersConfig& config, uint32_t level);

  void setBalanceParams(thormang3_walking_module_msgs::SetBalanceParam::Request& req);

  int control_cycle_msec_;
  boost::thread queue_thread_;
  boost::mutex step_control_mutex_;

  vigir_step_control::StepController::Ptr step_controller_;  

  thormang3_walking_module_msgs::SetBalanceParam::Request balance_params_;
};
}

#endif

