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

#ifndef WALK_CONTROL_MODULE_H_
#define WALK_CONTROL_MODULE_H_

#include <ros/ros.h>

#include <boost/noncopyable.hpp>

#include <boost/thread.hpp>
#include <ros/callback_queue.h>

// robotis
#include <thormang3_walking_module/WalkingModule.h>

// vigir walk control
#include <vigir_walk_control/walk_controller.h>



namespace thormang3
{
using namespace ROBOTIS;

class WalkControlModule
  : public WalkingMotionModule
  , private Singleton<WalkControlModule>
{
public:
  WalkControlModule();

  ~WalkControlModule();

  static WalkControlModule* GetInstance()
  {
     Singleton<WalkControlModule>::GetInstance();
  }

  void Initialize(const int control_cycle_msec, Robot* robot) override;

  void Process(std::map<std::string, Dynamixel*> dxls, std::map<std::string, double> sensors) override;

private:
  void QueueThread();

  int control_cycle_msec_;
  boost::thread queue_thread_;
  boost::mutex walk_control_mutex_;

  vigir_walk_control::WalkController::Ptr walk_controller_;
};
}

#endif
