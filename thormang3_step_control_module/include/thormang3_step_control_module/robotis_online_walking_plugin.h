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

#ifndef ROBOTIS_ONLINE_WALKING_PLUGIN_H__
#define ROBOTIS_ONLINE_WALKING_PLUGIN_H__

#include <ros/ros.h>

#include <vigir_step_control/step_controller_plugin.h>

#include <thor_mang_footstep_planning_plugins/thor_mang_step_plan_msg_plugin.h>



namespace thormang3
{
using namespace vigir_footstep_planning;
using namespace vigir_footstep_planning_msgs;
using namespace vigir_step_control;
using namespace thor_mang_footstep_planning;

class THORMANG3OnlineWalkingPlugin
  : public StepControllerPlugin
{
public:
  // typedefs
  typedef boost::shared_ptr<THORMANG3OnlineWalkingPlugin> Ptr;
  typedef boost::shared_ptr<const THORMANG3OnlineWalkingPlugin> ConstPtr;

  THORMANG3OnlineWalkingPlugin();
  virtual ~THORMANG3OnlineWalkingPlugin();

  /**
   * @brief Sets the StepPlanMsgPlugin to be used.
   * @param plugin Plugin of type StepPlanMsgPlugin
   */
  void setStepPlanMsgPlugin(StepPlanMsgPlugin::Ptr plugin) override;

  /**
   * @brief Merges given step plan to the current step queue of steps. Hereby, two cases have to considered:
   * 1. In case of an empty step queue (robot is standing) the step plan has to begin with step index 0.
   * 2. In case of an non-empty step queue (robot is walking) the first step of the step plan has to be
   * identical with the corresponding step (=same step index) in the step queue. Be aware that already performed
   * steps have been popped from step queue and therefore are not exisiting anymore.
   * The default implementation resets the plugin previously when in FINISHED or FAILED state.
   * @param step_plan Step plan to be merged into step queue.
   * @return false if an error has occured
   */
  bool updateStepPlan(const msgs::StepPlan& step_plan) override;

  /**
   * @brief This method is called when new step plan has been enqueued and previously the walk controller state was IDLE.
   */
  void initWalk() override;

  /**
   * @brief PreProcess Method is called before processing walk controller, e.g. for precompute/update data or check walking engine status.
   * For keeping the default behavior running, following variables has to be properly updated here:
   * - feedback.first_changeable_step_index
   * - next_step_index_needed
   * - setState(FINISEHD) when execution of all steps were successfully completed
   * - setState(FAILED) when an error has occured
   */
  void preProcess(const ros::TimerEvent& event) override;

  /**
   * @brief This method will be called when the next step should be added to execution pipeline. The call of this function should
   * be triggered by the process(...) method when nextStepIndexNeeded has been changed.
   * @param step Step to be executed now
   */
  bool executeStep(const msgs::Step& step) override;

  /**
   * @brief Will be called when (soft) stop is requested and resets plugin.
   */
  void stop() override;

protected:
  ThorMangStepPlanMsgPlugin::ConstPtr thor_mang_step_plan_msg_plugin_;
  robotis_framework::StepData last_step_data_;

  int last_remaining_unreserved_steps_;
};
}

#endif
