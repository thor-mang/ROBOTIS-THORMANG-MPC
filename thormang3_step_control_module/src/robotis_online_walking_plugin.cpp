#include <thormang3_step_control_module/robotis_online_walking_plugin.h>

#include <thormang3_walking_module/thormang3_online_walking.h>

#include <thormang3_walking_module/walking_module.h>



namespace thormang3
{
using namespace vigir_footstep_planning;

THORMANG3OnlineWalkingPlugin::THORMANG3OnlineWalkingPlugin()
  : StepControllerPlugin()
  , min_step_data_size(2)
{
}

THORMANG3OnlineWalkingPlugin::~THORMANG3OnlineWalkingPlugin()
{
}

void THORMANG3OnlineWalkingPlugin::setStepPlanMsgPlugin(StepPlanMsgPlugin::Ptr plugin)
{
  StepControllerPlugin::setStepPlanMsgPlugin(plugin);

  boost::unique_lock<boost::shared_mutex> lock(plugin_mutex_);

  thor_mang_step_plan_msg_plugin_ = boost::dynamic_pointer_cast<ThorMangStepPlanMsgPlugin>(plugin);

  if (!thor_mang_step_plan_msg_plugin_)
    ROS_ERROR("[THORMANG3OnlineWalkingPlugin] StepPlanMsgPlugin is not from type 'ThorMangStepPlanMsgPlugin'!");
}

bool THORMANG3OnlineWalkingPlugin::updateStepPlan(const msgs::StepPlan& step_plan)
{
  if (step_plan.steps.empty())
    return true;

  // transform initial step plan (afterwards stitching will do that automatically for us)
  if (step_queue_->empty())
  {
    const msgs::Step& step = step_plan.steps.front();

    robotis_framework::StepData ref_step;
    initStepData(ref_step);
    /// TODO: use robotis reference step here?
    //robotis_framework::THORMANG3OnlineWalking::GetInstance()->GetReferenceStepDatafotAddition(&ref_step);

    geometry_msgs::Pose ref_pose;
    if (step.foot.foot_index == msgs::Foot::LEFT)
      thor_mang_footstep_planning::toRos(ref_step.position_data.left_foot_pose, ref_pose);
    else if (step.foot.foot_index == msgs::Foot::RIGHT)
      thor_mang_footstep_planning::toRos(ref_step.position_data.right_foot_pose, ref_pose);
    else
    {
      ROS_ERROR("[THORMANG3OnlineWalkingPlugin] updateStepPlan: First step of input step plan has unknown foot index.");
      return false;
    }

    // determine transformation to robotis frame
    tf::Transform transform = vigir_footstep_planning::StepPlan::getTransform(step.foot.pose, ref_pose);

    msgs::StepPlan step_plan_transformed = step_plan;
    vigir_footstep_planning::StepPlan::transformStepPlan(step_plan_transformed, transform);

    return StepControllerPlugin::updateStepPlan(step_plan_transformed);
  }
  else
  {
    /// TODO: Handle reverse spooling correctly

    return StepControllerPlugin::updateStepPlan(step_plan);
  }

  return false;
}

void THORMANG3OnlineWalkingPlugin::initWalk()
{
  thormang3::THORMANG3OnlineWalking* online_walking = thormang3::THORMANG3OnlineWalking::getInstance();

  if (online_walking->isRunning())
  {
    ROS_ERROR("[THORMANG3OnlineWalkingPlugin] Can't start walking as walking engine is still running. This is likely a bug and should be fixed immediately!");
    setState(FAILED);
    return;
  }

  //online_walking->initialize();
  //online_walking->setInitialPose();
  //online_walking->setInitalWaistYawAngle();

  // init feedback states
  msgs::ExecuteStepPlanFeedback feedback;
  feedback.header.stamp = ros::Time::now();
  feedback.last_performed_step_index = -min_step_data_size;
  feedback.currently_executing_step_index =-min_step_data_size+1;
  feedback.first_changeable_step_index = 0;
  setFeedbackState(feedback);

  setState(ACTIVE);

  online_walking->start();

  ROS_INFO("[THORMANG3OnlineWalkingPlugin] Starting walking.");
}

void THORMANG3OnlineWalkingPlugin::preProcess(const ros::TimerEvent& event)
{
  StepControllerPlugin::preProcess(event);

  if (getState() != ACTIVE)
    return;

  thormang3::THORMANG3OnlineWalking* online_walking = thormang3::THORMANG3OnlineWalking::getInstance();

  // first check if walking engine is still running
  if (!online_walking->isRunning())
  {
    ROS_INFO("[THORMANG3OnlineWalkingPlugin] Walking engine has stopped unexpectedly. This is likely a bug and should be fixed immediately!");
    setState(FAILED);
    return;
  }

  //int remaining_unreserved_steps = online_walking->getNumofRemainingUnreservedStepData();
  int step_data_size = online_walking->getStepDataSize();
  int current_step_id = online_walking->getCurrentStepIdx();

  // remove closing step from count which is automatically added
  if (step_data_size > 0)
    step_data_size--;

  int diff = min_step_data_size - std::max(current_step_id, 0);

  ROS_WARN("Steps: %i Diff: %i Current Step: %i", step_data_size, diff, current_step_id);

  if (diff > 0)
  {
    msgs::ExecuteStepPlanFeedback feedback = getFeedbackState();
    feedback.header.stamp = ros::Time::now();

    // step(s) has been performed recently
    feedback.last_performed_step_index += diff;
    feedback.currently_executing_step_index += diff;
    setFeedbackState(feedback);

    // check if further steps in queue are available
    int steps_remaining = std::max(0, (step_queue_->lastStepIndex() - feedback.first_changeable_step_index) + 1);

    // steps are available
    if (steps_remaining > 0)
    {
      int steps = std::min(diff, steps_remaining);
      setNextStepIndexNeeded(getNextStepIndexNeeded()+steps);
    }
    // queue has been completely flushed out and executed
    else
    {
      // check for successful execution of queue
      if (step_queue_->lastStepIndex() == feedback.last_performed_step_index)
      {
        ROS_INFO("[THORMANG3OnlineWalkingPlugin] Walking finished.");

        feedback.currently_executing_step_index = -1;
        feedback.first_changeable_step_index = -1;
        setFeedbackState(feedback);

        step_queue_->reset();
        updateQueueFeedback();

        setState(FINISHED);
      }
    }
  }
}

bool THORMANG3OnlineWalkingPlugin::executeStep(const msgs::Step& step)
{
  /// TODO: flush step
  thormang3::THORMANG3OnlineWalking* online_walking = thormang3::THORMANG3OnlineWalking::getInstance();

  // add initial step
  if (step.step_index == 0)
  {
    robotis_framework::StepData step_data;
    initStepData(step_data);
    /// TODO: use robotis reference step here?
    //online_walking->getReferenceStepDatafotAddition(&_refStepData);
    if (!online_walking->addStepData(step_data))
    {
      ROS_INFO("[THORMANG3OnlineWalkingPlugin] executeStep: Error while adding initial step.");
      return false;
    }
    last_step_data_ = step_data;

    // add final step
    step_data.position_data.foot_z_swap = 0.0;
    step_data.position_data.body_z_swap = 0.0;
    step_data.position_data.moving_foot = thormang3_walking_module_msgs::StepPositionData::STANDING;
    step_data.time_data.walking_state = thormang3_walking_module_msgs::StepTimeData::IN_WALKING_ENDING;
    step_data.time_data.abs_step_time += 1.0;
    if (!online_walking->addStepData(step_data))
    {
      ROS_INFO("[THORMANG3OnlineWalkingPlugin] executeStep: Error while adding (temp) final step.");
      return false;
    }
  }
  else
  {
    // remove final step to be updated
    online_walking->eraseLastStepData();

    // add step
    /// TODO: use robotis reference step here?
    robotis_framework::StepData step_data = last_step_data_;
    step_data << step;
    if (!online_walking->addStepData(step_data))
    {
      ROS_INFO("[THORMANG3OnlineWalkingPlugin] executeStep: Error while adding step %i.", step.step_index);
      return false;
    }
    last_step_data_ = step_data;

    // readd updated final step
    //step_data.position_data.foot_z_swap = 0.0;
    step_data.position_data.body_z_swap = 0.0;
    step_data.position_data.moving_foot = thormang3_walking_module_msgs::StepPositionData::STANDING;
    step_data.time_data.walking_state = thormang3_walking_module_msgs::StepTimeData::IN_WALKING_ENDING;
    step_data.time_data.abs_step_time += 1.6;
    if (!online_walking->addStepData(step_data))
    {
      ROS_INFO("[THORMANG3OnlineWalkingPlugin] executeStep: Error while adding (temp) final step.");
      return false;
    }
  }

  msgs::ExecuteStepPlanFeedback feedback = getFeedbackState();
  feedback.first_changeable_step_index++;
  setFeedbackState(feedback);

  ROS_INFO("[THORMANG3OnlineWalkingPlugin] Send step %i to walking engine.", step.step_index);
  return true;
}

void THORMANG3OnlineWalkingPlugin::stop()
{
  StepControllerPlugin::stop();

  /// TODO: Stop when both feet on ground

  thormang3::THORMANG3OnlineWalking::getInstance()->stop();
}
} // namespace

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(thormang3::THORMANG3OnlineWalkingPlugin, vigir_step_control::StepControllerPlugin)

