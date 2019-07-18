#include <thormang3_step_control_module/robotis_online_walking_plugin.h>

#include <thormang3_walking_module/thormang3_online_walking.h>

#include <thormang3_walking_module/walking_module.h>



namespace thormang3
{
THORMANG3OnlineWalkingPlugin::THORMANG3OnlineWalkingPlugin()
  : StepControllerPlugin()
  , last_remaining_unreserved_steps_(0)
  , min_unreserved_steps_(2)
{
}

THORMANG3OnlineWalkingPlugin::~THORMANG3OnlineWalkingPlugin()
{
}

msgs::ErrorStatus THORMANG3OnlineWalkingPlugin::setStepPlanMsgPlugin(StepPlanMsgPlugin::Ptr plugin)
{
  msgs::ErrorStatus status = StepControllerPlugin::setStepPlanMsgPlugin(plugin);

  if (hasError(status))
    return status;

  UniqueLock lock(plugin_mutex_);

  thor_mang_step_plan_msg_plugin_ = vigir_pluginlib::cast<ThorMangStepPlanMsgPlugin>(plugin);
  if (!thor_mang_step_plan_msg_plugin_)
    status += ErrorStatusError(msgs::ErrorStatus::ERR_UNKNOWN, getName(), "Given StepPlanMsgPlugin is not from type 'ThorMangStepPlanMsgPlugin'!");

  return status;
}

msgs::ErrorStatus THORMANG3OnlineWalkingPlugin::updateStepPlan(const msgs::StepPlan& step_plan)
{
  if (step_plan.plan.steps.empty())
    return msgs::ErrorStatus();

  // transform initial step plan (afterwards stitching will do that automatically for us)
  if (step_queue_->empty())
  {
    const msgs::Step& step = step_plan.plan.steps.front();

    if (step.support.size() != 2)
      return ErrorStatusError(msgs::ErrorStatus::ERR_UNKNOWN, getName(), "updateStepPlan: First step of an initial step plan must contain 2 support legs.");
    else if (step.support[0].foot_idx != Foot::LEFT)
      return ErrorStatusError(msgs::ErrorStatus::ERR_UNKNOWN, getName(), "Initial step foothold with idx = 0 is not left foot!");
    else if (step.support[1].foot_idx != Foot::RIGHT)
      return ErrorStatusError(msgs::ErrorStatus::ERR_UNKNOWN, getName(), "Initial step foothold with idx = 1 is not right foot!");

    robotis_framework::StepData ref_step;
    initStepData(ref_step);
    /// TODO: use robotis reference step here?
    //robotis_framework::THORMANG3OnlineWalking::GetInstance()->GetReferenceStepDatafotAddition(&ref_step);

    l3::Pose ref_pose;
    toRos(ref_step.position_data.left_foot_pose, ref_pose);

    l3::Pose cur_pose;
    l3::poseMsgToL3(step.support[0].pose, cur_pose);

    // determine transformation to robotis frame
    Transform transform = ref_pose * cur_pose.inverse();
    msgs::StepPlan step_plan_transformed = l3_footstep_planning::StepPlan::transformStepPlan(step_plan, transform);

    return StepControllerPlugin::updateStepPlan(step_plan_transformed);
  }
  else
  {
    /// TODO: Handle reverse spooling correctly

    return StepControllerPlugin::updateStepPlan(step_plan);
  }
}

msgs::ErrorStatus THORMANG3OnlineWalkingPlugin::initWalk()
{
  thormang3::THORMANG3OnlineWalking* online_walking = thormang3::THORMANG3OnlineWalking::getInstance();

  if (online_walking->isRunning())
  {
    setState(FAILED);
    return ErrorStatusError(msgs::ErrorStatus::ERR_UNKNOWN, getName(), "Can't start walking as walking engine is still running. This is likely a bug and should be fixed immediately!");
  }

  //online_walking->initialize();
  //online_walking->setInitialPose();
  //online_walking->setInitalWaistYawAngle();

  last_remaining_unreserved_steps_ = 0;

  // init feedback states
  msgs::ExecuteStepPlanFeedback feedback;
  feedback.header.stamp = ros::Time::now();
  feedback.last_queued_step_idx = -1;
  feedback.currently_executing_step_idx = 0;
  feedback.first_changeable_step_idx = 0;
  setFeedbackState(feedback);

  setState(ACTIVE);

  online_walking->start();

  ROS_INFO("[%s] Starting walking.", getName().c_str());

  return msgs::ErrorStatus();
}

msgs::ErrorStatus THORMANG3OnlineWalkingPlugin::preProcess(const ros::TimerEvent& event)
{
  msgs::ErrorStatus status = StepControllerPlugin::preProcess(event);

  if (getState() != ACTIVE)
    return status;

  thormang3::THORMANG3OnlineWalking* online_walking = thormang3::THORMANG3OnlineWalking::getInstance();

  msgs::ExecuteStepPlanFeedback feedback = getFeedbackState();

  if (!online_walking->isRunning())
  {
    // queue has been completely flushed out and executed
    if (step_queue_->empty())
    {
      ROS_INFO("[%s] Walking finished.", getName().c_str());

      feedback.currently_executing_step_idx = -1;
      feedback.first_changeable_step_idx = -1;
      setFeedbackState(feedback);

      step_queue_->clear();
      updateQueueFeedback();

      setState(FINISHED);
      return status;
    }
    // walking engine has been stopped unexpectedly
    else
    {
      setState(FAILED);
      status += ErrorStatusError(msgs::ErrorStatus::ERR_UNKNOWN, getName(), "Walking engine has stopped unexpectedly. This is likely a bug and should be fixed immediately!");
      return status;
    }
  }

  // checking current state of walking engine
  int remaining_unreserved_steps = online_walking->getNumofRemainingUnreservedStepData();
  int executed_steps = std::max(last_remaining_unreserved_steps_ - remaining_unreserved_steps, 0);
  last_remaining_unreserved_steps_ = remaining_unreserved_steps;

  int needed_steps = min_unreserved_steps_ - remaining_unreserved_steps;

  // skip when final pseudo step is in execution in order terminating the walking controller
  if (step_queue_->empty() && remaining_unreserved_steps == 0)
    return status;

  // update step(s) which has been performed recently
  feedback.header.stamp = ros::Time::now();
  feedback.last_performed_step_idx += executed_steps;
  feedback.currently_executing_step_idx += executed_steps;

  if (needed_steps > 0)
  {
    // check if further steps in queue are available
    int steps_remaining = std::max(0, step_queue_->lastStepIndex() - getLastStepIndexSent());

    // steps are available
    if (steps_remaining > 0)
    {
      needed_steps = std::min(needed_steps, steps_remaining);

      setNextStepIndexNeeded(getNextStepIndexNeeded()+needed_steps);
      feedback.first_changeable_step_idx = getNextStepIndexNeeded()+1;
    }
  }

  setFeedbackState(feedback);

  return status;
}

msgs::ErrorStatus THORMANG3OnlineWalkingPlugin::executeStep(Step::ConstPtr step)
{
  /// TODO: flush step
  thormang3::THORMANG3OnlineWalking* online_walking = thormang3::THORMANG3OnlineWalking::getInstance();

  // add initial step
  if (step->getStepIndex() == 0)
  {
    robotis_framework::StepData step_data;
    initStepData(step_data);

    /// TODO: use robotis reference step here?
    //online_walking->getReferenceStepDatafotAddition(&_refStepData);
    if (!online_walking->addStepData(step_data))
      return ErrorStatusError(msgs::ErrorStatus::ERR_INVALID_STEP, getName(), "executeStep: Error while adding initial step " + boost::lexical_cast<std::string>(step->getStepIndex()) + ".");

    last_step_data_ = step_data;

    // add final step
    step_data.position_data.foot_z_swap = 0.0;
    step_data.position_data.body_z_swap = 0.0;
    step_data.position_data.moving_foot = thormang3_walking_module_msgs::StepPositionData::STANDING;
    step_data.time_data.walking_state = thormang3_walking_module_msgs::StepTimeData::IN_WALKING_ENDING;
    step_data.time_data.abs_step_time += 1.0;
    if (!online_walking->addStepData(step_data))
      return ErrorStatusError(msgs::ErrorStatus::ERR_INVALID_STEP, getName(), "executeStep: Error while adding (temp) final step.");
  }
  else
  {
    if (step->getStepDataMap().size() != 1)
      return ErrorStatusError(msgs::ErrorStatus::ERR_INVALID_STEP, getName(), "executeStep: Step " + boost::lexical_cast<std::string>(step->getStepIndex()) + " does not contain exactly 1 step data.");

    // get ref data
    robotis_framework::StepData ref_step_data;
    online_walking->getReferenceStepDatafotAddition(&ref_step_data);

    // remove final step to be updated
    online_walking->eraseLastStepData();

    // add step
    /// TODO: use robotis reference step here?
    robotis_framework::StepData step_data = last_step_data_;
    step_data << *step;
    /// TODO: compensate drift in z
    step_data.position_data.body_pose.z = ref_step_data.position_data.body_pose.z;
    if (step->getStepDataMap().begin()->second->target->foot_idx == Foot::LEFT)
      step_data.position_data.left_foot_pose.z = ref_step_data.position_data.right_foot_pose.z;
    else
      step_data.position_data.right_foot_pose.z = ref_step_data.position_data.left_foot_pose.z;

    if (!online_walking->addStepData(step_data))
      return ErrorStatusError(msgs::ErrorStatus::ERR_INVALID_STEP, getName(), "executeStep: Error while adding step " + boost::lexical_cast<std::string>(step->getStepIndex()) + ".");

    last_step_data_ = step_data;

    // readd updated final step
    //step_data.position_data.foot_z_swap = 0.0;
    step_data.position_data.body_z_swap = 0.0;
    step_data.position_data.moving_foot = thormang3_walking_module_msgs::StepPositionData::STANDING;
    step_data.time_data.walking_state = thormang3_walking_module_msgs::StepTimeData::IN_WALKING_ENDING;
    step_data.time_data.abs_step_time += 1.6;

    if (!online_walking->addStepData(step_data))
      return ErrorStatusError(msgs::ErrorStatus::ERR_INVALID_STEP, getName(), "executeStep: Error while adding (temp) final step.");
  }

  last_remaining_unreserved_steps_ = online_walking->getNumofRemainingUnreservedStepData();
  return msgs::ErrorStatus();
}

msgs::ErrorStatus THORMANG3OnlineWalkingPlugin::stop()
{
  /// TODO: Stop during DSP

  thormang3::THORMANG3OnlineWalking::getInstance()->stop();

  return StepControllerPlugin::stop();
}
} // namespace

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(thormang3::THORMANG3OnlineWalkingPlugin, l3_step_controller::StepControllerPlugin)

