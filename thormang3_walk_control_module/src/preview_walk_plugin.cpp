#include <thormang3_walk_control_module/preview_walk_plugin.h>

#include <thormang3_walking_module/PreviewControlWalking.h>

#include <thormang3_walking_module/WalkingModuleCommon.h>



namespace thormang3
{
using namespace vigir_footstep_planning;

PreviewWalkPlugin::PreviewWalkPlugin()
  : WalkControllerPlugin()
{
}

PreviewWalkPlugin::~PreviewWalkPlugin()
{
}

void PreviewWalkPlugin::setStepPlanMsgPlugin(StepPlanMsgPlugin::Ptr plugin)
{
  WalkControllerPlugin::setStepPlanMsgPlugin(plugin);

  boost::unique_lock<boost::shared_mutex> lock(plugin_mutex_);

  thor_mang_step_plan_msg_plugin_ = boost::dynamic_pointer_cast<ThorMangStepPlanMsgPlugin>(plugin);

  if (!thor_mang_step_plan_msg_plugin_)
    ROS_ERROR("[PreviewWalkPlugin] StepPlanMsgPlugin is not from type 'ThorMangStepPlanMsgPlugin'!");
}

void PreviewWalkPlugin::updateStepPlan(const msgs::StepPlan& step_plan)
{
  if (step_plan.steps.empty())
    return;

  // transform initial step plan (afterwards stitching will do that automatically for us)
  if (step_queue_->empty())
  {
    const msgs::Step& step = step_plan.steps.front();

    ROBOTIS::StepData ref_step;
    initStepData(ref_step);
    /// TODO: use robotis reference step here?
    //ROBOTIS::PreviewControlWalking::GetInstance()->GetReferenceStepDatafotAddition(&ref_step);

    geometry_msgs::Pose ref_pose;
    if (step.foot.foot_index == msgs::Foot::LEFT)
      thor_mang_footstep_planning::toRos(ref_step.PositionData.stLeftFootPosition, ref_pose);
    else if (step.foot.foot_index == msgs::Foot::RIGHT)
      thor_mang_footstep_planning::toRos(ref_step.PositionData.stRightFootPosition, ref_pose);
    else
    {
      ROS_ERROR("[PreviewWalkPlugin] updateStepPlan: First step of input step plan has unknown foot index.");
      return;
    }

    // determine transformation to robotis frame
    tf::Transform transform = vigir_footstep_planning::StepPlan::getTransform(step.foot.pose, ref_pose);

    msgs::StepPlan step_plan_transformed = step_plan;
    vigir_footstep_planning::StepPlan::transformStepPlan(step_plan_transformed, transform);

    WalkControllerPlugin::updateStepPlan(step_plan_transformed);
  }
  else
  {
    /// TODO: Handle reverse spooling correctly

    WalkControllerPlugin::updateStepPlan(step_plan);
  }
}

void PreviewWalkPlugin::initWalk()
{
  ROBOTIS::PreviewControlWalking* prev_walking = ROBOTIS::PreviewControlWalking::GetInstance();

  if (prev_walking->IsRunning())
  {
    ROS_INFO("[PreviewWalkPlugin] Can't start walking as walking engine is still running. This is likely a bug and should be fixed immediately!");
    setState(FAILED);
    return;
  }

  prev_walking->Initialize();
  //prev_walking->SetInitialPose();
  //prev_walking->SetInitalWaistYawAngle();

  int min_steps_in_queue = 2;
  
  // initialize with +min_steps_in_queue that will cause trigger to fill step data of walking engine in preProcess() step
  last_remaining_unreserved_steps_ = prev_walking->GetNumofRemainingUnreservedStepData() + min_steps_in_queue;

  // init feedback states
  msgs::ExecuteStepPlanFeedback feedback;
  feedback.header.stamp = ros::Time::now();
  feedback.last_performed_step_index = -min_steps_in_queue;
  feedback.currently_executing_step_index =-min_steps_in_queue+1;
  feedback.first_changeable_step_index = 0;
  setFeedbackState(feedback);

  setState(ACTIVE);

  prev_walking->Start();

  ROS_INFO("[PreviewWalkPlugin] Starting walking.");
}

void PreviewWalkPlugin::preProcess(const ros::TimerEvent& event)
{
  WalkControllerPlugin::preProcess(event);

  if (getState() != ACTIVE)
    return;

  ROBOTIS::PreviewControlWalking* prev_walking = ROBOTIS::PreviewControlWalking::GetInstance();

  // first check if walking engine is still running
  if (!prev_walking->IsRunning())
  {
    ROS_INFO("[PreviewWalkPlugin] Walking engine has stopped unexpectedly. This is likely a bug and should be fixed immediately!");
    setState(FAILED);
    return;
  }

  int remaining_unreserved_steps = prev_walking->GetNumofRemainingUnreservedStepData();

  int diff = last_remaining_unreserved_steps_ - remaining_unreserved_steps;
  last_remaining_unreserved_steps_ = remaining_unreserved_steps;

  if (remaining_unreserved_steps <= 1)
  {
    msgs::ExecuteStepPlanFeedback feedback = getFeedbackState();
    feedback.header.stamp = ros::Time::now();

    // step(s) has been performed recently
    if (diff > 0)
    {
      feedback.last_performed_step_index += diff;
      feedback.currently_executing_step_index += diff;
      setFeedbackState(feedback);
    }

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
        ROS_INFO("[PreviewWalkPlugin] Walking finished.");

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

bool PreviewWalkPlugin::executeStep(const msgs::Step& step)
{
  /// TODO: flush step
  ROBOTIS::PreviewControlWalking* prev_walking = ROBOTIS::PreviewControlWalking::GetInstance();
  
  // add initial step
  if (step.step_index == 0)
  {
    ROBOTIS::StepData step_data;
    initStepData(step_data);
    /// TODO: use robotis reference step here?
    //prev_walking->GetReferenceStepDatafotAddition(&_refStepData);
    if (!prev_walking->AddStepData(step_data))
    {
      ROS_INFO("[PreviewWalkPlugin] executeStep: Error while adding initial step.");
      return false;
    }
    last_step_data_ = step_data;
    
    // add final step
    step_data.PositionData.bMovingFoot = ROBOTIS::MovingFootFlag::NFootMove;
    step_data.TimeData.bWalkingState = ROBOTIS::WalkingStateFlag::InWalkingEnding;
    step_data.TimeData.dAbsStepTime += 2.0;
    if (!prev_walking->AddStepData(step_data))
    {
      ROS_INFO("[PreviewWalkPlugin] executeStep: Error while adding (temp) final step.");
      return false;
    }
  }
  else
  {
    // remove final step to be updated
    prev_walking->EraseLastStepData();
    
    /// TODO: use robotis reference step here?
    ROBOTIS::StepData step_data = last_step_data_;
    step_data << step;
    step_data.TimeData.dAbsStepTime += last_step_data_.TimeData.dAbsStepTime;
    if (!prev_walking->AddStepData(step_data))
    {
      ROS_INFO("[PreviewWalkPlugin] executeStep: Error while adding step %i.", step.step_index);
      return false;
    }
    last_step_data_ = step_data;
    
    // readd updated final step
    step_data.PositionData.bMovingFoot = ROBOTIS::MovingFootFlag::NFootMove;
    step_data.TimeData.bWalkingState = ROBOTIS::WalkingStateFlag::InWalkingEnding;
    step_data.TimeData.dAbsStepTime += 2.0;
    if (!prev_walking->AddStepData(step_data))
    {
      ROS_INFO("[PreviewWalkPlugin] executeStep: Error while adding (temp) final step.");
      return false;
    }
  }

  last_remaining_unreserved_steps_ = ROBOTIS::PreviewControlWalking::GetInstance()->GetNumofRemainingUnreservedStepData();

  msgs::ExecuteStepPlanFeedback feedback = getFeedbackState();
  feedback.first_changeable_step_index++;
  setFeedbackState(feedback);
  
  ROS_INFO("[PreviewWalkPlugin] Send step %i to walking engine.", step.step_index);
  return true;
}

void PreviewWalkPlugin::stop()
{
  WalkControllerPlugin::stop();

  /// TODO: Stop when both feet on ground

  ROBOTIS::PreviewControlWalking::GetInstance()->Stop();
}
} // namespace

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(thormang3::PreviewWalkPlugin, vigir_walk_control::WalkControllerPlugin)
