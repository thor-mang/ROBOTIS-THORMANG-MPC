#include <thormang3_walk_control_module/preview_walk_plugin.h>



namespace thormang3
{
PreviewWalkPlugin::PreviewWalkPlugin()
  : WalkControllerPlugin()
{
}

PreviewWalkPlugin::~PreviewWalkPlugin()
{
}

void PreviewWalkPlugin::initWalk()
{
  // init feedback
  msgs::ExecuteStepPlanFeedback feedback;
  feedback.header.stamp = ros::Time::now();
  feedback.last_performed_step_index = -2;
  feedback.currently_executing_step_index =-1;
  feedback.first_changeable_step_index = 0;
  setFeedbackState(feedback);

  next_step_needed_time_ = ros::Time::now();

  ROS_INFO("[PreviewWalkPlugin] Start fake execution.");
}

void PreviewWalkPlugin::preProcess(const ros::TimerEvent& event)
{
  WalkControllerPlugin::preProcess(event);

  if (getState() != ACTIVE)
    return;

  // fake succesful execution of single step
  if (next_step_needed_time_ <= ros::Time::now())
  {
    msgs::ExecuteStepPlanFeedback feedback = getFeedbackState();

    feedback.header.stamp = ros::Time::now();
    feedback.last_performed_step_index++;

    // check for succesful execution of queue
    if (step_queue_->lastStepIndex() == feedback.last_performed_step_index)
    {
      ROS_INFO("[PreviewWalkPlugin] Fake execution finished.");

      feedback.currently_executing_step_index = -1;
      feedback.first_changeable_step_index = -1;
      setFeedbackState(feedback);

      step_queue_->reset();
      updateQueueFeedback();

      setState(FINISHED);
    }
    // otherwise trigger fake execution of next step
    else
    {
      feedback.currently_executing_step_index++;
      feedback.first_changeable_step_index++;
      setFeedbackState(feedback);

      setNextStepIndexNeeded(feedback.currently_executing_step_index);
    }
  }
}

bool PreviewWalkPlugin::executeStep(const msgs::Step& step)
{
  next_step_needed_time_ = ros::Time::now() + ros::Duration(1.0 + step.step_duration);
  ROS_INFO("[PreviewWalkPlugin] Fake execution of step %i", step.step_index);
  return true;
}
} // namespace

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(thormang3::PreviewWalkPlugin, vigir_walk_control::WalkControllerPlugin)
