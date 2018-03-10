#include <thormang3_step_control_module/step_control_module.h>

#include <vigir_generic_params/parameter_manager.h>
#include <vigir_pluginlib/plugin_manager.h>



namespace thormang3
{
StepControlModule::StepControlModule()
  : OnlineWalkingModule()
  , gazebo_mode_(false)
{
  enable_ = false;
  module_name_ = "walking_module"; // set unique module name
  control_mode_ = robotis_framework::PositionControl;
}

StepControlModule::~StepControlModule()
{
  queue_thread_.join();
}

void StepControlModule::onModuleEnable()
{
  step_control_mutex_.lock();

  // (re)initialize parameter and plugin manager
  vigir_generic_params::ParameterManager::initialize(queue_nh_);
  vigir_pluginlib::PluginManager::initialize(queue_nh_);

  // start walk controller
  step_controller_.reset(new vigir_step_control::StepController(queue_nh_));
  step_controller_->initialize(queue_nh_, false);

  step_control_mutex_.unlock();

  setBalanceParams(balance_params_);
}

void StepControlModule::initialize(const int control_cycle_msec, robotis_framework::Robot* robot)
{
  boost::mutex::scoped_lock lock(step_control_mutex_);

  // init basic walking
  OnlineWalkingModule::initialize(control_cycle_msec, robot);

  control_cycle_msec_ = control_cycle_msec;

  /** start ros thread */
  queue_thread_ = boost::thread(boost::bind(&StepControlModule::queueThread, this));
}

void StepControlModule::process(std::map<std::string, robotis_framework::Dynamixel*> dxls, std::map<std::string, double> sensors)
{
  boost::mutex::scoped_lock lock(step_control_mutex_);

  if (step_controller_)
    step_controller_->update();

  // finally run low level process
  OnlineWalkingModule::process(dxls, sensors);
}

void StepControlModule::queueThread()
{
  step_control_mutex_.lock();

  queue_nh_ = ros::NodeHandle("step_control_module");
  ros::CallbackQueue callback_queue;
  queue_nh_.setCallbackQueue(&callback_queue);

  // dynamic reconfigure
  dynamic_reconfigure::Server<thormang3_step_control_module::BalanceParametersConfig> server(queue_nh_);
  dynamic_reconfigure::Server<thormang3_step_control_module::BalanceParametersConfig>::CallbackType callback_f;

  step_control_mutex_.unlock();

  callback_f = boost::bind(&StepControlModule::dynamicReconfigureCallback, this, _1, _2);
  server.setCallback(callback_f);

  ros::WallDuration duration(control_cycle_msec_/1000.0);
  while(queue_nh_.ok())
    callback_queue.callAvailable(duration);
}

void StepControlModule::dynamicReconfigureCallback(thormang3_step_control_module::BalanceParametersConfig& config, uint32_t /*level*/)
{
  boost::mutex::scoped_lock lock(step_control_mutex_);

  balance_params_ = thormang3_walking_module_msgs::SetBalanceParam::Request();

  balance_params_.updating_duration = 2.0;

  // ####### cob_offset #######
  balance_params_.balance_param.cob_x_offset_m = config.cob_x_offset_m;
  balance_params_.balance_param.cob_y_offset_m = config.cob_y_offset_m;

  // ####### FeedForward #####
  balance_params_.balance_param.hip_roll_swap_angle_rad = config.hip_roll_swap_angle_rad;

  // in Gazebo mode these are kept zero (force disabling balancing)
  if (config.balance && !gazebo_mode_)
  {
    // ########## Gains ########
    // gyro
    balance_params_.balance_param.foot_roll_gyro_p_gain = config.foot_roll_gyro_p_gain;
    balance_params_.balance_param.foot_roll_gyro_d_gain = config.foot_roll_gyro_d_gain;
    balance_params_.balance_param.foot_pitch_gyro_p_gain = config.foot_pitch_gyro_p_gain;
    balance_params_.balance_param.foot_pitch_gyro_d_gain = config.foot_pitch_gyro_d_gain;

    // imu
    balance_params_.balance_param.foot_roll_angle_p_gain = config.foot_roll_angle_p_gain;
    balance_params_.balance_param.foot_roll_angle_d_gain = config.foot_roll_angle_d_gain;
    balance_params_.balance_param.foot_pitch_angle_p_gain = config.foot_pitch_angle_p_gain;
    balance_params_.balance_param.foot_pitch_angle_d_gain = config.foot_pitch_angle_d_gain;

    // ft
    balance_params_.balance_param.foot_x_force_p_gain = config.foot_x_force_p_gain;
    balance_params_.balance_param.foot_x_force_d_gain = config.foot_x_force_d_gain;
    balance_params_.balance_param.foot_y_force_p_gain = config.foot_y_force_p_gain;
    balance_params_.balance_param.foot_y_force_d_gain = config.foot_y_force_d_gain;
    balance_params_.balance_param.foot_z_force_p_gain = config.foot_z_force_p_gain;
    balance_params_.balance_param.foot_z_force_d_gain = config.foot_z_force_d_gain;

    balance_params_.balance_param.foot_roll_torque_p_gain = config.foot_roll_torque_p_gain;
    balance_params_.balance_param.foot_roll_torque_d_gain = config.foot_roll_torque_d_gain;
    balance_params_.balance_param.foot_pitch_torque_p_gain = config.foot_pitch_torque_p_gain;
    balance_params_.balance_param.foot_pitch_torque_d_gain = config.foot_pitch_torque_d_gain;

    // ########## CUT OFF FREQUENCY ##########
    // gyro
    balance_params_.balance_param.roll_gyro_cut_off_frequency = config.roll_gyro_cut_off_frequency;
    balance_params_.balance_param.pitch_gyro_cut_off_frequency = config.pitch_gyro_cut_off_frequency;
 
    // imu
    balance_params_.balance_param.roll_angle_cut_off_frequency = config.roll_angle_cut_off_frequency;
    balance_params_.balance_param.pitch_angle_cut_off_frequency = config.pitch_angle_cut_off_frequency;

    // ft
    balance_params_.balance_param.foot_x_force_cut_off_frequency = config.foot_x_force_cut_off_frequency;
    balance_params_.balance_param.foot_y_force_cut_off_frequency = config.foot_y_force_cut_off_frequency;
    balance_params_.balance_param.foot_z_force_cut_off_frequency = config.foot_z_force_cut_off_frequency;

    balance_params_.balance_param.foot_roll_torque_cut_off_frequency = config.foot_roll_torque_cut_off_frequency;
    balance_params_.balance_param.foot_pitch_torque_cut_off_frequency = config.foot_pitch_torque_cut_off_frequency;
  }

  setBalanceParams(balance_params_);
}

void StepControlModule::setBalanceParams(thormang3_walking_module_msgs::SetBalanceParam::Request& req)
{
  thormang3_walking_module_msgs::SetBalanceParam::Response resp;

  if (OnlineWalkingModule::setBalanceParamServiceCallback(req, resp))
  {
    switch (resp.result)
    {
      case thormang3_walking_module_msgs::SetBalanceParam::Response::NOT_ENABLED_WALKING_MODULE:
        ROS_WARN("[StepControlModule] Walking module disabled, balance parameter are not set.");
        break;
      case thormang3_walking_module_msgs::SetBalanceParam::Response::PREV_REQUEST_IS_NOT_FINISHED:
        ROS_ERROR("[StepControlModule] Previous request is still pending. Can't set new parameters!");
        break;
      default:
        ROS_INFO("[StepControlModule] Set new balance parameters.");
        break;
    }
  }
  else
    ROS_ERROR("[StepControlModule] Couldn't call balance parameter service!");
}
}
