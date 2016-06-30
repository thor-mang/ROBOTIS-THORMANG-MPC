#include <thormang3_walk_control_module/walk_control_module.h>

#include <vigir_generic_params/parameter_manager.h>
#include <vigir_pluginlib/plugin_manager.h>



namespace thormang3
{
WalkControlModule::WalkControlModule()
  : WalkingMotionModule()
  , control_cycle_msec_(8)
{
  enable = false;
  module_name = "walk_control_module"; // set unique module name
  control_mode = POSITION_CONTROL;
}

WalkControlModule::~WalkControlModule()
{
  queue_thread_.join();
}

void WalkControlModule::Initialize(const int control_cycle_msec, Robot* robot)
{
  boost::mutex::scoped_lock lock(walk_control_mutex_);

  // init basic walking
  WalkingMotionModule::Initialize(control_cycle_msec, robot);

  control_cycle_msec_ = control_cycle_msec;

  /** start ros thread */
  queue_thread_ = boost::thread(boost::bind(&WalkControlModule::QueueThread, this));
}

void WalkControlModule::Process(std::map<std::string, Dynamixel*> dxls, std::map<std::string, double> sensors)
{
  boost::mutex::scoped_lock lock(walk_control_mutex_);

  if (walk_controller_)
    walk_controller_->update();

  // finally run low level process
  WalkingMotionModule::Process(dxls, sensors);
}

void WalkControlModule::QueueThread()
{
  walk_control_mutex_.lock();

  ros::NodeHandle _nh("walk_control_module");
  ros::CallbackQueue _callback_queue;

  _nh.setCallbackQueue(&_callback_queue);

  // initialize parameter and plugin manager
  vigir_generic_params::ParameterManager::initialize(_nh);
  vigir_pluginlib::PluginManager::initialize(_nh);

  // start walk controller
  walk_controller_.reset(new vigir_walk_control::WalkController(_nh, false));

  // dynamic reconfigure
  dynamic_reconfigure::Server<thormang3_walk_control_module::BalanceParametersConfig> server_(_nh);
  dynamic_reconfigure::Server<thormang3_walk_control_module::BalanceParametersConfig>::CallbackType callback_f_;
  callback_f_ = boost::bind(&WalkControlModule::DynamicReconfigureCallback, this, _1, _2);
  server_.setCallback(callback_f_);

  walk_control_mutex_.unlock();

  ros::WallDuration duration(control_cycle_msec_/1000.0);
  while(_nh.ok())
    _callback_queue.callAvailable(duration);
}

void WalkControlModule::DynamicReconfigureCallback(thormang3_walk_control_module::BalanceParametersConfig &config, uint32_t level) {
  ROS_INFO_STREAM("Balance parameters changed.");
  thormang3_walking_module_msgs::BalanceParam  balance_param_msg;

  // ####### cob_offset #######
  balance_param_msg.cob_x_offset_m = config.cob_x_offset_m;
  balance_param_msg.cob_y_offset_m = config.cob_y_offset_m;

  // ####### FeedForward #####
  balance_param_msg.hip_roll_swap_angle_rad = config.hip_roll_swap_angle_rad;

  // ########## Gain ########
  // gyro
  balance_param_msg.gyro_gain = config.gyro_gain;

  // imu
  balance_param_msg.foot_roll_angle_gain = config.foot_roll_angle_gain;
  balance_param_msg.foot_pitch_angle_gain = config.foot_pitch_angle_gain;
  // ft sensor
  balance_param_msg.foot_x_force_gain = config.foot_x_force_gain;
  balance_param_msg.foot_y_force_gain = config.foot_z_force_gain;
  balance_param_msg.foot_z_force_gain = config.foot_z_force_gain;
  balance_param_msg.foot_roll_torque_gain = config.foot_roll_torque_gain;
  balance_param_msg.foot_pitch_torque_gain = config.foot_pitch_torque_gain;

  // ########## TIME CONSTANT ##########
  // imu
  balance_param_msg.foot_roll_angle_time_constant = config.foot_roll_angle_time_constant;
  balance_param_msg.foot_pitch_angle_time_constant = config.foot_pitch_angle_time_constant;
  // ft
  balance_param_msg.foot_x_force_time_constant = config.foot_x_force_time_constant;
  balance_param_msg.foot_y_force_time_constant = config.foot_y_force_time_constant;
  balance_param_msg.foot_z_force_time_constant = config.foot_z_force_time_constant;
  balance_param_msg.foot_roll_torque_time_constant = config.foot_roll_torque_time_constant;
  balance_param_msg.foot_pitch_torque_time_constant = config.foot_pitch_torque_time_constant;

  // ######### Balance Enabled #########
  PreviewControlWalking::GetInstance()->BALANCE_ENABLE = config.balance;

  WalkingMotionModule::SetBalanceParam(balance_param_msg);
}

}
