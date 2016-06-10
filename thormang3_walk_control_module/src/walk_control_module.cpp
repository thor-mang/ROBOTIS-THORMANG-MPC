#include <thormang3_walk_control_module/walk_control_module.h>

#include <vigir_generic_params/parameter_manager.h>
#include <vigir_pluginlib/plugin_manager.h>



namespace thormang3
{
WalkControlModule::WalkControlModule()
  : WalkingMotionModule()
  , control_cycle_msec_(8)
{
  enable_ = false;
  module_name_ = "walk_control_module"; // set unique module name
  control_mode_ = robotis_framework::PositionControl;
}

WalkControlModule::~WalkControlModule()
{
  queue_thread_.join();
}

void WalkControlModule::initialize(const int control_cycle_msec, robotis_framework::Robot* robot)
{
  boost::mutex::scoped_lock lock(walk_control_mutex_);

  // init basic walking
  WalkingMotionModule::initialize(control_cycle_msec, robot);

  control_cycle_msec_ = control_cycle_msec;

  /** start ros thread */
  queue_thread_ = boost::thread(boost::bind(&WalkControlModule::queueThread, this));
}

void WalkControlModule::process(std::map<std::string, robotis_framework::Dynamixel*> dxls, std::map<std::string, double> sensors)
{
  boost::mutex::scoped_lock lock(walk_control_mutex_);

  if (walk_controller_)
    walk_controller_->update();

  // finally run low level process
  WalkingMotionModule::process(dxls, sensors);
}

void WalkControlModule::queueThread()
{
  walk_control_mutex_.lock();

  ros::NodeHandle nh("walk_control_module");
  ros::CallbackQueue callback_queue;

  nh.setCallbackQueue(&callback_queue);

  // initialize parameter and plugin manager
  vigir_generic_params::ParameterManager::initialize(nh);
  vigir_pluginlib::PluginManager::initialize(nh);

  // start walk controller
  walk_controller_.reset(new vigir_walk_control::WalkController(nh, false));

  walk_control_mutex_.unlock();

  ros::WallDuration duration(control_cycle_msec_/1000.0);
  while(nh.ok())
    callback_queue.callAvailable(duration);
}
}
