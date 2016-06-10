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

  walk_control_mutex_.unlock();

  ros::WallDuration duration(control_cycle_msec_/1000.0);
  while(_nh.ok())
    _callback_queue.callAvailable(duration);
}
}
