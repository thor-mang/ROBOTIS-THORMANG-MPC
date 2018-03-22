#include <thormang3_l3_module/l3_module.h>



namespace thormang3
{
L3Module::L3Module()
  : control_cycle_msec_(8)
  , last_time_stamp_(ros::Time())
{
  enable_ = false;
  module_name_ = "l3_module"; // set unique module name
  control_mode_ = robotis_framework::PositionControl;
}

L3Module::~L3Module()
{
  for (auto& kv : result_)
    delete kv.second;

  queue_thread_.join();
}

void L3Module::onModuleEnable()
{
  boost::mutex::scoped_lock lock(l3_mutex_);

  // (re)initialize parameter and plugin manager
  vigir_generic_params::ParameterManager::initialize(queue_nh_);
  vigir_pluginlib::PluginManager::initialize(queue_nh_);

  // (re)initialize walk controller
  walk_controller_.reset(new l3_zmp_walk::WalkController(queue_nh_));
  walk_controller_->initialize(queue_nh_);

  // fetch sensor modules
  l3_zmp_walk::InterfaceManager::getPlugins(sensor_modules_);

  // get thormang joint cmd plugin
  l3_zmp_walk::InterfaceManager::getPlugin(joint_cmd_interface_);

  is_running_ = true;
}

void L3Module::initialize(const int control_cycle_msec, robotis_framework::Robot* robot)
{
  boost::mutex::scoped_lock lock(l3_mutex_);

  control_cycle_msec_ = control_cycle_msec;

  ros::NodeHandle nh("l3");

  // read joints from ros param server
  result_.clear();
  XmlRpc::XmlRpcValue joints = nh.param("joints", XmlRpc::XmlRpcValue());
  if (joints.getType() == XmlRpc::XmlRpcValue::TypeArray)
  {
    for (size_t i = 0; i < joints.size(); i++)
      result_[static_cast<std::string>(joints[i])] = new robotis_framework::DynamixelState();
  }
  else
    ROS_ERROR("[L3Module] Joints must be given as an array of strings.");

  /** start ros thread */
  queue_thread_ = boost::thread(boost::bind(&L3Module::queueThread, this));
}

void L3Module::process(std::map<std::string, robotis_framework::Dynamixel*> dxls, std::map<std::string, double> sensors)
{
  boost::mutex::scoped_lock lock(l3_mutex_);

  if (!walk_controller_->getState() == l3::READY)
  {
    ROS_ERROR_THROTTLE(10.0, "[L3Module] Walk Controller not ready. Probably initialization has failed.");
    return;
  }

  if (last_time_stamp_.isZero())
  {
    last_time_stamp_ = ros::Time::now();
    return;
  }

  ros::Time current_time = ros::Time::now();
  ros::Duration elapsed_time = current_time - last_time_stamp_;
  last_time_stamp_ = current_time;

  // update (sensor) interfaces
  for (RobotisSensorModule::Ptr module : sensor_modules_)
    module->process(dxls, sensors);

  // run controller
  walk_controller_->update(elapsed_time.toNSec()*10e-7);

  // call joint cmd interface for thormang
  joint_cmd_interface_->process(result_);
}

bool L3Module::isRunning()
{
  return is_running_;
}

void L3Module::stop()
{
  is_running_ = false;
  return;
}

void L3Module::queueThread()
{
  l3_mutex_.lock();

  queue_nh_ = ros::NodeHandle("l3");
  ros::CallbackQueue callback_queue;
  queue_nh_.setCallbackQueue(&callback_queue);

  l3_mutex_.unlock();

  ros::WallDuration duration(control_cycle_msec_/1000.0);
  while (queue_nh_.ok())
    callback_queue.callAvailable(duration);
}
}
