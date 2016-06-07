#include <thormang3_ros_control_module/ros_control_module.h>



namespace ROBOTIS
{
RosControlModule::RosControlModule()
  : control_cycle_msec_(8)
  , last_time_stamp_(ros::Time::now())
  , reset_controllers_(false)
{
  enable = false;
  module_name = "ros_control_module"; // set unique module name
  control_mode = POSITION_CONTROL;
}

RosControlModule::~RosControlModule()
{
  for (auto& kv : result)
    delete kv.second;

  queue_thread_.join();
}

void RosControlModule::Initialize(const int control_cycle_msec, Robot* robot)
{
  boost::mutex::scoped_lock lock(ros_control_mutex_);

  control_cycle_msec_ = control_cycle_msec;

  ros::NodeHandle nh("joints/ros_control_module");

  // clear already exisiting outputs
  for (auto& kv : result)
    delete kv.second;
  result.clear();

  /** initialize IMU */

  imu_sensor_interface_ = hardware_interface::ImuSensorInterface();

  imu_data_.name = "waist_imu";
  imu_data_.frame_id = "imu_link";
  imu_data_.orientation = imu_orientation_;
  imu_data_.angular_velocity = imu_angular_velocity_;
  imu_data_.linear_acceleration = imu_linear_acceleration_;
  hardware_interface::ImuSensorHandle imu_sensor_handle(imu_data_);
  imu_sensor_interface_.registerHandle(imu_sensor_handle);

  registerInterface(&imu_sensor_interface_);

  /** initialize FT-Sensors */

  force_torque_sensor_interface_ = hardware_interface::ForceTorqueSensorInterface();

  // read ft sensors from ros param server
  XmlRpc::XmlRpcValue ft_sensors = nh.param("ft_sensors", XmlRpc::XmlRpcValue());
  if (ft_sensors.getType() == XmlRpc::XmlRpcValue::TypeStruct)
  {
    for (XmlRpc::XmlRpcValue::iterator itr = ft_sensors.begin(); itr != ft_sensors.end(); itr++)
    {
      const std::string& sensor = itr->first;
      XmlRpc::XmlRpcValue val = itr->second;

      if (val.getType() == XmlRpc::XmlRpcValue::TypeArray && val.size() >= 2)
      {
        for (size_t i = 0; i < val.size(); i++)
        {
          const std::string& name = val[0];
          const std::string& frame_id = val[1];

          hardware_interface::ForceTorqueSensorHandle ft_sensor_handle(name, frame_id, force_[sensor], torque_[sensor]);
          force_torque_sensor_interface_.registerHandle(ft_sensor_handle);
        }
      }
      else
        ROS_ERROR("[RosControlModule] FT-Sensors info must be given as array of strings.");
    }
  }
  else
    ROS_ERROR("[RosControlModule] FT-Sensors config malformed.");

  registerInterface(&force_torque_sensor_interface_);

  /** initialize joints */

  // read joints from ros param server
  result.clear();
  XmlRpc::XmlRpcValue joints = nh.param("joints", XmlRpc::XmlRpcValue());
  if (joints.getType() == XmlRpc::XmlRpcValue::TypeArray)
  {
    for (size_t i = 0; i < joints.size(); i++)
      result[static_cast<std::string>(joints[i])] = new DynamixelState();
  }
  else
    ROS_ERROR("[RosControlModule] Joints must be given as an array of strings.");

  jnt_state_interface_ = hardware_interface::JointStateInterface();
  jnt_pos_interface_ = hardware_interface::PositionJointInterface();

  for (std::map<std::string, DynamixelState*>::iterator state_iter = result.begin(); state_iter != result.end(); state_iter++)
  {
    const std::string& joint_name = state_iter->first;

    // connect and register the joint state interface
    hardware_interface::JointStateHandle state_handle(joint_name, &pos_[joint_name], &vel_[joint_name], &eff_[joint_name]);
    jnt_state_interface_.registerHandle(state_handle);

    // connect and register the joint position interface
    hardware_interface::JointHandle pos_handle(state_handle, &cmd_[joint_name]);
    jnt_pos_interface_.registerHandle(pos_handle);
  }

  registerInterface(&jnt_state_interface_);
  registerInterface(&jnt_pos_interface_);

  /** start secondary controller manager and ros thread */
  controller_manager_thread_ = boost::thread(boost::bind(&RosControlModule::ControllerManagerThread, this));
  queue_thread_ = boost::thread(boost::bind(&RosControlModule::QueueThread, this));
}

void RosControlModule::Process(std::map<std::string, Dynamixel*> dxls, std::map<std::string, double> sensors)
{
  boost::mutex::scoped_lock lock(ros_control_mutex_);

  /** update IMU */

  /// TODO: Handle covariance matrices

  /** update FT-Sensors */

  for (auto& kv : force_)
  {
    kv.second[0] = sensors[kv.first + "_fx_raw_N"];
    kv.second[1] = sensors[kv.first + "_fy_raw_N"];
    kv.second[2] = sensors[kv.first + "_fz_raw_N"];
  }
  for (auto& kv : torque_)
  {
    kv.second[0] = sensors[kv.first + "_tx_raw_Nm"];
    kv.second[1] = sensors[kv.first + "_ty_raw_Nm"];
    kv.second[2] = sensors[kv.first + "_tz_raw_Nm"];
  }

  /** update joints */

  for(std::map<std::string, DynamixelState*>::iterator state_iter = result.begin(); state_iter != result.end(); state_iter++)
  {
    const std::string& joint_name = state_iter->first;

    pos_[joint_name] = dxls[state_iter->first]->dxl_state->present_position;
    vel_[joint_name] = dxls[state_iter->first]->dxl_state->present_velocity;
    eff_[joint_name] = dxls[state_iter->first]->dxl_state->present_current;
  }

  /** call controllers */

  // time measurements
  ros::Time current_time = ros::Time::now();
  ros::Duration elapsed_time = current_time - last_time_stamp_;
  last_time_stamp_ = current_time;

  if (controller_manager_)
  {
    controller_manager_->update(current_time, elapsed_time, reset_controllers_);
    reset_controllers_ = false;
  }

  /** write back commands */

  for(std::map<std::string, DynamixelState*>::iterator state_iter = result.begin(); state_iter != result.end(); state_iter++)
  {
    const std::string& joint_name = state_iter->first;

    result[joint_name]->goal_position = cmd_[joint_name];
  }
}

bool RosControlModule::IsRunning()
{
  return false; // hacky, return always false to signal release of resources
}

void RosControlModule::Stop()
{
  return;
}

void RosControlModule::ControllerManagerThread()
{
  // this thread keeps the controller manager active for loading and unloading controllers
  while (ros::ok())
  {
    usleep(control_cycle_msec_);
    
    // skip if enabled as Process will update controller
    if (enable)
      continue;
    
    boost::mutex::scoped_lock lock(ros_control_mutex_);
    
    // time measurements
    ros::Time current_time = ros::Time::now();
    ros::Duration elapsed_time = current_time - last_time_stamp_;
    last_time_stamp_ = current_time;
  
    if (controller_manager_)
    {
      controller_manager_->update(current_time, elapsed_time);
      reset_controllers_ = true; // set flag for resetting controllers when manager becomes enabled again
    }
  }
}

void RosControlModule::QueueThread()
{
  ros_control_mutex_.lock();

  ros::NodeHandle _ros_node("joints");
  ros::CallbackQueue _callback_queue;

  _ros_node.setCallbackQueue(&_callback_queue);

  // Initialize ros control
  last_time_stamp_ = ros::Time::now();
  controller_manager_.reset(new controller_manager::ControllerManager(this, _ros_node));

  ros_control_mutex_.unlock();

  ros::WallDuration duration(control_cycle_msec_/1000.0);
  while(_ros_node.ok())
    _callback_queue.callAvailable(duration);
}
}
