#include <thormang3_ros_control_module/ros_control_module.h>



namespace thormang3
{
RosControlModule::RosControlModule()
    : control_cycle_msec_(8)
{
  enable_          = false;
  module_name_     = "ros_control_module"; // set unique module name
  control_mode_    = robotis_framework::PositionControl;
  
  ros::NodeHandle nh("/thor_mang"); // TODO: Namespace handling
  
  XmlRpc::XmlRpcValue joints = nh.param("joints", XmlRpc::XmlRpcValue());
  
  if (joints.getType() == XmlRpc::XmlRpcValue::TypeArray)
  {
    for (size_t i = 0; i < joints.size(); i++)
    {
      result_[static_cast<std::string>(joints[i])] = new robotis_framework::DynamixelState();
      ROS_WARN("Joint: %s", static_cast<std::string>(joints[i]).c_str());
    }
  }
  else
    ROS_ERROR("[RosControlModule] Joints must be given as an array of strings.");
  
  registerInterface(&jnt_state_interface_);
  registerInterface(&jnt_pos_interface_);
}

RosControlModule::~RosControlModule()
{
  queue_thread_.join();
}

void RosControlModule::initialize(const int control_cycle_msec, robotis_framework::Robot* robot)
{
  control_cycle_msec_ = control_cycle_msec;
  
  last_time_stamp_ = ros::Time::now();
  
  queue_thread_ = boost::thread(boost::bind(&RosControlModule::QueueThread, this));
  
  jnt_state_interface_ = hardware_interface::JointStateInterface();
  jnt_pos_interface_ = hardware_interface::PositionJointInterface();
  
  for(std::map<std::string, robotis_framework::DynamixelState*>::iterator state_iter = result_.begin(); state_iter != result_.end(); state_iter++)
  {
    const std::string& joint_name = state_iter->first;
    
    // connect and register the joint state interface
    hardware_interface::JointStateHandle state_handle(joint_name, &pos_[joint_name], &vel_[joint_name], &eff_[joint_name]);
    jnt_state_interface_.registerHandle(state_handle);
    
    // connect and register the joint position interface
    hardware_interface::JointHandle pos_handle(state_handle, &cmd_[joint_name]);
    jnt_pos_interface_.registerHandle(pos_handle);
  }
}

void RosControlModule::process(std::map<std::string, robotis_framework::Dynamixel*> dxls, std::map<std::string, double> /*sensors*/)
{
  // time measurements
  ros::Time current_time = ros::Time::now();
  ros::Duration elapsed_time = current_time - last_time_stamp_;
  last_time_stamp_ = current_time;
  
  if (!enable_)
  {
    controller_manager->update(ros::Time::now(), elapsed_time);
    return;
  }

  for(std::map<std::string, robotis_framework::DynamixelState*>::iterator state_iter = result_.begin(); state_iter != result_.end(); state_iter++)
  {
    const std::string& joint_name = state_iter->first;
    
    pos_[joint_name] = dxls[state_iter->first]->dxl_state_->present_position_;
    vel_[joint_name] = dxls[state_iter->first]->dxl_state_->present_velocity_;
    eff_[joint_name] = dxls[state_iter->first]->dxl_state_->present_torque_;
  }

  controller_manager->update(ros::Time::now(), elapsed_time);
  
  for(std::map<std::string, robotis_framework::DynamixelState*>::iterator state_iter = result_.begin(); state_iter != result_.end(); state_iter++)
  {
    const std::string& joint_name = state_iter->first;
    
    result_[joint_name]->goal_position_ = cmd_[joint_name];
  }
}

bool RosControlModule::isRunning()
{
  return false;
}

void RosControlModule::stop()
{
  return;
}

void RosControlModule::QueueThread()
{
  ros::NodeHandle _ros_node("/thor_mang"); // TODO: Namespace handling
  ros::CallbackQueue _callback_queue;

  _ros_node.setCallbackQueue(&_callback_queue);
  
  // Initialize ros control
  controller_manager.reset(new controller_manager::ControllerManager(this, _ros_node));

  while(_ros_node.ok())
  {
    _callback_queue.callAvailable();
    usleep(100);
  }
}
}
