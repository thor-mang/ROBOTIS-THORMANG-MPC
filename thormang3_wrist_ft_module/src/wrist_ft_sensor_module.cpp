#include "thormang3_wrist_ft_module/wrist_ft_sensor_module.h"

#define EXT_PORT_DATA_1 "external_port_data_1"
#define EXT_PORT_DATA_2 "external_port_data_2"
#define EXT_PORT_DATA_3 "external_port_data_3"
#define EXT_PORT_DATA_4 "external_port_data_4"

using namespace ROBOTIS;

WristForceTorqueSensor::WristForceTorqueSensor()
  : control_cycle_msec_(8)
  , gazebo_robot_name("robotis")
  , gazebo_mode(false)
{
  module_name = "thormang3_wrist_force_torque_sensor_module"; // set unique module name

  thormang3_kd_ = new ROBOTIS::ThorMang3KinematicsDynamics(ROBOTIS::WHOLE_BODY);

  r_wrist_fx_raw_N  = r_wrist_fy_raw_N  = r_wrist_fz_raw_N  = 0;
  r_wrist_tx_raw_Nm = r_wrist_ty_raw_Nm = r_wrist_tz_raw_Nm = 0;
  l_wrist_fx_raw_N  = l_wrist_fy_raw_N  = l_wrist_fz_raw_N  = 0;
  l_wrist_tx_raw_Nm = l_wrist_ty_raw_Nm = l_wrist_tz_raw_Nm = 0;

  result["r_wrist_fx_raw_N"]	= r_wrist_fx_raw_N;
  result["r_wrist_fy_raw_N"]	= r_wrist_fy_raw_N;
  result["r_wrist_fz_raw_N"]	= r_wrist_fz_raw_N;
  result["r_wrist_tx_raw_Nm"]	= r_wrist_tx_raw_Nm;
  result["r_wrist_ty_raw_Nm"]	= r_wrist_ty_raw_Nm;
  result["r_wrist_tz_raw_Nm"]	= r_wrist_tz_raw_Nm;

  result["l_wrist_fx_raw_N"]	= l_wrist_fx_raw_N;
  result["l_wrist_fy_raw_N"]	= l_wrist_fy_raw_N;
  result["l_wrist_fz_raw_N"]	= l_wrist_fz_raw_N;
  result["l_wrist_tx_raw_Nm"]	= l_wrist_tx_raw_Nm;
  result["l_wrist_ty_raw_Nm"]	= l_wrist_ty_raw_Nm;
  result["l_wrist_tz_raw_Nm"]	= l_wrist_tz_raw_Nm;

  exist_r_wrist_an_r_ = false;
  exist_r_wrist_an_p_ = false;
  exist_l_wrist_an_r_ = false;
  exist_l_wrist_an_p_ = false;

  ft_period_		 = 2 * 1000/ control_cycle_msec_;

  ft_get_count_	= 0;
}

WristForceTorqueSensor::~WristForceTorqueSensor()
{
  queue_thread_.join();
}

void WristForceTorqueSensor::Initialize(const int control_cycle_msec, Robot *robot)
{
  control_cycle_msec_ = control_cycle_msec;

  ft_period_		 = 2 * 1000/ control_cycle_msec_;
  ft_get_count_ = ft_period_;

  queue_thread_       = boost::thread(boost::bind(&WristForceTorqueSensor::QueueThread, this));

  WristForceTorqueSensorInitialize();
}

void WristForceTorqueSensor::WristForceTorqueSensorInitialize()
{
  boost::mutex::scoped_lock lock(ft_sensor_mutex_);

  ros::NodeHandle _ros_node;


  std::string _wrist_ft_data_path  = _ros_node.param<std::string>("ft_data_path", "");
  std::string _ft_calib_data_path = _ros_node.param<std::string>("ft_calibration_data_path", "");

  r_wrist_ft_sensor_.Initialize(_wrist_ft_data_path, "ft_right_wrist", "r_arm_wr_p_link" , "sensor/ft/right_wrist/raw", "sensor/ft/right_wrist/scaled");
  l_wrist_ft_sensor_.Initialize(_wrist_ft_data_path, "ft_left_wrist",  "l_arm_wr_p_link",  "sensor/ft/left_wrist/raw",  "sensor/ft/left_wrist/scaled");
}


void 	WristForceTorqueSensor::FTSensorCalibrationCommandCallback(const std_msgs::String::ConstPtr& msg)
{
  boost::mutex::scoped_lock lock(ft_sensor_mutex_);

  if( (ft_command_ == FT_NONE) && (ft_period_ == ft_get_count_) )
  {
    std::string _command = msg->data;

    if(_command == "ft_air")
    {
      ft_get_count_ = 0;
      ft_command_ = FT_AIR;
      PublishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Start measuring FT_AIR");
      has_ft_air_		= false;
      r_wrist_ft_air_.fill(0);
      l_wrist_ft_air_.fill(0);
    }
  }
  else
    ROS_INFO("previous task is alive");
}

void WristForceTorqueSensor::PublishStatusMsg(unsigned int type, std::string msg)
{
  robotis_controller_msgs::StatusMsg _status;
  _status.header.stamp = ros::Time::now();
  _status.type = type;
  _status.module_name = "WristFT";
  _status.status_msg = msg;

  thormang3_wrist_ft_status_pub_.publish(_status);
}

void WristForceTorqueSensor::GazeboFTSensorCallback(const geometry_msgs::WrenchStamped::ConstPtr msg)
{
  if (!gazebo_mode)
    return;

  boost::mutex::scoped_lock lock(ft_sensor_mutex_);

  geometry_msgs::Wrench msg_transformed;
  msg_transformed.force.x = msg->wrench.force.x;
  msg_transformed.force.y = msg->wrench.force.y;
  msg_transformed.force.z = msg->wrench.force.z;

  if (msg->header.frame_id == "l_arm_wr_p_link")
  {
    l_wrist_ft_sensor_.SetCurrentForceTorqueRaw(msg_transformed);
  }
  else if (msg->header.frame_id == "r_arm_wr_p_link")
  {
    r_wrist_ft_sensor_.SetCurrentForceTorqueRaw(msg_transformed);
  }
  else
  {
    ROS_ERROR_ONCE_NAMED(msg->header.frame_id.c_str(), "[WristForceTorqueSensor] Unknown ft sensor callback with frame_id '%s'.", msg->header.frame_id.c_str());
    return;
  }
}

void WristForceTorqueSensor::QueueThread()
{
  ros::NodeHandle _nh;
  ros::CallbackQueue _callback_queue;

  _nh.setCallbackQueue(&_callback_queue);

  /* subscriber */
  ros::Subscriber ft_calib_command_sub	= _nh.subscribe("robotis/wrist_ft/ft_calib_command",	1, &WristForceTorqueSensor::FTSensorCalibrationCommandCallback, this);
  ros::Subscriber ft_left_wrist_sub	= _nh.subscribe("/gazebo/" + gazebo_robot_name + "/sensor/ft/left_wrist",	1, &WristForceTorqueSensor::GazeboFTSensorCallback, this);
  ros::Subscriber ft_right_wrist_sub	= _nh.subscribe("/gazebo/" + gazebo_robot_name + "/sensor/ft/right_wrist",	1, &WristForceTorqueSensor::GazeboFTSensorCallback, this);

  /* publisher */
  thormang3_wrist_ft_status_pub_	= _nh.advertise<robotis_controller_msgs::StatusMsg>("robotis/status", 1);

  ros::WallDuration duration(control_cycle_msec_/1000.0);
  while(_nh.ok())
    _callback_queue.callAvailable(duration);
}

void WristForceTorqueSensor::Process(std::map<std::string, Dynamixel *> dxls, std::map<std::string, Sensor *> sensors)
{
  boost::mutex::scoped_lock lock(ft_sensor_mutex_);

  exist_r_wrist_an_r_ = false;
  exist_r_wrist_an_p_ = false;
  exist_l_wrist_an_r_ = false;
  exist_l_wrist_an_p_ = false;

  if (!gazebo_mode)
  {
    std::map<std::string, Dynamixel*>::iterator _dxl_it = dxls.find("r_arm_wr_p");
    if(_dxl_it != dxls.end()) {

      r_wrist_ft_current_voltage_[0] = (_dxl_it->second->dxl_state->bulk_read_table[EXT_PORT_DATA_1])*3.3/4095.0;
      r_wrist_ft_current_voltage_[1] = (_dxl_it->second->dxl_state->bulk_read_table[EXT_PORT_DATA_2])*3.3/4095.0;
      r_wrist_ft_current_voltage_[2] = (_dxl_it->second->dxl_state->bulk_read_table[EXT_PORT_DATA_3])*3.3/4095.0;
      r_wrist_ft_current_voltage_[3] = (_dxl_it->second->dxl_state->bulk_read_table[EXT_PORT_DATA_4])*3.3/4095.0;
      exist_r_wrist_an_r_ = true;
    }
    else
      return;

    _dxl_it = dxls.find("r_arm_wr_r");
    if(_dxl_it != dxls.end()) {
      r_wrist_ft_current_voltage_[4] = (_dxl_it->second->dxl_state->bulk_read_table[EXT_PORT_DATA_1])*3.3/4095.0;
      r_wrist_ft_current_voltage_[5] = (_dxl_it->second->dxl_state->bulk_read_table[EXT_PORT_DATA_2])*3.3/4095.0;
      exist_r_wrist_an_p_ = true;
    }
    else
      return;

    _dxl_it = dxls.find("l_arm_wr_p");
    if(_dxl_it != dxls.end()) {
      l_wrist_ft_current_voltage_[0] = (_dxl_it->second->dxl_state->bulk_read_table[EXT_PORT_DATA_1])*3.3/4095.0;
      l_wrist_ft_current_voltage_[1] = (_dxl_it->second->dxl_state->bulk_read_table[EXT_PORT_DATA_2])*3.3/4095.0;
      l_wrist_ft_current_voltage_[2] = (_dxl_it->second->dxl_state->bulk_read_table[EXT_PORT_DATA_3])*3.3/4095.0;
      l_wrist_ft_current_voltage_[3] = (_dxl_it->second->dxl_state->bulk_read_table[EXT_PORT_DATA_4])*3.3/4095.0;
      exist_l_wrist_an_r_ = true;
    }
    else
      return;

    _dxl_it = dxls.find("l_arm_wr_r");
    if(_dxl_it != dxls.end())	{
      l_wrist_ft_current_voltage_[4] = (_dxl_it->second->dxl_state->bulk_read_table[EXT_PORT_DATA_1])*3.3/4095.0;
      l_wrist_ft_current_voltage_[5] = (_dxl_it->second->dxl_state->bulk_read_table[EXT_PORT_DATA_2])*3.3/4095.0;
      exist_l_wrist_an_p_ = true;
    }
    else
      return;
  }

  if (gazebo_mode || (exist_r_wrist_an_r_ && exist_r_wrist_an_p_))
  {
    if (!gazebo_mode)
    {
      r_wrist_ft_sensor_.SetCurrentVoltageOutputPublishForceTorque(	r_wrist_ft_current_voltage_[0],
          r_wrist_ft_current_voltage_[1],
          r_wrist_ft_current_voltage_[2],
          r_wrist_ft_current_voltage_[3],
          r_wrist_ft_current_voltage_[4],
          r_wrist_ft_current_voltage_[5]);
    }
    else
    {
      r_wrist_ft_sensor_.PublishForceTorque();
    }

    r_wrist_ft_sensor_.GetCurrentForceTorqueRaw(&r_wrist_fx_raw_N,  &r_wrist_fy_raw_N,  &r_wrist_fz_raw_N,
                                               &r_wrist_tx_raw_Nm, &r_wrist_ty_raw_Nm, &r_wrist_tz_raw_Nm);

    result["r_wrist_fx_raw_N"]	= r_wrist_fx_raw_N;
    result["r_wrist_fy_raw_N"]	= r_wrist_fy_raw_N;
    result["r_wrist_fz_raw_N"]	= r_wrist_fz_raw_N;
    result["r_wrist_tx_raw_Nm"]	= r_wrist_tx_raw_Nm;
    result["r_wrist_ty_raw_Nm"]	= r_wrist_ty_raw_Nm;
    result["r_wrist_tz_raw_Nm"]	= r_wrist_tz_raw_Nm;

  }

  if (gazebo_mode || (exist_l_wrist_an_r_ && exist_l_wrist_an_p_))
  {
    if (!gazebo_mode)
    {
      l_wrist_ft_sensor_.SetCurrentVoltageOutputPublishForceTorque(l_wrist_ft_current_voltage_[0],
          l_wrist_ft_current_voltage_[1],
          l_wrist_ft_current_voltage_[2],
          l_wrist_ft_current_voltage_[3],
          l_wrist_ft_current_voltage_[4],
          l_wrist_ft_current_voltage_[5]);
    }
    else
    {
      l_wrist_ft_sensor_.PublishForceTorque();
    }

    l_wrist_ft_sensor_.GetCurrentForceTorqueRaw(&l_wrist_fx_raw_N,  &l_wrist_fy_raw_N,  &l_wrist_fz_raw_N,
                                               &l_wrist_tx_raw_Nm, &l_wrist_ty_raw_Nm, &l_wrist_tz_raw_Nm);

    result["l_wrist_fx_raw_N"]	= l_wrist_fx_raw_N;
    result["l_wrist_fy_raw_N"]	= l_wrist_fy_raw_N;
    result["l_wrist_fz_raw_N"]	= l_wrist_fz_raw_N;
    result["l_wrist_tx_raw_Nm"]	= l_wrist_tx_raw_Nm;
    result["l_wrist_ty_raw_Nm"]	= l_wrist_ty_raw_Nm;
    result["l_wrist_tz_raw_Nm"]	= l_wrist_tz_raw_Nm;
  }


  if(ft_command_ == FT_NONE )
    return;
  else if(ft_command_ == FT_AIR)
  {
    ft_get_count_++;

    r_wrist_ft_air_.coeffRef(0, 0) += r_wrist_fx_raw_N;
    r_wrist_ft_air_.coeffRef(1, 0) += r_wrist_fy_raw_N;
    r_wrist_ft_air_.coeffRef(2, 0) += r_wrist_fz_raw_N;
    r_wrist_ft_air_.coeffRef(3, 0) += r_wrist_tx_raw_Nm;
    r_wrist_ft_air_.coeffRef(4, 0) += r_wrist_ty_raw_Nm;
    r_wrist_ft_air_.coeffRef(5, 0) += r_wrist_tz_raw_Nm;

    l_wrist_ft_air_.coeffRef(0, 0) += l_wrist_fx_raw_N;
    l_wrist_ft_air_.coeffRef(1, 0) += l_wrist_fy_raw_N;
    l_wrist_ft_air_.coeffRef(2, 0) += l_wrist_fz_raw_N;
    l_wrist_ft_air_.coeffRef(3, 0) += l_wrist_tx_raw_Nm;
    l_wrist_ft_air_.coeffRef(4, 0) += l_wrist_ty_raw_Nm;
    l_wrist_ft_air_.coeffRef(5, 0) += l_wrist_tz_raw_Nm;

    if(ft_get_count_ == ft_period_)
    {
      r_wrist_ft_air_  = r_wrist_ft_air_ / (double)ft_period_;
      l_wrist_ft_air_  = l_wrist_ft_air_ / (double)ft_period_;

      has_ft_air_ = true;
      PublishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Finish measuring FT_AIR");
      ft_command_ = FT_NONE;
    }
  }
}
