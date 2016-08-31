#include "thormang3_wrist_ft_module/wrist_force_torque_sensor_module.h"

#define EXT_PORT_DATA_1 "external_port_data_1"
#define EXT_PORT_DATA_2 "external_port_data_2"
#define EXT_PORT_DATA_3 "external_port_data_3"
#define EXT_PORT_DATA_4 "external_port_data_4"

using namespace thormang3;

WristForceTorqueSensor::WristForceTorqueSensor()
  : control_cycle_msec_(8)
{
  module_name_ = "thormang3_wrist_force_torque_sensor_module"; // set unique module name

  thormang3_kd_ = new KinematicsDynamics(thormang3::WholeBody);

  r_wrist_fx_raw_N  = r_wrist_fy_raw_N  = r_wrist_fz_raw_N  = 0;
  r_wrist_tx_raw_Nm = r_wrist_ty_raw_Nm = r_wrist_tz_raw_Nm = 0;
  l_wrist_fx_raw_N  = l_wrist_fy_raw_N  = l_wrist_fz_raw_N  = 0;
  l_wrist_tx_raw_Nm = l_wrist_ty_raw_Nm = l_wrist_tz_raw_Nm = 0;

  r_wrist_fx_scaled_N_  = r_wrist_fy_scaled_N_  = r_wrist_fz_scaled_N_  = 0;
  r_wrist_tx_scaled_Nm_ = r_wrist_ty_scaled_Nm_ = r_wrist_tz_scaled_Nm_ = 0;
  l_wrist_fx_scaled_N_  = l_wrist_fy_scaled_N_  = l_wrist_fz_scaled_N_  = 0;
  l_wrist_tx_scaled_Nm_ = l_wrist_ty_scaled_Nm_ = l_wrist_tz_scaled_Nm_ = 0;

  result_["r_wrist_fx_raw_N"]	= r_wrist_fx_raw_N;
  result_["r_wrist_fy_raw_N"]	= r_wrist_fy_raw_N;
  result_["r_wrist_fz_raw_N"]	= r_wrist_fz_raw_N;
  result_["r_wrist_tx_raw_Nm"]	= r_wrist_tx_raw_Nm;
  result_["r_wrist_ty_raw_Nm"]	= r_wrist_ty_raw_Nm;
  result_["r_wrist_tz_raw_Nm"]	= r_wrist_tz_raw_Nm;

  result_["l_wrist_fx_raw_N"]	= l_wrist_fx_raw_N;
  result_["l_wrist_fy_raw_N"]	= l_wrist_fy_raw_N;
  result_["l_wrist_fz_raw_N"]	= l_wrist_fz_raw_N;
  result_["l_wrist_tx_raw_Nm"]	= l_wrist_tx_raw_Nm;
  result_["l_wrist_ty_raw_Nm"]	= l_wrist_ty_raw_Nm;
  result_["l_wrist_tz_raw_Nm"]	= l_wrist_tz_raw_Nm;

  result_["r_wrist_fx_scaled_N"]  = r_wrist_fx_scaled_N_;
  result_["r_wrist_fy_scaled_N"]  = r_wrist_fy_scaled_N_;
  result_["r_wrist_fz_scaled_N"]  = r_wrist_fz_scaled_N_;
  result_["r_wrist_tx_scaled_Nm"] = r_wrist_tx_scaled_Nm_;
  result_["r_wrist_ty_scaled_Nm"] = r_wrist_ty_scaled_Nm_;
  result_["r_wrist_tz_scaled_Nm"] = r_wrist_tz_scaled_Nm_;

  result_["l_wrist_fx_scaled_N"]  = l_wrist_fx_scaled_N_;
  result_["l_wrist_fy_scaled_N"]  = l_wrist_fy_scaled_N_;
  result_["l_wrist_fz_scaled_N"]  = l_wrist_fz_scaled_N_;
  result_["l_wrist_tx_scaled_Nm"] = l_wrist_tx_scaled_Nm_;
  result_["l_wrist_ty_scaled_Nm"] = l_wrist_ty_scaled_Nm_;
  result_["l_wrist_tz_scaled_Nm"] = l_wrist_tz_scaled_Nm_;

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

void WristForceTorqueSensor::initialize(const int control_cycle_msec, robotis_framework::Robot *robot)
{
  control_cycle_msec_ = control_cycle_msec;

  ft_period_		 = 2 * 1000/ control_cycle_msec_;
  ft_get_count_ = ft_period_;

  queue_thread_       = boost::thread(boost::bind(&WristForceTorqueSensor::msgQueueThread, this));

  wristForceTorqueSensorInitialize();
}

void WristForceTorqueSensor::wristForceTorqueSensorInitialize()
{
  boost::mutex::scoped_lock lock(ft_sensor_mutex_);

  ros::NodeHandle _ros_node;
  std::string wrist_ft_data_path  = _ros_node.param<std::string>("ft_data_path", "");
  std::string ft_calib_data_path = _ros_node.param<std::string>("wrist_ft_calibration_data_path", "");

  r_wrist_ft_sensor_.initialize(wrist_ft_data_path, "ft_right_wrist", "r_wrist_ft_link" , "sensor/ft/right_wrist/raw", "sensor/ft/right_wrist/scaled");
  l_wrist_ft_sensor_.initialize(wrist_ft_data_path, "ft_left_wrist",  "l_wrist_ft_link",  "sensor/ft/left_wrist/raw",  "sensor/ft/left_wrist/scaled");

  YAML::Node doc;
  doc = YAML::LoadFile(ft_calib_data_path.c_str());

  std::vector<double> ft;
  ft = doc["ft_right_wrist_air"].as<std::vector<double> >();
  r_wrist_ft_air_ = Eigen::Map<Eigen::MatrixXd>(ft.data(), 6, 1);

  ft.clear();
  ft = doc["ft_left_wrist_air"].as<std::vector<double> >();
  l_wrist_ft_air_ = Eigen::Map<Eigen::MatrixXd>(ft.data(), 6, 1);

  r_wrist_ft_sensor_.setScaleParam(1, r_wrist_ft_air_);
  l_wrist_ft_sensor_.setScaleParam(1, l_wrist_ft_air_);
  publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Loaded wrist FT Calibration Data");
}

void WristForceTorqueSensor::saveFTCalibrationData(const std::string &path)
{
  if(!has_ft_air_) return;

  YAML::Emitter out;

  out << YAML::BeginMap;

  // air - right
  std::vector<double> ft_calibration;
  for(int ix = 0; ix < 6; ix++)
    ft_calibration.push_back(r_wrist_ft_air_.coeff(ix, 0));
  out << YAML::Key << "ft_right_wrist_air" << YAML::Value << ft_calibration;

  // air - left
  ft_calibration.clear();
  for(int ix = 0; ix < 6; ix++)
    ft_calibration.push_back(l_wrist_ft_air_.coeff(ix, 0));
  out << YAML::Key << "ft_left_wrist_air" << YAML::Value << ft_calibration;

  out << YAML::EndMap;

  // output to file
  std::ofstream fout(path.c_str());
  fout << out.c_str();

  ROS_INFO("Save FT wrist calibration data");
  publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Saved wrist FT Calibration Data");
}


void WristForceTorqueSensor::FTSensorCalibrationCommandCallback(const std_msgs::String::ConstPtr& msg)
{
  boost::mutex::scoped_lock lock(ft_sensor_mutex_);

  if( (ft_command_ == FT_NONE) && (ft_period_ == ft_get_count_) )
  {
    std::string command = msg->data;

    if(command == "ft_air")
    {
      ft_get_count_ = 0;
      ft_command_ = FT_AIR;
      publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Start measuring FT_AIR");
      has_ft_air_		= false;
      r_wrist_ft_air_.fill(0);
      l_wrist_ft_air_.fill(0);
    }
    else if(command == "ft_send")
    {
      if(has_ft_air_)
      {
        r_wrist_ft_sensor_.setScaleParam(1, r_wrist_ft_air_);
        l_wrist_ft_sensor_.setScaleParam(1, l_wrist_ft_air_);

        publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Applied wrist FT Calibration");
      }
      else
      {
        publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, "There is no value for calibration");
      }
    }
    else if(command == "ft_save")
    {
      ros::NodeHandle ros_node;
      std::string ft_calib_data_path = ros_node.param<std::string>("wrist_ft_calibration_data_path", "");
      saveFTCalibrationData(ft_calib_data_path);
    }
  }
  else
    ROS_INFO("previous task is alive");
}

void WristForceTorqueSensor::publishStatusMsg(unsigned int type, std::string msg)
{
  robotis_controller_msgs::StatusMsg _status;
  _status.header.stamp = ros::Time::now();
  _status.type = type;
  _status.module_name = "WristFT";
  _status.status_msg = msg;

  ROS_INFO_STREAM("Status: " << msg);

  thormang3_wrist_ft_status_pub_.publish(_status);
}

void WristForceTorqueSensor::gazeboFTSensorCallback(const geometry_msgs::WrenchStamped::ConstPtr msg)
{
  if (!gazebo_mode_)
    return;

  boost::mutex::scoped_lock lock(ft_sensor_mutex_);

  geometry_msgs::Wrench msg_transformed;
  msg_transformed.force.x = msg->wrench.force.x;
  msg_transformed.force.y = msg->wrench.force.y;
  msg_transformed.force.z = msg->wrench.force.z;

  msg_transformed.torque.x = msg->wrench.torque.x;
  msg_transformed.torque.y = msg->wrench.torque.y;
  msg_transformed.torque.z = msg->wrench.torque.z;

  if (msg->header.frame_id == "l_wrist_ft_link")
  {
    l_wrist_ft_sensor_.setCurrentForceTorqueRaw(msg_transformed);
  }
  else if (msg->header.frame_id == "r_wrist_ft_link")
  {
    r_wrist_ft_sensor_.setCurrentForceTorqueRaw(msg_transformed);
  }
  else
  {
    ROS_ERROR_ONCE_NAMED(msg->header.frame_id.c_str(), "[ThorMang3WristForceTorqueSensor] Unknown ft sensor callback with frame_id '%s'.", msg->header.frame_id.c_str());
    return;
  }
}

void WristForceTorqueSensor::msgQueueThread()
{
  ros::NodeHandle     _ros_node;
  ros::CallbackQueue  _callback_queue;

  _ros_node.setCallbackQueue(&_callback_queue);

  /* subscriber */
  ros::Subscriber ft_calib_command_sub	= _ros_node.subscribe("robotis/wrists_ft/ft_calib_command",	1, &WristForceTorqueSensor::FTSensorCalibrationCommandCallback, this);
  ros::Subscriber ft_left_wrist_sub	= _ros_node.subscribe("/gazebo/" + gazebo_robot_name_ + "/sensor/ft/left_wrist",	1, &WristForceTorqueSensor::gazeboFTSensorCallback, this);
  ros::Subscriber ft_right_wrist_sub	= _ros_node.subscribe("/gazebo/" + gazebo_robot_name_ + "/sensor/ft/right_wrist",	1, &WristForceTorqueSensor::gazeboFTSensorCallback, this);

  /* publisher */
  thormang3_wrist_ft_status_pub_	= _ros_node.advertise<robotis_controller_msgs::StatusMsg>("robotis/status", 1);

  ros::WallDuration duration(control_cycle_msec_/1000.0);
  while(_ros_node.ok())
    _callback_queue.callAvailable(duration);
}

void WristForceTorqueSensor::process(std::map<std::string, robotis_framework::Dynamixel *> dxls, std::map<std::string, robotis_framework::Sensor *> sensors)
{
  boost::mutex::scoped_lock lock(ft_sensor_mutex_);

  exist_r_wrist_an_r_ = false;
  exist_r_wrist_an_p_ = false;
  exist_l_wrist_an_r_ = false;
  exist_l_wrist_an_p_ = false;

  if (!gazebo_mode_)
  {
    std::map<std::string, robotis_framework::Dynamixel*>::iterator dxl_it = dxls.find("r_arm_wr_p");
    if(dxl_it != dxls.end()) {

      r_wrist_ft_current_voltage_[0] = (dxl_it->second->dxl_state_->bulk_read_table_[EXT_PORT_DATA_1])*3.3/4095.0;
      r_wrist_ft_current_voltage_[1] = (dxl_it->second->dxl_state_->bulk_read_table_[EXT_PORT_DATA_2])*3.3/4095.0;
      r_wrist_ft_current_voltage_[2] = (dxl_it->second->dxl_state_->bulk_read_table_[EXT_PORT_DATA_3])*3.3/4095.0;
      r_wrist_ft_current_voltage_[3] = (dxl_it->second->dxl_state_->bulk_read_table_[EXT_PORT_DATA_4])*3.3/4095.0;
      exist_r_wrist_an_r_ = true;
    }
    else
      return;

    dxl_it = dxls.find("r_arm_wr_r");
    if(dxl_it != dxls.end()) {
      r_wrist_ft_current_voltage_[4] = (dxl_it->second->dxl_state_->bulk_read_table_[EXT_PORT_DATA_1])*3.3/4095.0;
      r_wrist_ft_current_voltage_[5] = (dxl_it->second->dxl_state_->bulk_read_table_[EXT_PORT_DATA_2])*3.3/4095.0;
      exist_r_wrist_an_p_ = true;
    }
    else
      return;

    dxl_it = dxls.find("l_arm_wr_p");
    if(dxl_it != dxls.end()) {
      l_wrist_ft_current_voltage_[0] = (dxl_it->second->dxl_state_->bulk_read_table_[EXT_PORT_DATA_1])*3.3/4095.0;
      l_wrist_ft_current_voltage_[1] = (dxl_it->second->dxl_state_->bulk_read_table_[EXT_PORT_DATA_2])*3.3/4095.0;
      l_wrist_ft_current_voltage_[2] = (dxl_it->second->dxl_state_->bulk_read_table_[EXT_PORT_DATA_3])*3.3/4095.0;
      l_wrist_ft_current_voltage_[3] = (dxl_it->second->dxl_state_->bulk_read_table_[EXT_PORT_DATA_4])*3.3/4095.0;
      exist_l_wrist_an_r_ = true;
    }
    else
      return;

    dxl_it = dxls.find("l_arm_wr_r");
    if(dxl_it != dxls.end())	{
      l_wrist_ft_current_voltage_[4] = (dxl_it->second->dxl_state_->bulk_read_table_[EXT_PORT_DATA_1])*3.3/4095.0;
      l_wrist_ft_current_voltage_[5] = (dxl_it->second->dxl_state_->bulk_read_table_[EXT_PORT_DATA_2])*3.3/4095.0;
      exist_l_wrist_an_p_ = true;
    }
    else
      return;
  }

  if (gazebo_mode_ || (exist_r_wrist_an_r_ && exist_r_wrist_an_p_))
  {
    if (!gazebo_mode_)
    {
      r_wrist_ft_sensor_.setCurrentVoltageOutputPublish(r_wrist_ft_current_voltage_[0],
          r_wrist_ft_current_voltage_[1],
          r_wrist_ft_current_voltage_[2],
          r_wrist_ft_current_voltage_[3],
          r_wrist_ft_current_voltage_[4],
          r_wrist_ft_current_voltage_[5]);
    }
    else
    {
      r_wrist_ft_sensor_.publishForceTorque();
    }

    r_wrist_ft_sensor_.getCurrentForceTorqueRaw(&r_wrist_fx_raw_N,  &r_wrist_fy_raw_N,  &r_wrist_fz_raw_N,
                                               &r_wrist_tx_raw_Nm, &r_wrist_ty_raw_Nm, &r_wrist_tz_raw_Nm);
    r_wrist_ft_sensor_.getCurrentForceTorqueScaled(&r_wrist_fx_scaled_N_,  &r_wrist_fy_scaled_N_,  &r_wrist_fz_scaled_N_,
        &r_wrist_tx_scaled_Nm_, &r_wrist_ty_scaled_Nm_, &r_wrist_tz_scaled_Nm_);

    result_["r_wrist_fx_raw_N"]	= r_wrist_fx_raw_N;
    result_["r_wrist_fy_raw_N"]	= r_wrist_fy_raw_N;
    result_["r_wrist_fz_raw_N"]	= r_wrist_fz_raw_N;
    result_["r_wrist_tx_raw_Nm"]	= r_wrist_tx_raw_Nm;
    result_["r_wrist_ty_raw_Nm"]	= r_wrist_ty_raw_Nm;
    result_["r_wrist_tz_raw_Nm"]	= r_wrist_tz_raw_Nm;

    result_["r_wrist_fx_scaled_N"]  = r_wrist_fx_scaled_N_;
    result_["r_wrist_fy_scaled_N"]  = r_wrist_fy_scaled_N_;
    result_["r_wrist_fz_scaled_N"]  = r_wrist_fz_scaled_N_;
    result_["r_wrist_tx_scaled_Nm"] = r_wrist_tx_scaled_Nm_;
    result_["r_wrist_ty_scaled_Nm"] = r_wrist_ty_scaled_Nm_;
    result_["r_wrist_tz_scaled_Nm"] = r_wrist_tz_scaled_Nm_;

  }

  if (gazebo_mode_ || (exist_l_wrist_an_r_ && exist_l_wrist_an_p_))
  {
    if (!gazebo_mode_)
    {
      l_wrist_ft_sensor_.setCurrentVoltageOutputPublish(l_wrist_ft_current_voltage_[0],
          l_wrist_ft_current_voltage_[1],
          l_wrist_ft_current_voltage_[2],
          l_wrist_ft_current_voltage_[3],
          l_wrist_ft_current_voltage_[4],
          l_wrist_ft_current_voltage_[5]);
    }
    else
    {
      l_wrist_ft_sensor_.publishForceTorque();
    }

    l_wrist_ft_sensor_.getCurrentForceTorqueRaw(&l_wrist_fx_raw_N,  &l_wrist_fy_raw_N,  &l_wrist_fz_raw_N,
                                               &l_wrist_tx_raw_Nm, &l_wrist_ty_raw_Nm, &l_wrist_tz_raw_Nm);
    l_wrist_ft_sensor_.getCurrentForceTorqueScaled(&l_wrist_fx_scaled_N_,  &l_wrist_fy_scaled_N_,  &l_wrist_fz_scaled_N_,
        &l_wrist_tx_scaled_Nm_, &l_wrist_ty_scaled_Nm_, &l_wrist_tz_scaled_Nm_);

    result_["l_wrist_fx_raw_N"]	= l_wrist_fx_raw_N;
    result_["l_wrist_fy_raw_N"]	= l_wrist_fy_raw_N;
    result_["l_wrist_fz_raw_N"]	= l_wrist_fz_raw_N;
    result_["l_wrist_tx_raw_Nm"]	= l_wrist_tx_raw_Nm;
    result_["l_wrist_ty_raw_Nm"]	= l_wrist_ty_raw_Nm;
    result_["l_wrist_tz_raw_Nm"]	= l_wrist_tz_raw_Nm;

    result_["l_wrist_fx_scaled_N"]  = l_wrist_fx_scaled_N_;
    result_["l_wrist_fy_scaled_N"]  = l_wrist_fy_scaled_N_;
    result_["l_wrist_fz_scaled_N"]  = l_wrist_fz_scaled_N_;
    result_["l_wrist_tx_scaled_Nm"] = l_wrist_tx_scaled_Nm_;
    result_["l_wrist_ty_scaled_Nm"] = l_wrist_ty_scaled_Nm_;
    result_["l_wrist_tz_scaled_Nm"] = l_wrist_tz_scaled_Nm_;
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
      publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Finish measuring FT_AIR");
      ft_command_ = FT_NONE;
    }
  }
}
