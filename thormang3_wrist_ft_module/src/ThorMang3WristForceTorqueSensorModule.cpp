/*
 * ForceTorqueSensor.cpp
 *
 *  Created on: 2016. 2. 17.
 *      Author: HJSONG
 */




#include "thormang3_wrist_ft_module/ThorMang3WristForceTorqueSensorModule.h"


using namespace ROBOTIS;

ThorMang3WristForceTorqueSensor *ThorMang3WristForceTorqueSensor::unique_instance_ = new ThorMang3WristForceTorqueSensor();

ThorMang3WristForceTorqueSensor::ThorMang3WristForceTorqueSensor()
    : control_cycle_msec_(0)
{
    enable          = false;
    module_name     = "thormang3_wrist_force_torque_sensor";
    control_mode    = POSITION_CONTROL;

    r_wrist_ft_current_voltage_[0] = 0.5*3.3;
    r_wrist_ft_current_voltage_[1] = 0.5*3.3;
    r_wrist_ft_current_voltage_[2] = 0.5*3.3;
    r_wrist_ft_current_voltage_[3] = 0.5*3.3;
    r_wrist_ft_current_voltage_[4] = 0.5*3.3;
    r_wrist_ft_current_voltage_[5] = 0.5*3.3;

    l_wrist_ft_current_voltage_[0] = 0.5*3.3;
    l_wrist_ft_current_voltage_[1] = 0.5*3.3;
    l_wrist_ft_current_voltage_[2] = 0.5*3.3;
    l_wrist_ft_current_voltage_[3] = 0.5*3.3;
    l_wrist_ft_current_voltage_[4] = 0.5*3.3;
    l_wrist_ft_current_voltage_[5] = 0.5*3.3;

	r_wrist_fx_raw_N_ = r_wrist_fy_raw_N_ = r_wrist_fz_raw_N_ = r_wrist_tx_raw_Nm_ = r_wrist_ty_raw_Nm_ =  r_wrist_tz_raw_Nm_ = 0;
	l_wrist_fx_raw_N_ = l_wrist_fy_raw_N_ = l_wrist_fz_raw_N_ = l_wrist_tx_raw_Nm_ = l_wrist_ty_raw_Nm_ =  l_wrist_tz_raw_Nm_ = 0;
	r_wrist_fx_scaled_N_  = r_wrist_fy_scaled_N_  = r_wrist_fz_scaled_N_  = 0;
	r_wrist_tx_scaled_Nm_ = r_wrist_ty_scaled_Nm_ = r_wrist_tz_scaled_Nm_ = 0;
	l_wrist_fx_scaled_N_  = l_wrist_fy_scaled_N_  = l_wrist_fz_scaled_N_  = 0;
	l_wrist_tx_scaled_Nm_ = l_wrist_ty_scaled_Nm_ = l_wrist_tz_scaled_Nm_ = 0;
	r_wrist_ft_null_.resize(6, 1);	l_wrist_ft_null_.resize(6, 1);
	r_wrist_ft_null_.fill(0);		l_wrist_ft_null_.fill(0);



	status_msg.data = "";
	r_wrist_ft_calib_start_ = l_wrist_ft_calib_start_ = false;
	current_wrist_ft_calib_time_ = 2.0;
	wrist_ft_calib_duration_ = 2.0;

    result["r_arm_wr_y"]    = new DynamixelState();
    result["l_arm_wr_y"]    = new DynamixelState();
    result["r_arm_wr_p"]    = new DynamixelState();
    result["l_arm_wr_p"]    = new DynamixelState();
}

ThorMang3WristForceTorqueSensor::~ThorMang3WristForceTorqueSensor()
{
    queue_thread_.join();
}

void ThorMang3WristForceTorqueSensor::Initialize(const int control_cycle_msec)
{
    queue_thread_ = boost::thread(boost::bind(&ThorMang3WristForceTorqueSensor::QueueThread, this));

    control_cycle_msec_ = control_cycle_msec;

    WristForceTorqueSensorInitialize();
}

void ThorMang3WristForceTorqueSensor::QueueThread()
{
    ros::NodeHandle     _ros_node;
    ros::CallbackQueue  _callback_queue;

    _ros_node.setCallbackQueue(&_callback_queue);

    /* subscribe topics */
    thormang3_wrist_ft_status_pub_ = _ros_node.advertise<std_msgs::String>("/robotis/sensor/wrist_ft/status_message", 1);
    //ros::Subscriber sub1 = _ros_node.subscribe("/test_module_topic", 10, &ThorMang3WristForceTorqueSensor::TopicCallback, this);
    ros::Subscriber _calibration_duration_sub = _ros_node.subscribe("robotis/sensor/wrist_ft/calib_duration", 1, &ThorMang3WristForceTorqueSensor::WristFTCalibratingDurationCallback, this);
    ros::Subscriber _calibration_command_sub  = _ros_node.subscribe("robotis/sensor/wrist_ft/calib_command",  1, &ThorMang3WristForceTorqueSensor::WristFTCalibrationCommandCallback,  this);


    while(_ros_node.ok())
    {
        _callback_queue.callAvailable();
    }
}

void	ThorMang3WristForceTorqueSensor::WristFTCalibratingDurationCallback(const std_msgs::Float64::ConstPtr& msg)
{
	if(msg->data < control_cycle_msec_*0.001)
		ROS_ERROR("Invalid Duration");
	else {
		if(r_wrist_ft_calib_start_ || l_wrist_ft_calib_start_)
			ROS_ERROR("Previous Command is not finished");
		else
			wrist_ft_calib_duration_ = msg->data;
	}
}

void	ThorMang3WristForceTorqueSensor::WristFTCalibrationCommandCallback(const std_msgs::String::ConstPtr& msg)
{
	if(msg->data == "right_wrist_ft_calibration")
	{
		if(r_wrist_ft_calib_start_ || l_wrist_ft_calib_start_)
			ROS_ERROR("Previous Command is not finished");
		else {
			current_wrist_ft_calib_time_ = 0;
			r_wrist_ft_null_.fill(0);
			r_wrist_ft_calib_start_ = true;
		}
	}
	else if(msg->data == "left_wrist_ft_calibration")
	{
		if(r_wrist_ft_calib_start_ || l_wrist_ft_calib_start_)
			ROS_ERROR("Previous Command is not finished");
		else {
			current_wrist_ft_calib_time_ = 0;
			l_wrist_ft_null_.fill(0);
			l_wrist_ft_calib_start_ = true;
		}
	}
	else if(msg->data == "all_wrist_ft_calibration")
	{
		if(r_wrist_ft_calib_start_ || l_wrist_ft_calib_start_)
			ROS_ERROR("Previous Command is not finished");
		else {
			current_wrist_ft_calib_time_ = 0;
			r_wrist_ft_null_.fill(0);
			l_wrist_ft_null_.fill(0);
			r_wrist_ft_calib_start_ = true;
			l_wrist_ft_calib_start_ = true;
		}
	}
	else
		ROS_ERROR("Invalid Command");
}

void	ThorMang3WristForceTorqueSensor::WristForceTorqueSensorInitialize()
{
    ros::NodeHandle _ros_node;
    std::string _path = "";
    _path = ros::package::getPath("thormang3_wrist_ft_module") + "/config/wrist_ft_data.yaml";
    std::string _wrist_ft_data_path = _ros_node.param<std::string>("wrist_ft_data_path", _path);

    r_wrist_ft_sensor_.Initialize(_wrist_ft_data_path, "ft_right_wrist", "r_arm_ft_link" , "/robotis/sensor/ft_right_wrist/raw", "/robotis/sensor/ft_right_wrist/scaled");
    l_wrist_ft_sensor_.Initialize(_wrist_ft_data_path, "ft_left_wrist",  "l_arm_ft_link",  "/robotis/sensor/ft_left_wrist/raw",  "/robotis/sensor/ft_left_wrist/scaled");

    r_wrist_ft_sensor_.SetScaleParam(1.0, r_wrist_ft_null_);
    l_wrist_ft_sensor_.SetScaleParam(1.0, l_wrist_ft_null_);
}

void	ThorMang3WristForceTorqueSensor::Process(std::map<std::string, Dynamixel *> dxls)
{
	bool _exist_r_arm_wr_y = false;
	bool _exist_r_arm_wr_p = false;
	bool _exist_l_arm_wr_y = false;
	bool _exist_l_arm_wr_p = false;

	for(std::map<std::string, DynamixelState *>::iterator state_iter = result.begin(); state_iter != result.end(); state_iter++)
	{
		std::string _joint_name = state_iter->first;

        Dynamixel *_dxl = NULL;
        std::map<std::string, Dynamixel*>::iterator _dxl_it = dxls.find(_joint_name);
        if(_dxl_it != dxls.end())
            _dxl = _dxl_it->second;
        else
            continue;

		if(_joint_name == "r_arm_wr_p")
		{
			r_wrist_ft_current_voltage_[0] = (_dxl->dxl_state->ext_port_data[0])*3.3/4095.0;
			r_wrist_ft_current_voltage_[1] = (_dxl->dxl_state->ext_port_data[1])*3.3/4095.0;
			r_wrist_ft_current_voltage_[2] = (_dxl->dxl_state->ext_port_data[2])*3.3/4095.0;
			r_wrist_ft_current_voltage_[3] = (_dxl->dxl_state->ext_port_data[3])*3.3/4095.0;
			_exist_r_arm_wr_y = true;
		}
		else if(_joint_name == "r_arm_wr_y")
		{
			r_wrist_ft_current_voltage_[4] = (_dxl->dxl_state->ext_port_data[0])*3.3/4095.0;
			r_wrist_ft_current_voltage_[5] = (_dxl->dxl_state->ext_port_data[1])*3.3/4095.0;
			_exist_r_arm_wr_p = true;
		}
		else if(_joint_name == "l_arm_wr_p")
		{
			l_wrist_ft_current_voltage_[0] = (_dxl->dxl_state->ext_port_data[0])*3.3/4095.0;
			l_wrist_ft_current_voltage_[1] = (_dxl->dxl_state->ext_port_data[1])*3.3/4095.0;
			l_wrist_ft_current_voltage_[2] = (_dxl->dxl_state->ext_port_data[2])*3.3/4095.0;
			l_wrist_ft_current_voltage_[3] = (_dxl->dxl_state->ext_port_data[3])*3.3/4095.0;
			_exist_l_arm_wr_y = true;
		}
		else if(_joint_name == "l_arm_wr_y")
		{
			l_wrist_ft_current_voltage_[4] = (_dxl->dxl_state->ext_port_data[0])*3.3/4095.0;
			l_wrist_ft_current_voltage_[5] = (_dxl->dxl_state->ext_port_data[1])*3.3/4095.0;
			_exist_l_arm_wr_p = true;
		}
		else
			continue;
	}





//	printf(" r: %f %f %f %f %f %f\n", r_foot_ft_current_voltage_[0],
//															   r_foot_ft_current_voltage_[1],
//															   r_foot_ft_current_voltage_[2],
//															   r_foot_ft_current_voltage_[3],
//															   r_foot_ft_current_voltage_[4],
//															   r_foot_ft_current_voltage_[5]);
//
//	printf(" l: %f %f %f %f %f %f\n", l_foot_ft_current_voltage_[0],
//															   l_foot_ft_current_voltage_[1],
//															   l_foot_ft_current_voltage_[2],
//															   l_foot_ft_current_voltage_[3],
//															   l_foot_ft_current_voltage_[4],
//															   l_foot_ft_current_voltage_[5]);
	if(_exist_r_arm_wr_y &&  _exist_r_arm_wr_p) {
		r_wrist_ft_sensor_.SetCurrentVoltageOutputPublishForceTorque(r_wrist_ft_current_voltage_[0],
				r_wrist_ft_current_voltage_[1],
				r_wrist_ft_current_voltage_[2],
				r_wrist_ft_current_voltage_[3],
				r_wrist_ft_current_voltage_[4],
				r_wrist_ft_current_voltage_[5]);
		r_wrist_ft_sensor_.GetCurrentForceTorqueScaled(	&r_wrist_fx_scaled_N_,  &r_wrist_fy_scaled_N_,  &r_wrist_fz_scaled_N_,
				&r_wrist_tx_scaled_Nm_, &r_wrist_ty_scaled_Nm_, &r_wrist_tz_scaled_Nm_);
		r_wrist_ft_sensor_.GetCurrentForceTorqueRaw( &r_wrist_fx_raw_N_, &r_wrist_fy_raw_N_, &r_wrist_fz_raw_N_,
				&r_wrist_tx_raw_Nm_, &r_wrist_ty_raw_Nm_, &r_wrist_tz_raw_Nm_);
	}

	if(_exist_l_arm_wr_y &&  _exist_l_arm_wr_p) {
		l_wrist_ft_sensor_.SetCurrentVoltageOutputPublishForceTorque(	l_wrist_ft_current_voltage_[0],
				l_wrist_ft_current_voltage_[1],
				l_wrist_ft_current_voltage_[2],
				l_wrist_ft_current_voltage_[3],
				l_wrist_ft_current_voltage_[4],
				l_wrist_ft_current_voltage_[5]);
		l_wrist_ft_sensor_.GetCurrentForceTorqueScaled(	&l_wrist_fx_scaled_N_,  &l_wrist_fy_scaled_N_,  &l_wrist_fz_scaled_N_,
				&l_wrist_tx_scaled_Nm_, &l_wrist_ty_scaled_Nm_, &l_wrist_tz_scaled_Nm_);
		l_wrist_ft_sensor_.GetCurrentForceTorqueRaw( &l_wrist_fx_raw_N_, &l_wrist_fy_raw_N_, &l_wrist_fz_raw_N_, &l_wrist_tx_raw_Nm_,
				&l_wrist_ty_raw_Nm_, &l_wrist_tz_raw_Nm_);
	}

	if(r_wrist_ft_calib_start_)
	{
		r_wrist_ft_null_.coeffRef(0,0) += r_wrist_fx_raw_N_;
		r_wrist_ft_null_.coeffRef(1,0) += r_wrist_fy_raw_N_;
		r_wrist_ft_null_.coeffRef(2,0) += r_wrist_fz_raw_N_;
		r_wrist_ft_null_.coeffRef(3,0) += r_wrist_tx_raw_Nm_;
		r_wrist_ft_null_.coeffRef(4,0) += r_wrist_ty_raw_Nm_;
		r_wrist_ft_null_.coeffRef(5,0) += r_wrist_tz_raw_Nm_;

		if(current_wrist_ft_calib_time_ < control_cycle_msec_*0.001) {
			status_msg.data = "start_r_wrist_ft_calib";
			thormang3_wrist_ft_status_pub_.publish(status_msg);
		}
		else if(current_wrist_ft_calib_time_ >= wrist_ft_calib_duration_) {
			r_wrist_ft_null_ = r_wrist_ft_null_ / (current_wrist_ft_calib_time_ / (control_cycle_msec_*0.001));
		    r_wrist_ft_sensor_.SetScaleParam(1.0, r_wrist_ft_null_);
			status_msg.data = "finish_r_wrist_ft_calib";
			thormang3_wrist_ft_status_pub_.publish(status_msg);
			r_wrist_ft_calib_start_ = false;
		}
	}

	if(l_wrist_ft_calib_start_)
	{
		l_wrist_ft_null_.coeffRef(0,0) += l_wrist_fx_raw_N_;
		l_wrist_ft_null_.coeffRef(1,0) += l_wrist_fy_raw_N_;
		l_wrist_ft_null_.coeffRef(2,0) += l_wrist_fz_raw_N_;
		l_wrist_ft_null_.coeffRef(3,0) += l_wrist_tx_raw_Nm_;
		l_wrist_ft_null_.coeffRef(4,0) += l_wrist_ty_raw_Nm_;
		l_wrist_ft_null_.coeffRef(5,0) += l_wrist_tz_raw_Nm_;

		if(current_wrist_ft_calib_time_ < control_cycle_msec_*0.001) {
			status_msg.data = "start_l_wrist_ft_calib";
			thormang3_wrist_ft_status_pub_.publish(status_msg);
		}
		else if(current_wrist_ft_calib_time_ >= wrist_ft_calib_duration_) {
			l_wrist_ft_null_ = l_wrist_ft_null_ / (current_wrist_ft_calib_time_ / (control_cycle_msec_*0.001));
		    l_wrist_ft_sensor_.SetScaleParam(1.0, l_wrist_ft_null_);
			status_msg.data = "finish_r_wrist_ft_calib";
			thormang3_wrist_ft_status_pub_.publish(status_msg);
			l_wrist_ft_calib_start_ = false;
		}
	}

	if(r_wrist_ft_calib_start_ || l_wrist_ft_calib_start_)
	{
		current_wrist_ft_calib_time_ += control_cycle_msec_*0.001;
	}



    if(enable == false)
        return;
}
