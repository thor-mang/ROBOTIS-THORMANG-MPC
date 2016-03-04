/*
 * ForceTorqueSensor.cpp
 *
 *  Created on: 2016. 2. 17.
 *      Author: HJSONG
 */




#include "thormang3_walking_module/ForceTorqueSensor.h"

namespace ROBOTIS
{


ForceTorqueSensor::ForceTorqueSensor()
{
	ft_coeff_mat_ = Eigen::MatrixXd::Zero(6,6);
	ft_unload_volatge_.resize(6, 1);
	ft_unload_volatge_.fill(3.3*0.5);

	ft_current_volatge_.resize(6, 1);
	ft_current_volatge_.fill(3.3*0.5);

	ft_null_.resize(6, 1);
	ft_null_.fill(0);
	ft_raw_.resize(6, 1);
	ft_raw_.fill(0);
	ft_scaled_.resize(6, 1);
	ft_scaled_.fill(0);

	ft_scale_factor_ = 1.0;

	ft_frame_id_ = "";
	ft_raw_publish_name_    ="";
	ft_scaled_publish_name_ ="";
}


ForceTorqueSensor::~ForceTorqueSensor()
{

}


bool ForceTorqueSensor::Initialize(const std::string& _ft_data_path,		const std::string& _ft_data_key,
								   const std::string& _ft_frame_id,
								   const std::string& _ft_raw_publish_name, const std::string& _ft_scaled_publish_name)
{
	if((_ft_raw_publish_name == "") || (_ft_scaled_publish_name == ""))
		return false;

	ft_frame_id_			= _ft_frame_id;
	ft_raw_publish_name_	= _ft_raw_publish_name;
	ft_scaled_publish_name_	= _ft_scaled_publish_name;

	ros::NodeHandle _nh;

	ft_raw_msg_.header.frame_id    = ft_frame_id_;
	ft_scaled_msg_.header.frame_id = ft_frame_id_;

	ft_raw_pub_		= _nh.advertise<geometry_msgs::WrenchStamped>(ft_raw_publish_name_,	1);
	ft_scaled_pub_	= _nh.advertise<geometry_msgs::WrenchStamped>(ft_scaled_publish_name_,	1);

	return parseFTData(_ft_data_path, _ft_data_key);;
}

void ForceTorqueSensor::SetScaleFactror(double _ft_scale_factor)
{
	ft_scale_factor_ = _ft_scale_factor;
}

void ForceTorqueSensor::SetNullForceTorque(Eigen::MatrixXd _ft_null)
{
	if( (_ft_null.rows() != 6) || (_ft_null.cols() != 1) ){
		ROS_ERROR("Invalid ft null size");
		return;
	}

	ft_null_ = _ft_null;
}

void ForceTorqueSensor::SetScaleParam(double _ft_scale_factor, Eigen::MatrixXd _ft_null)
{
	SetScaleFactror(_ft_scale_factor);
	SetNullForceTorque(_ft_null);
}

bool ForceTorqueSensor::parseFTData(const std::string& _ft_data_path, const std::string& _ft_data_key)
{
	std::string _ft_mat_key		= _ft_data_key;
	std::string _ft_unload_key	= _ft_data_key + "_unload";

	YAML::Node doc;
	try
	{
		// load yaml
		doc = YAML::LoadFile(_ft_data_path.c_str());
	}
	catch(const std::exception& e)
	{
		ROS_ERROR("WalkingModule : Fail to load ft_data yaml file.");
		return false;
	}

	std::vector<double> _ft;

	//todo : check the key is availabe or not
	_ft = doc[_ft_mat_key].as< std::vector<double> >();
	ft_coeff_mat_ = Eigen::Map<Eigen::MatrixXd>(_ft.data(), 6, 6);
	ft_coeff_mat_.transposeInPlace();
	std::cout << "[" <<_ft_mat_key << "_mat]" <<std::endl;
	std::cout << ft_coeff_mat_ <<std::endl;

	_ft.clear();
	//todo : check the key is availabe or not
	_ft = doc[_ft_unload_key].as< std::vector<double> >();
	ft_unload_volatge_ = Eigen::Map<Eigen::MatrixXd>(_ft.data(), 6, 1);
	std::cout << "[" <<_ft_unload_key << "]"	<< std::endl;
	std::cout << ft_unload_volatge_.transpose() << std::endl;

	return true;
}


void ForceTorqueSensor::SetCurrentVoltageOutput(double _voltage0, double _voltage1, double _voltage2,
							 double _voltage3, double _voltage4, double _voltage5)
{
	ft_current_volatge_.coeffRef(0, 0) = _voltage0;
	ft_current_volatge_.coeffRef(1, 0) = _voltage1;
	ft_current_volatge_.coeffRef(2, 0) = _voltage2;
	ft_current_volatge_.coeffRef(3, 0) = _voltage3;
	ft_current_volatge_.coeffRef(4, 0) = _voltage4;
	ft_current_volatge_.coeffRef(5, 0) = _voltage5;


	ft_raw_		= 					  ft_coeff_mat_ * (ft_current_volatge_ - ft_unload_volatge_);
	ft_scaled_	= ft_scale_factor_ * (ft_raw_ - ft_null_ );

	ros::Time _now = ros::Time::now();
	ft_raw_msg_.header.stamp		= _now;
	ft_scaled_msg_.header.stamp		= _now;
	ft_raw_msg_.wrench.force.x		= ft_raw_.coeff(0,0);
	ft_raw_msg_.wrench.force.y		= ft_raw_.coeff(1,0);
	ft_raw_msg_.wrench.force.z		= ft_raw_.coeff(2,0);
	ft_raw_msg_.wrench.torque.x		= ft_raw_.coeff(3,0);
	ft_raw_msg_.wrench.torque.y		= ft_raw_.coeff(4,0);
	ft_raw_msg_.wrench.torque.z		= ft_raw_.coeff(5,0);

	ft_scaled_msg_.wrench.force.x	= ft_scaled_.coeff(0,0);
	ft_scaled_msg_.wrench.force.y	= ft_scaled_.coeff(1,0);
	ft_scaled_msg_.wrench.force.z	= ft_scaled_.coeff(2,0);
	ft_scaled_msg_.wrench.torque.x	= ft_scaled_.coeff(3,0);
	ft_scaled_msg_.wrench.torque.y	= ft_scaled_.coeff(4,0);
	ft_scaled_msg_.wrench.torque.z	= ft_scaled_.coeff(5,0);
}

void ForceTorqueSensor::SetCurrentVoltageOutput(Eigen::MatrixXd _voltage)
{
	if((_voltage.rows() != 6) || (_voltage.cols() != 1)) {
		ROS_ERROR("Invalid voltage size");
		return;
	}

	SetCurrentVoltageOutput(_voltage.coeff(0,0), _voltage.coeff(1,0), _voltage.coeff(2,0),
							_voltage.coeff(3,0), _voltage.coeff(4,0), _voltage.coeff(5,0));
}

Eigen::MatrixXd ForceTorqueSensor::GetCurrentForceTorqueRaw()
{
	return ft_raw_;
}

Eigen::MatrixXd ForceTorqueSensor::GetCurrentForceTorqueScaled()
{
	return ft_scaled_;
}

void ForceTorqueSensor::GetCurrentForceTorqueRaw(double* _force_x_N,   double* _force_y_N,   double* _force_z_N,
						      	  	  	  	  	 double* _torque_x_Nm, double* _torque_y_Nm, double* _torque_z_Nm)
{
	*_force_x_N   = ft_raw_.coeff(0,0);
	*_force_y_N   = ft_raw_.coeff(1,0);
	*_force_z_N   = ft_raw_.coeff(2,0);
	*_torque_x_Nm = ft_raw_.coeff(3,0);
	*_torque_y_Nm = ft_raw_.coeff(4,0);
	*_torque_z_Nm = ft_raw_.coeff(5,0);
}

void ForceTorqueSensor::GetCurrentForceTorqueScaled(double* _force_x_N,   double* _force_y_N,   double* _force_z_N,
						     	 	 	 	 	 	double* _torque_x_Nm, double* _torque_y_Nm, double* _torque_z_Nm)
{
	*_force_x_N   = ft_scaled_.coeff(0,0);
	*_force_y_N   = ft_scaled_.coeff(1,0);
	*_force_z_N   = ft_scaled_.coeff(2,0);
	*_torque_x_Nm = ft_scaled_.coeff(3,0);
	*_torque_y_Nm = ft_scaled_.coeff(4,0);
	*_torque_z_Nm = ft_scaled_.coeff(5,0);
}

void ForceTorqueSensor::SetCurrentVoltageOutputPublishForceTorque(double _voltage0, double _voltage1, double _voltage2,
		 	 	 	 	 	 	 	 	 	   	   	   	   	   	  double _voltage3, double _voltage4, double _voltage5)
{
	SetCurrentVoltageOutput(_voltage0, _voltage1, _voltage2, _voltage3, _voltage4, _voltage5);

	ft_raw_pub_.publish(ft_raw_msg_);
	ft_scaled_pub_.publish(ft_scaled_msg_);
}

void ForceTorqueSensor::SetCurrentVoltageOutputPublish(Eigen::MatrixXd _voltage)
{
	if((_voltage.rows() != 6) || (_voltage.cols() != 1)) {
		ROS_ERROR("Invalid voltage size");
		return;
	}

	SetCurrentVoltageOutput(_voltage);
	ft_raw_pub_.publish(ft_raw_msg_);
	ft_scaled_pub_.publish(ft_scaled_msg_);
}














}
