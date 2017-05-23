/*******************************************************************************
 * Copyright (c) 2016, ROBOTIS CO., LTD.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of ROBOTIS nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

#include "ati_ft_sensor/ati_force_torque_sensor_twe.h"

using namespace thormang3;


ATIForceTorqueSensorTWE::ATIForceTorqueSensorTWE()
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

	is_ft_raw_published_     = false;
	is_ft_scaled_published_  = false;
}


ATIForceTorqueSensorTWE::~ATIForceTorqueSensorTWE()
{

}


bool ATIForceTorqueSensorTWE::initialize(const std::string& ft_data_path, const std::string& ft_data_key,
								   const std::string& ft_frame_id,
								   const std::string& ft_raw_publish_name, const std::string& ft_scaled_publish_name)
{
	ft_frame_id_			= ft_frame_id;
	ft_raw_publish_name_	= ft_raw_publish_name;
	ft_scaled_publish_name_	= ft_scaled_publish_name;

	ros::NodeHandle _nh;

	ft_raw_msg_.header.frame_id    = ft_frame_id_;
	ft_scaled_msg_.header.frame_id = ft_frame_id_;

	if(ft_raw_publish_name_ != "") {
		ft_raw_pub_		= _nh.advertise<geometry_msgs::WrenchStamped>(ft_raw_publish_name_,	1);
		is_ft_raw_published_ = true;
	}

	if(ft_scaled_publish_name_ != "") {
		ft_scaled_pub_	= _nh.advertise<geometry_msgs::WrenchStamped>(ft_scaled_publish_name_,	1);
		is_ft_scaled_published_ = true;
	}

	return parseFTData(ft_data_path, ft_data_key);;
}

void ATIForceTorqueSensorTWE::setScaleFactror(double ft_scale_factor)
{
	ft_scale_param_mutex_.lock();
	ft_scale_factor_ = ft_scale_factor;
	ft_scale_param_mutex_.unlock();
}

void ATIForceTorqueSensorTWE::setNullForceTorque(Eigen::MatrixXd ft_null)
{
	if( (ft_null.rows() != 6) || (ft_null.cols() != 1) ){
		ROS_ERROR("Invalid ft null size");
		return;
	}

	ft_scale_param_mutex_.lock();
	ft_null_ = ft_null;
	ft_scale_param_mutex_.unlock();
}

void ATIForceTorqueSensorTWE::setScaleParam(double ft_scale_factor, Eigen::MatrixXd ft_null)
{
	setScaleFactror(ft_scale_factor);
	setNullForceTorque(ft_null);
}

bool ATIForceTorqueSensorTWE::parseFTData(const std::string& ft_data_path, const std::string& ft_data_key)
{
	std::string _ft_mat_key		= ft_data_key + "_calibration_matrix";
	std::string _ft_unload_key	= ft_data_key + "_unload";

	YAML::Node doc;

	doc = YAML::LoadFile(ft_data_path.c_str());

	std::vector<double> _ft;

	_ft = doc[_ft_mat_key].as< std::vector<double> >();
	ft_coeff_mat_ = Eigen::Map<Eigen::MatrixXd>(_ft.data(), 6, 6);
	ft_coeff_mat_.transposeInPlace();
	std::cout << "[" <<_ft_mat_key << "_mat]" <<std::endl;
	std::cout << ft_coeff_mat_ <<std::endl;

	_ft.clear();
	_ft = doc[_ft_unload_key].as< std::vector<double> >();
	ft_unload_volatge_ = Eigen::Map<Eigen::MatrixXd>(_ft.data(), 6, 1);
	std::cout << "[" <<_ft_unload_key << "]"	<< std::endl;
	std::cout << ft_unload_volatge_.transpose() << std::endl;

	return true;
}


void ATIForceTorqueSensorTWE::setCurrentVoltageOutput(double voltage0, double voltage1, double voltage2,
							 	 	 	 	 	 	  double voltage3, double voltage4, double voltage5)
{
	ft_current_volatge_.coeffRef(0, 0) = voltage0;
	ft_current_volatge_.coeffRef(1, 0) = voltage1;
	ft_current_volatge_.coeffRef(2, 0) = voltage2;
	ft_current_volatge_.coeffRef(3, 0) = voltage3;
	ft_current_volatge_.coeffRef(4, 0) = voltage4;
	ft_current_volatge_.coeffRef(5, 0) = voltage5;


	ft_raw_		= 					  ft_coeff_mat_ * (ft_current_volatge_ - ft_unload_volatge_);

	ft_scale_param_mutex_.lock();
	ft_scaled_	= ft_scale_factor_ * (ft_raw_ - ft_null_ );
	ft_scale_param_mutex_.unlock();

	ft_raw_msg_.header.stamp		= ros::Time::now();
	ft_scaled_msg_.header.stamp		= ft_raw_msg_.header.stamp;
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

void ATIForceTorqueSensorTWE::setCurrentVoltageOutput(Eigen::MatrixXd voltage)
{
	if((voltage.rows() != 6) || (voltage.cols() != 1)) {
		ROS_ERROR("Invalid voltage size");
		return;
	}

	setCurrentVoltageOutput(voltage.coeff(0,0), voltage.coeff(1,0), voltage.coeff(2,0),
							voltage.coeff(3,0), voltage.coeff(4,0), voltage.coeff(5,0));
}

Eigen::MatrixXd ATIForceTorqueSensorTWE::getCurrentForceTorqueRaw()
{
	return ft_raw_;
}

Eigen::MatrixXd ATIForceTorqueSensorTWE::getCurrentForceTorqueScaled()
{
	return ft_scaled_;
}

void ATIForceTorqueSensorTWE::getCurrentForceTorqueRaw(double* force_x_N,   double* force_y_N,   double* force_z_N,
						      	  	  	  	  	 double* torque_x_Nm, double* torque_y_Nm, double* torque_z_Nm)
{
	*force_x_N   = ft_raw_.coeff(0,0);
	*force_y_N   = ft_raw_.coeff(1,0);
	*force_z_N   = ft_raw_.coeff(2,0);
	*torque_x_Nm = ft_raw_.coeff(3,0);
	*torque_y_Nm = ft_raw_.coeff(4,0);
	*torque_z_Nm = ft_raw_.coeff(5,0);
}

void ATIForceTorqueSensorTWE::getCurrentForceTorqueScaled(double* force_x_N,   double* force_y_N,   double* force_z_N,
						     	 	 	 	 	 	   double* torque_x_Nm, double* torque_y_Nm, double* torque_z_Nm)
{
	*force_x_N   = ft_scaled_.coeff(0,0);
	*force_y_N   = ft_scaled_.coeff(1,0);
	*force_z_N   = ft_scaled_.coeff(2,0);
	*torque_x_Nm = ft_scaled_.coeff(3,0);
	*torque_y_Nm = ft_scaled_.coeff(4,0);
	*torque_z_Nm = ft_scaled_.coeff(5,0);
}

void ATIForceTorqueSensorTWE::setCurrentVoltageOutputPublish(double voltage0, double voltage1, double voltage2,
		 	 	 	 	 	 	 	 	 	   	   	   	   	   	  double voltage3, double voltage4, double voltage5)
{
	setCurrentVoltageOutput(voltage0, voltage1, voltage2, voltage3, voltage4, voltage5);

	if(is_ft_raw_published_)
		ft_raw_pub_.publish(ft_raw_msg_);

	if(is_ft_scaled_published_)
		ft_scaled_pub_.publish(ft_scaled_msg_);
}

void ATIForceTorqueSensorTWE::setCurrentVoltageOutputPublish(Eigen::MatrixXd voltage)
{
	if((voltage.rows() != 6) || (voltage.cols() != 1)) {
		ROS_ERROR("Invalid voltage size");
		return;
	}

	setCurrentVoltageOutput(voltage);

	if(is_ft_raw_published_)
		ft_raw_pub_.publish(ft_raw_msg_);

	if(is_ft_scaled_published_)
		ft_scaled_pub_.publish(ft_scaled_msg_);
}
