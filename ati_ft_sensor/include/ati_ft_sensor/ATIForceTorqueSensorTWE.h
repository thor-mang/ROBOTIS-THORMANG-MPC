/*
 * ATIForceTorqueSensorTWE.h
 *
 *  Created on: 2016. 1. 25.
 *      Author: zerom
 */

#ifndef ATI_FORCE_TORQUE_SENSOR_TWE_H_
#define ATI_FORCE_TORQUE_SENSOR_TWE_H_


#include <ros/ros.h>
#include <ros/package.h>
#include <boost/thread.hpp>
#include <geometry_msgs/WrenchStamped.h>
#include <yaml-cpp/yaml.h>
#include "robotis_math/RobotisMath.h"

namespace ROBOTIS
{

class ATIForceTorqueSensorTWE
{
private:
	Eigen::MatrixXd ft_coeff_mat_;
	Eigen::MatrixXd ft_unload_volatge_;
	Eigen::MatrixXd ft_current_volatge_;
	Eigen::MatrixXd ft_null_;
	Eigen::MatrixXd ft_raw_;
	Eigen::MatrixXd ft_scaled_;

	boost::mutex    ft_scale_param_mutex_;


	double ft_scale_factor_;

	std::string ft_frame_id_;
	std::string ft_raw_publish_name_;
	std::string ft_scaled_publish_name_;

	bool is_ft_raw_published_;
	bool is_ft_scaled_published_;


	ros::Publisher ft_raw_pub_;
	ros::Publisher ft_scaled_pub_;

	geometry_msgs::WrenchStamped ft_raw_msg_;
	geometry_msgs::WrenchStamped ft_scaled_msg_;

	bool parseFTData(const std::string& ft_data_path, const std::string& ft_data_key);

public:
	ATIForceTorqueSensorTWE();
	~ATIForceTorqueSensorTWE();

	bool Initialize(const std::string& ft_data_path,			const std::string& ft_data_key,
					const std::string& ft_frame_id,
					const std::string& ft_raw_publish_name,	const std::string& ft_scaled_publish_name);

	void SetScaleFactror(double ft_scale_factor);
	void SetNullForceTorque(Eigen::MatrixXd _ft_null);
	void SetScaleParam(double ft_scale_factor, Eigen::MatrixXd ft_null);


	void SetCurrentVoltageOutput(double voltage0, double voltage1, double voltage2,
								 double voltage3, double voltage4, double voltage5);
	void SetCurrentVoltageOutput(Eigen::MatrixXd voltage);

	Eigen::MatrixXd GetCurrentForceTorqueRaw();
	Eigen::MatrixXd GetCurrentForceTorqueScaled();

	void GetCurrentForceTorqueRaw(double* force_x_N,   double* force_y_N,   double* force_z_N,
							      double* torque_x_Nm, double* torque_y_Nm, double* torque_z_Nm);
	void GetCurrentForceTorqueScaled(double* force_x_N,   double* force_y_N,   double* force_z_N,
							     	 double* torque_x_Nm, double* torque_y_Nm, double* torque_z_Nm);

	void SetCurrentVoltageOutputPublishForceTorque(double voltage0, double voltage1, double voltage2,
			 	 	 	 	 	 	 	 	 	   double voltage3, double voltage4, double voltage5);
	void SetCurrentVoltageOutputPublish(Eigen::MatrixXd voltage);
};

}


#endif /* ATI_FORCE_TORQUE_SENSOR_TWE_H_ */
