/*
 * TestMotionModule.h
 *
 *  Created on: 2016. 1. 25.
 *      Author: zerom
 */

#ifndef ROBOTIS_THORMANG_WALKING_MODULE_INCLUDE_WALKING_MODULE_FORCETORQUESENSOR_H_
#define ROBOTIS_THORMANG_WALKING_MODULE_INCLUDE_WALKING_MODULE_FORCETORQUESENSOR_H_


#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <yaml-cpp/yaml.h>

#include "math/walking_module_math.h"


namespace ROBOTIS
{

class ForceTorqueSensor
{
private:
	Eigen::MatrixXd ft_coeff_mat_;
	Eigen::MatrixXd ft_unload_volatge_;
	Eigen::MatrixXd ft_current_volatge_;
	Eigen::MatrixXd ft_null_;
	Eigen::MatrixXd ft_raw_;
	Eigen::MatrixXd ft_scaled_;


	double ft_scale_factor_;

	std::string ft_frame_id_;
	std::string ft_raw_publish_name_;
	std::string ft_scaled_publish_name_;

	ros::Publisher ft_raw_pub_;
	ros::Publisher ft_scaled_pub_;

	geometry_msgs::WrenchStamped ft_raw_msg_;
	geometry_msgs::WrenchStamped ft_scaled_msg_;

public:
	ForceTorqueSensor();
	~ForceTorqueSensor();

	bool Initialize(const std::string& _ft_data_path,			const std::string& _ft_data_key,
					const std::string& _ft_frame_id,
					const std::string& _ft_raw_publish_name,	const std::string& _ft_scaled_publish_name);

	void SetScaleFactror(double _ft_scale_factor);
	void SetNullForceTorque(Eigen::MatrixXd _ft_null);
	void SetScaleParam(double _ft_scale_factor, Eigen::MatrixXd _ft_null);

	bool parseFTData(const std::string& _ft_data_path, const std::string& _ft_data_key);


	void SetCurrentVoltageOutput(double _voltage0, double _voltage1, double _voltage2,
								 double _voltage3, double _voltage4, double _voltage5);
	void SetCurrentVoltageOutput(Eigen::MatrixXd _voltage);

	Eigen::MatrixXd GetCurrentForceTorqueRaw();
	Eigen::MatrixXd GetCurrentForceTorqueScaled();

	void GetCurrentForceTorqueRaw(double* _force_x_N,   double* _force_y_N,   double* _force_z_N,
							      double* _torque_x_Nm, double* _torque_y_Nm, double* _torque_z_Nm);
	void GetCurrentForceTorqueScaled(double* _force_x_N,   double* _force_y_N,   double* _force_z_N,
							     	 double* _torque_x_Nm, double* _torque_y_Nm, double* _torque_z_Nm);

	void SetCurrentVoltageOutputPublishForceTorque(double _voltage0, double _voltage1, double _voltage2,
			 	 	 	 	 	 	 	 	 	   double _voltage3, double _voltage4, double _voltage5);
	void SetCurrentVoltageOutputPublish(Eigen::MatrixXd _voltage);
};

}


#endif /* ROBOTIS_THORMANG_TEST_MOTION_MODULE_INCLUDE_TEST_MOTION_MODULE_TESTMOTIONMODULE_H_ */
