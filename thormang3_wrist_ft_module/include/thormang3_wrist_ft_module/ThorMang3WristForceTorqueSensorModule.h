/*
 * TestMotionModule.h
 *
 *  Created on: 2016. 1. 25.
 *      Author: zerom
 */

#ifndef ROBOTIS_THORMANG3_WRIST_FORCETORQUESENSOR_H_
#define ROBOTIS_THORMANG3_WRIST_FORCETORQUESENSOR_H_




#include <ros/callback_queue.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <boost/thread.hpp>

#include "ATIForceTorqueSensor.h"
#include "robotis_framework_common/MotionModule.h"


namespace ROBOTIS
{

class ThorMang3WristForceTorqueSensor : public MotionModule
{
private:
    static ThorMang3WristForceTorqueSensor *unique_instance_;

    int             control_cycle_msec_;
    boost::thread   queue_thread_;

    ThorMang3WristForceTorqueSensor();


    ATIForceTorqueSensor r_wrist_ft_sensor_, l_wrist_ft_sensor_;
    double r_wrist_ft_current_voltage_[6], l_wrist_ft_current_voltage_[6];
	double r_wrist_fx_raw_N_, r_wrist_fy_raw_N_, r_wrist_fz_raw_N_, r_wrist_tx_raw_Nm_, r_wrist_ty_raw_Nm_, r_wrist_tz_raw_Nm_;
	double l_wrist_fx_raw_N_, l_wrist_fy_raw_N_, l_wrist_fz_raw_N_, l_wrist_tx_raw_Nm_, l_wrist_ty_raw_Nm_, l_wrist_tz_raw_Nm_;
	double r_wrist_fx_scaled_N_,  r_wrist_fy_scaled_N_,  r_wrist_fz_scaled_N_;
	double r_wrist_tx_scaled_Nm_, r_wrist_ty_scaled_Nm_, r_wrist_tz_scaled_Nm_;
	double l_wrist_fx_scaled_N_,  l_wrist_fy_scaled_N_,  l_wrist_fz_scaled_N_;
	double l_wrist_tx_scaled_Nm_, l_wrist_ty_scaled_Nm_, l_wrist_tz_scaled_Nm_;
	Eigen::MatrixXd r_wrist_ft_null_, l_wrist_ft_null_;

	ros::Publisher thormang3_wrist_ft_status_pub_;
	std_msgs::String status_msg;
	bool	r_wrist_ft_calib_start_, l_wrist_ft_calib_start_;
	double	current_wrist_ft_calib_time_, wrist_ft_calib_duration_;

    void	QueueThread();

    void	WristFTCalibratingDurationCallback(const std_msgs::Float64::ConstPtr& msg);
    void	WristFTCalibrationCommandCallback(const std_msgs::String::ConstPtr& msg);

public:
    virtual ~ThorMang3WristForceTorqueSensor();

    static ThorMang3WristForceTorqueSensor *GetInstance() { return unique_instance_; }

    /* ROS Topic Callback Functions */
    //void    TopicCallback(const std_msgs::Int16::ConstPtr &msg);

    void    Initialize(const int control_cycle_msec);
    void 	WristForceTorqueSensorInitialize();
    void    Process(std::map<std::string, Dynamixel *> dxls);
};

}


#endif /* ROBOTIS_THORMANG_TEST_MOTION_MODULE_INCLUDE_TEST_MOTION_MODULE_TESTMOTIONMODULE_H_ */
