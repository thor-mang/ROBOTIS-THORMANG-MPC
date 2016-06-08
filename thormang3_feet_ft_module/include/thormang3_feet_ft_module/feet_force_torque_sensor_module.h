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

/*
 * feet_force_torque_sensor_module.h
 *
 *  Created on: 2016. 3. 22.
 *      Author: Jay Song
 */

#ifndef THORMANG3_FEET_FT_MODULE_FEET_FORCE_TORQUE_SENSOR_MODULE_H_
#define THORMANG3_FEET_FT_MODULE_FEET_FORCE_TORQUE_SENSOR_MODULE_H_

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <boost/thread.hpp>
#include <std_msgs/String.h>

#include <fstream>

#include "robotis_math/robotis_math.h"
#include "robotis_controller_msgs/StatusMsg.h"
#include "robotis_framework_common/sensor_module.h"

#include "thormang3_feet_ft_module_msgs/BothWrench.h"
#include "thormang3_kinematics_dynamics/kinematics_dynamics.h"
#include "ati_ft_sensor/ati_force_torque_sensor.h"



namespace thormang3
{

class FeetForceTorqueSensor : public robotis_framework::SensorModule, public robotis_framework::Singleton<FeetForceTorqueSensor>
{
public:
  FeetForceTorqueSensor();
  ~FeetForceTorqueSensor();


  double r_foot_fx_raw_N_,  r_foot_fy_raw_N_,  r_foot_fz_raw_N_;
  double r_foot_tx_raw_Nm_, r_foot_ty_raw_Nm_, r_foot_tz_raw_Nm_;
  double l_foot_fx_raw_N_,  l_foot_fy_raw_N_,  l_foot_fz_raw_N_;
  double l_foot_tx_raw_Nm_, l_foot_ty_raw_Nm_, l_foot_tz_raw_Nm_;

  double r_foot_fx_scaled_N_,  r_foot_fy_scaled_N_,  r_foot_fz_scaled_N_;
  double r_foot_tx_scaled_Nm_, r_foot_ty_scaled_Nm_, r_foot_tz_scaled_Nm_;
  double l_foot_fx_scaled_N_,  l_foot_fy_scaled_N_,  l_foot_fz_scaled_N_;
  double l_foot_tx_scaled_Nm_, l_foot_ty_scaled_Nm_, l_foot_tz_scaled_Nm_;

  void initialize(const int control_cycle_msec, robotis_framework::Robot *robot);
  void process(std::map<std::string, robotis_framework::Dynamixel *> dxls, std::map<std::string, robotis_framework::Sensor *> sensors);

private:
  void queueThread();

  void initializeFeetForceTorqueSensor();
  void saveFTCalibrationData(const std::string &path);

  void ftSensorCalibrationCommandCallback(const std_msgs::String::ConstPtr& msg);
  void publishStatusMsg(unsigned int type, std::string msg);
  void publishBothFTData(int type, Eigen::MatrixXd &ft_right, Eigen::MatrixXd &ft_left);


  int             control_cycle_msec_;
  boost::thread   queue_thread_;
  boost::mutex    publish_mutex_;


  KinematicsDynamics* thormang3_kd_;


  bool exist_r_leg_an_r_, exist_r_leg_an_p_;
  bool exist_l_leg_an_r_, exist_l_leg_an_p_;

  ATIForceTorqueSensorTWE r_foot_ft_sensor_;
  ATIForceTorqueSensorTWE l_foot_ft_sensor_;

  Eigen::MatrixXd r_foot_ft_air_, l_foot_ft_air_;
  Eigen::MatrixXd r_foot_ft_gnd_, l_foot_ft_gnd_;

  double r_foot_ft_current_voltage_[6];
  double l_foot_ft_current_voltage_[6];


  double total_mass_;
  double r_foot_ft_scale_factor_, l_foot_ft_scale_factor_;

  bool  has_ft_air_;
  bool  has_ft_gnd_;
  int   ft_command_;
  int   ft_period_;
  int   ft_get_count_;

  const int FT_NONE;
  const int FT_AIR;
  const int FT_GND;
  const int FT_CALC;


  ros::Publisher  thormang3_foot_ft_status_pub_;
  ros::Publisher  thormang3_foot_ft_both_ft_pub_;
};


}


#endif /* THORMANG3_FEET_FT_MODULE_FEET_FORCE_TORQUE_SENSOR_MODULE_H_ */
