//=================================================================================================
// Copyright (c) 2016, Alexander Stumpf, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#ifndef THORMANG3_WRIST_FORCE_TORQUE_SENSOR_MODULE_H_
#define THORMANG3_WRIST_FORCE_TORQUE_SENSOR_MODULE_H_

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <boost/thread.hpp>
#include <std_msgs/String.h>
#include <geometry_msgs/WrenchStamped.h>

#include <fstream>

#include "robotis_math/robotis_math.h"

#include "thormang3_feet_ft_module_msgs/BothWrench.h"


#include "robotis_controller_msgs/StatusMsg.h"
#include "robotis_framework_common/sensor_module.h"

#include "thormang3_kinematics_dynamics/kinematics_dynamics.h"
#include "ati_ft_sensor/ati_force_torque_sensor.h"



namespace thormang3
{

class WristForceTorqueSensor : public robotis_framework::SensorModule, public robotis_framework::Singleton<WristForceTorqueSensor>
{
enum
{
  FT_NONE = 0,
  FT_AIR = 1,
  FT_GND = 2,
  FT_CALC = 3,
};

public:
  WristForceTorqueSensor();
  ~WristForceTorqueSensor();

  bool gazebo_mode_;
  std::string gazebo_robot_name_;

  double r_wrist_fx_raw_N,  r_wrist_fy_raw_N,  r_wrist_fz_raw_N;
  double r_wrist_tx_raw_Nm, r_wrist_ty_raw_Nm, r_wrist_tz_raw_Nm;
  double l_wrist_fx_raw_N,  l_wrist_fy_raw_N,  l_wrist_fz_raw_N;
  double l_wrist_tx_raw_Nm, l_wrist_ty_raw_Nm, l_wrist_tz_raw_Nm;

  double r_wrist_fx_scaled_N_,  r_wrist_fy_scaled_N_,  r_wrist_fz_scaled_N_;
  double r_wrist_tx_scaled_Nm_, r_wrist_ty_scaled_Nm_, r_wrist_tz_scaled_Nm_;
  double l_wrist_fx_scaled_N_,  l_wrist_fy_scaled_N_,  l_wrist_fz_scaled_N_;
  double l_wrist_tx_scaled_Nm_, l_wrist_ty_scaled_Nm_, l_wrist_tz_scaled_Nm_;

  void  initialize(const int control_cycle_msec, robotis_framework::Robot *robot) override;
  void  process(std::map<std::string, robotis_framework::Dynamixel *> dxls, std::map<std::string, robotis_framework::Sensor *> sensors) override;

private:
  void queueThread();

  void initializeWristForceTorqueSensor();
  void saveFTCalibrationData(const std::string &path);

  void ftSensorCalibrationCommandCallback(const std_msgs::String::ConstPtr& msg);
  void gazeboFTSensorCallback(const geometry_msgs::WrenchStamped::ConstPtr msg);

  void publishStatusMsg(unsigned int type, std::string msg);

  int             control_cycle_msec_;
  boost::thread   queue_thread_;
  boost::mutex    publish_mutex_;
  boost::mutex    ft_sensor_mutex_;

  KinematicsDynamics* thormang3_kd_;

  ATIForceTorqueSensorTWE r_wrist_ft_sensor_;
  ATIForceTorqueSensorTWE l_wrist_ft_sensor_;

  Eigen::MatrixXd r_wrist_ft_air_, l_wrist_ft_air_;

  bool	has_ft_air_;

  bool exist_r_arm_wr_p;
  bool exist_r_arm_wr_y;
  bool exist_l_arm_wr_p;
  bool exist_l_arm_wr_y;

  double r_wrist_ft_current_voltage_[6];
  double l_wrist_ft_current_voltage_[6];

  ros::Publisher  thormang3_wrist_ft_status_pub_;

  int 	ft_command_;
  int		ft_period_;
  int		ft_get_count_;
};
}

#endif /* THORMANG3_FEET_FORCE_TORQUE_SENSOR_MODULE_H_ */
