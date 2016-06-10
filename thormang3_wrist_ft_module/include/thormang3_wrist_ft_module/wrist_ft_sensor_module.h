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

#include "robotis_math/RobotisMath.h"

#include "thormang3_feet_ft_module_msgs/BothWrench.h"


#include "thormang3_kinematics_dynamics/ThorMang3KinematicsDynamics.h"
#include "robotis_controller_msgs/StatusMsg.h"
#include "robotis_framework_common/SensorModule.h"

#include "ati_ft_sensor/ATIForceTorqueSensor.h"



namespace ROBOTIS
{

class WristForceTorqueSensor : public SensorModule, public Singleton<WristForceTorqueSensor>
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

  /* ROS Topic Callback Functions */
  void  GazeboFTSensorCallback(const geometry_msgs::WrenchStamped::ConstPtr msg);

  void  Initialize(const int control_cycle_msec, Robot *robot);
  void  Process(std::map<std::string, Dynamixel *> dxls, std::map<std::string, Sensor *> sensors);

  void	Stop();
  bool	IsRunning();

  bool gazebo_mode;
  std::string gazebo_robot_name;

  double r_wrist_fx_raw_N,  r_wrist_fy_raw_N,  r_wrist_fz_raw_N;
  double r_wrist_tx_raw_Nm, r_wrist_ty_raw_Nm, r_wrist_tz_raw_Nm;
  double l_wrist_fx_raw_N,  l_wrist_fy_raw_N,  l_wrist_fz_raw_N;
  double l_wrist_tx_raw_Nm, l_wrist_ty_raw_Nm, l_wrist_tz_raw_Nm;

private:
  void WristForceTorqueSensorInitialize();

  void QueueThread();

  void FTSensorCalibrationCommandCallback(const std_msgs::String::ConstPtr& msg);
  void PublishStatusMsg(unsigned int type, std::string msg);

  int             control_cycle_msec_;
  boost::thread   queue_thread_;
  boost::mutex    publish_mutex_;
  boost::mutex    ft_sensor_mutex_;

  ThorMang3KinematicsDynamics* thormang3_kd_;

  ATIForceTorqueSensorTWE r_wrist_ft_sensor_;
  ATIForceTorqueSensorTWE l_wrist_ft_sensor_;

  Eigen::MatrixXd r_wrist_ft_air_, l_wrist_ft_air_;

  bool	has_ft_air_;

  bool exist_r_wrist_an_r_;
  bool exist_r_wrist_an_p_;
  bool exist_l_wrist_an_r_;
  bool exist_l_wrist_an_p_;

  double r_wrist_ft_current_voltage_[6];
  double l_wrist_ft_current_voltage_[6];

  ros::Publisher  thormang3_wrist_ft_status_pub_;

  int 	ft_command_;
  int		ft_period_;
  int		ft_get_count_;

};
}


#endif /* THORMANG3_FEET_FORCE_TORQUE_SENSOR_MODULE_H_ */
