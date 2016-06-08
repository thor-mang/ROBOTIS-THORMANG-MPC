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

#ifndef THORMANG3_IMU_SENSOR_MODULE_H_
#define THORMANG3_IMU_SENSOR_MODULE_H_

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <boost/thread.hpp>

#include <sensor_msgs/Imu.h>

#include <robotis_framework_common/SensorModule.h>



namespace ROBOTIS
{
class ThorMang3ImuSensor : public SensorModule, public Singleton<ThorMang3ImuSensor>
{
public:
  ThorMang3ImuSensor();
  ~ThorMang3ImuSensor();

  /* ROS Topic Callback Functions */
  void ImuSensorCallback(const sensor_msgs::Imu::ConstPtr msg, const std::string& tag);

  void Initialize(const int control_cycle_msec, Robot* robot);
  void Process(std::map<std::string, Dynamixel* > dxls, std::map<std::string, Sensor*> sensors);

  void Stop();
  bool IsRunning();

  bool gazebo_mode;
  std::string gazebo_robot_name;

private:
  void WristForceTorqueSensorInitialize();

  void QueueThread();

  void PublishStatusMsg(unsigned int type, std::string msg);

  int control_cycle_msec_;
  boost::thread queue_thread_;
  boost::mutex imu_sensor_mutex_;

  ros::Publisher imu_status_pub_;
};
}


#endif /* THORMANG3_FEET_FORCE_TORQUE_SENSOR_MODULE_H_ */
