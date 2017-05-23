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

#ifndef ATI_FT_SENSOR_ATI_FORCE_TORQUE_SENSOR_TWE_H_
#define ATI_FT_SENSOR_ATI_FORCE_TORQUE_SENSOR_TWE_H_


#include <ros/ros.h>
#include <ros/package.h>
#include <boost/thread.hpp>
#include <yaml-cpp/yaml.h>

#include <geometry_msgs/WrenchStamped.h>

#include "robotis_math/robotis_math.h"

namespace thormang3
{

class ATIForceTorqueSensorTWE
{
public:
  ATIForceTorqueSensorTWE();
  ~ATIForceTorqueSensorTWE();

  bool initialize(const std::string& ft_data_path,			  const std::string& ft_data_key,
                  const std::string& ft_frame_id,
                  const std::string& ft_raw_publish_name,	const std::string& ft_scaled_publish_name);

  void setScaleFactror(double ft_scale_factor);
  void setNullForceTorque(Eigen::MatrixXd _ft_null);
  void setScaleParam(double ft_scale_factor, Eigen::MatrixXd ft_null);


  void setCurrentVoltageOutput(double voltage0, double voltage1, double voltage2,
                               double voltage3, double voltage4, double voltage5);
  void setCurrentVoltageOutput(Eigen::MatrixXd voltage);

  Eigen::MatrixXd getCurrentForceTorqueRaw();
  Eigen::MatrixXd getCurrentForceTorqueScaled();

  void getCurrentForceTorqueRaw(double *force_x_N,   double *force_y_N,   double *force_z_N,
                                double *torque_x_Nm, double *torque_y_Nm, double *torque_z_Nm);
  void getCurrentForceTorqueScaled(double *force_x_N,   double *force_y_N,   double *force_z_N,
                                   double *torque_x_Nm, double *torque_y_Nm, double *torque_z_Nm);

  void setCurrentVoltageOutputPublish(double voltage0, double voltage1, double voltage2,
                                      double voltage3, double voltage4, double voltage5);
  void setCurrentVoltageOutputPublish(Eigen::MatrixXd voltage);

private:
  bool parseFTData(const std::string& ft_data_path, const std::string& ft_data_key);

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
};

}


#endif /* ATI_FT_SENSOR_ATI_FORCE_TORQUE_SENSOR_TWE_H_ */
