/*
 * wholebody_module.h
 *
 *  Created on: Aug 10, 2016
 *      Author: sch
 */

#ifndef THORMANG3_GRIPPER_MODULE_H_
#define THORMANG3_GRIPPER_MODULE_H_

#include <map>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/package.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/JointState.h>
#include <boost/thread.hpp>
#include <yaml-cpp/yaml.h>

#include <fstream>

#include "robotis_math/robotis_math.h"
#include "robotis_framework_common/motion_module.h"

#include "robotis_controller_msgs/JointCtrlModule.h"
#include "robotis_controller_msgs/StatusMsg.h"
#include "robotis_controller_msgs/SyncWriteItem.h"

namespace thormang3
{

#define NUM_GRIPPER_JOINTS  2

class GripperModule
  : public robotis_framework::MotionModule,
    public robotis_framework::Singleton<GripperModule>
{
private:
  double control_cycle_sec_;
  boost::thread  queue_thread_;
  boost::thread* tra_gene_tread_;

  /* sample subscriber & publisher */
  ros::Publisher status_msg_pub_;
  ros::Publisher set_ctrl_module_pub_;
  ros::Publisher goal_torque_limit_pub_;

  std::map<std::string, int> joint_name_to_id_;

  /* base parameters */
  bool is_moving_;

  Eigen::VectorXd present_joint_position_;
  Eigen::VectorXd present_joint_velocity_;
  Eigen::VectorXd goal_joint_position_;

  sensor_msgs::JointState goal_joint_pose_msg_;

  /* movement */
  double mov_time_;
  int all_time_steps_;
  int cnt_;

  Eigen::MatrixXd goal_joint_tra_;

  void queueThread();

  void setJointPoseMsgCallback(const sensor_msgs::JointState::ConstPtr& msg);
  void traGeneProcJointSpace();
  void setTorqueLimit();
  void setEndTrajectory();

public:
  GripperModule();
  virtual ~GripperModule();

  /* ROS Topic Callback Functions */
  void setModeMsgCallback(const std_msgs::String::ConstPtr& msg);

  void initialize(const int control_cycle_msec, robotis_framework::Robot *robot);
  void process(std::map<std::string, robotis_framework::Dynamixel *> dxls, std::map<std::string, double> sensors);

  void stop();
  bool isRunning();
  void publishStatusMsg(unsigned int type, std::string msg);
};

}

#endif /* THORMANG3_GRIPPER_MODULE_H_ */
