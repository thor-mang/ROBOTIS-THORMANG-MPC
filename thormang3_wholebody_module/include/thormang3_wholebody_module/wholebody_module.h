/*
 * wholebody_module.h
 *
 *  Created on: Aug 10, 2016
 *      Author: sch
 */

#ifndef THORMANG3_WHOLEBODY_MODULE_H_
#define THORMANG3_WHOLEBODY_MODULE_H_

#include <map>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/package.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Wrench.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <boost/thread.hpp>
#include <yaml-cpp/yaml.h>

#include <fstream>

#include "robotis_math/robotis_math.h"
#include "robotis_framework_common/motion_module.h"

#include "thormang3_kinematics_dynamics/kinematics_dynamics.h"
#include "thormang3_balance_control/thormang3_balance_control.h"

#include "robotis_controller_msgs/JointCtrlModule.h"
#include "robotis_controller_msgs/StatusMsg.h"

#include "thormang3_wholebody_module_msgs/JointPose.h"
#include "thormang3_wholebody_module_msgs/KinematicsPose.h"

#include "thormang3_wholebody_module_msgs/GetJointPose.h"
#include "thormang3_wholebody_module_msgs/GetKinematicsPose.h"

namespace thormang3
{

class WholebodyModule
  : public robotis_framework::MotionModule,
    public robotis_framework::Singleton<WholebodyModule>
{
private:
  double control_cycle_msec_;
  boost::thread  queue_thread_;
  boost::thread* tra_gene_tread_;

  /* sample subscriber & publisher */
  ros::Publisher  status_msg_pub_;
  ros::Publisher  set_ctrl_module_pub_;

  std::map<std::string, int> joint_name_to_id_;

  /* base parameters */
  bool is_moving_;

  Eigen::VectorXd present_joint_position_;
  Eigen::VectorXd present_joint_velocity_;
  Eigen::VectorXd goal_joint_position_;

  /* movement */
  double mov_time_;
  int all_time_steps_;
  int cnt_;

  int via_num_;
  Eigen::MatrixXd via_time_;

  Eigen::VectorXd joint_ini_pose_;
  Eigen::MatrixXd joint_ini_via_pose_;
  Eigen::MatrixXd joint_ini_via_d_pose_;
  Eigen::MatrixXd joint_ini_via_dd_pose_;

  Eigen::MatrixXd goal_joint_tra_;
  Eigen::MatrixXd goal_task_tra_;
  Eigen::MatrixXd goal_pevlis_tra_;
  Eigen::MatrixXd goal_l_foot_tra_;
  Eigen::MatrixXd goal_r_foot_tra_;

  /* inverse kinematics */
  bool ik_solving_;
  int ik_id_start_, ik_id_end_;

  Eigen::MatrixXd ik_weight_;
  Eigen::MatrixXd ik_target_position_, ik_target_rotation_;
  Eigen::Quaterniond ik_start_quaternion_, ik_goal_quaternion_;

  /* wholebody motion */
  bool wb_ik_solving_;
  Eigen::MatrixXd wb_pelvis_target_position_, wb_pelvis_target_rotation_;
  Eigen::Quaterniond wb_pelvis_start_quaternion_, wb_pelvis_goal_quaternion_;

  Eigen::MatrixXd wb_l_foot_target_position_, wb_l_foot_target_rotation_;
  Eigen::Quaterniond wb_l_foot_start_quaternion_, wb_l_foot_goal_quaternion_;
  Eigen::MatrixXd wb_r_foot_target_position_, wb_r_foot_target_rotation_;
  Eigen::Quaterniond wb_r_foot_start_quaternion_, wb_r_foot_goal_quaternion_;

  Eigen::MatrixXd wb_l_arm_target_position_, wb_l_arm_target_rotation_;
  Eigen::Quaterniond wb_l_arm_start_quaternion_, wb_l_arm_goal_quaternion_;
  Eigen::MatrixXd wb_r_arm_target_position_, wb_r_arm_target_rotation_;
  Eigen::Quaterniond wb_r_arm_start_quaternion_, wb_r_arm_goal_quaternion_;

  /* balance */
  bool is_balancing_;
  bool on_balance_gain_, off_balance_gain_;
  int balance_gain_cnt_;
  int balance_gain_time_steps_;
  Eigen::MatrixXd on_balance_gain_tra_, off_balance_gain_tra_;
  RobotisBalanceControl balance_control_;
  sensor_msgs::Imu imu_data_msg_;
  geometry_msgs::Wrench l_foot_ft_data_msg_;
  geometry_msgs::Wrench r_foot_ft_data_msg_;

  /* msgs */
  thormang3_wholebody_module_msgs::JointPose goal_joint_pose_msg_;
  thormang3_wholebody_module_msgs::KinematicsPose goal_kinematics_pose_msg_;

  void queueThread();

  void parseInverseKinematicsWeightData(const std::string &path);
  void parseIniPoseData(const std::string &path);

  void setIniPoseMsgCallback(const std_msgs::String::ConstPtr& msg);
  void setJointPoseMsgCallback(const thormang3_wholebody_module_msgs::JointPose::ConstPtr& msg);
  void setKinematicsPoseMsgCallback(const thormang3_wholebody_module_msgs::KinematicsPose::ConstPtr& msg);
  void setWholebodyBalanceMsgCallback(const std_msgs::String::ConstPtr& msg);

  void imuDataCallback(const sensor_msgs::Imu::ConstPtr& msg);

  void setInverseKinematics(int cnt);
  void setInverseKinematicsForLeftFoot(int cnt);
  void setInverseKinematicsForRightFoot(int cnt);
  void setPelvisPose(int cnt);
  void setBalanceControlGain(int cnt);

  void setStartTrajectory();
  void setEndTrajectory();
  void solveInverseKinematics();
  void solveWholebodyInverseKinematics();

  void traGeneProcForIniPose();
  void traGeneProcForStandWheelPose();

  void traGeneProcForTaskSpace();
  void traGeneProcForJointSpace();

  void traGeneProcForPelvis();
  void traGeneProcForWholebody();

  bool getJointPoseCallback(thormang3_wholebody_module_msgs::GetJointPose::Request &req,
                            thormang3_wholebody_module_msgs::GetJointPose::Response &res);
  bool getKinematicsPoseCallback(thormang3_wholebody_module_msgs::GetKinematicsPose::Request &req,
                                 thormang3_wholebody_module_msgs::GetKinematicsPose::Response &res);

  /* Parameter */
  KinematicsDynamics *robotis_;

public:
  WholebodyModule();
  virtual ~WholebodyModule();

  /* ROS Topic Callback Functions */
  void setModeMsgCallback(const std_msgs::String::ConstPtr& msg);

  void initialize(const int control_cycle_msec, robotis_framework::Robot *robot);
  void process(std::map<std::string, robotis_framework::Dynamixel *> dxls, std::map<std::string, double> sensors);

  void stop();
  bool isRunning();
  void publishStatusMsg(unsigned int type, std::string msg);
};

}

#endif /* THORMANG3_WHOLEBODY_MODULE_H_ */
