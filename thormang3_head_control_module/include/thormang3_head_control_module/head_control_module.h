/*
 * HeadControlModule.h
 *
 *  Created on: 2016. 1. 27.
 *      Author: kayman
 */

#ifndef THORMANG3_HEAD_CONTROL_MODULE_HEAD_CONTROL_MODULE_H_
#define THORMANG3_HEAD_CONTROL_MODULE_HEAD_CONTROL_MODULE_H_


#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <boost/thread.hpp>

#include "robotis_framework_common/motion_module.h"
#include "robotis_math/robotis_math.h"
#include "robotis_controller_msgs/StatusMsg.h"

namespace thormang3
{

class HeadControlModule : public robotis_framework::MotionModule, public robotis_framework::Singleton<HeadControlModule>
{
public:
  HeadControlModule();
  virtual ~HeadControlModule();

  void initialize(const int control_cycle_msec, robotis_framework::Robot *robot);
  void process(std::map<std::string, robotis_framework::Dynamixel *> dxls, std::map<std::string, double> sensors);

  void stop();
  bool isRunning();

private:
  /* ROS Topic Callback Functions */
  void get3DLidarCallback(const std_msgs::String::ConstPtr &msg);
  void setHeadJointCallback(const sensor_msgs::JointState::ConstPtr &msg);

  void queueThread();
  void jointTraGeneThread();

  void beforeMoveLidar();
  void startMoveLidar();
  void afterMoveLidar();
  void publishLidarMoveMsg(std::string msg_data);

  void startMoving();
  void finishMoving();
  void stopMoving();

  void publishStatusMsg(unsigned int type, std::string msg);

  Eigen::MatrixXd calcMinimumJerkTraPVA( double pos_start , double vel_start , double accel_start,
      double pos_end ,   double vel_end ,   double accel_end,
      double smp_time ,  double mov_time );

  int            control_cycle_msec_;
  boost::thread  queue_thread_;
  boost::thread *tra_gene_thread_;
  boost::mutex   tra_lock_;
  ros::Publisher moving_head_pub_;
  ros::Publisher status_msg_pub_;
  const bool     DEBUG;
  bool           stop_process_;
  bool           is_moving_;
  bool           is_direct_control_;
  int            tra_count_, tra_size_;
  double         moving_time_;
  int            current_state_;
  double         original_position_lidar_;

  Eigen::MatrixXd target_position_;
  Eigen::MatrixXd current_position_;
  Eigen::MatrixXd goal_position_;
  Eigen::MatrixXd goal_velocity_;
  Eigen::MatrixXd goal_acceleration_;
  Eigen::MatrixXd calc_joint_tra_;
  Eigen::MatrixXd calc_joint_vel_tra_;
  Eigen::MatrixXd calc_joint_accel_tra_;

  std::map<std::string, int> using_joint_name_;

  enum HeadLidarMode
  {
    None,
    BeforeStart,
    StartMove,
    EndMove,
    AfterMove,
    ModeCount
  };

};

}


#endif /* THORMANG3_HEAD_CONTROL_MODULE_HEAD_CONTROL_MODULE_H_ */
