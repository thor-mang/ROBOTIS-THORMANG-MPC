/*
 * HeadControlModule.h
 *
 *  Created on: 2016. 1. 27.
 *      Author: kayman
 */

#ifndef HEAD_CONTROL_MODULE_H_
#define HEAD_CONTROL_MODULE_H_

#define EIGEN_NO_DEBUG
#define EIGEN_NO_STATIC_ASSERT

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <boost/thread.hpp>

#include <Eigen/Dense>

#include <cmath>

#include "robotis_framework_common/MotionModule.h"

namespace ROBOTIS
{

class HeadControlModule : public MotionModule
{
private:
    static HeadControlModule *unique_instance_;

    int             control_cycle_msec_;
    boost::thread   queue_thread_;
    boost::thread	*tra_gene_thread_;
    boost::mutex	tra_lock_;
    ros::Publisher	moving_head_pub_;
    const bool 		DEBUG;
    bool			is_moving_;
    bool 			is_direct_control_;
    int 			tra_count_, tra_size_;
    double 			moving_time_;
    int				current_state_;
    double			original_position_lidar_;

    Eigen::MatrixXd target_position_;
    Eigen::MatrixXd current_position_;
    Eigen::MatrixXd goal_position_;
    Eigen::MatrixXd goal_velocity_;
    Eigen::MatrixXd goal_acceleration_;
    Eigen::MatrixXd calc_joint_tra_;
    Eigen::MatrixXd calc_joint_vel_tra_;
    Eigen::MatrixXd calc_joint_accel_tra_;

    std::map<std::string, int> using_joint_name_;

    HeadControlModule();

    void QueueThread();
    void beforeMoveLidar();
    void startMoveLidar();
    void afterMoveLidar();
    bool isMoving();
    void publishLidarMoveTopic(std::string msg_data);


    Eigen::MatrixXd minimumJerkTra( double pos_start , double vel_start , double accel_start,
                                      double pos_end ,   double vel_end ,   double accel_end,
                                      double smp_time ,  double mov_time );
    void jointTraGeneProc();

    enum HEAD_LIDAR_MODE
    {
        NONE,
        BEFORE_START,
        START_MOVE,
		END_MOVE,
		AFTER_MOVE,
		MODE_COUNT,
    };

public:
    virtual ~HeadControlModule();

    static HeadControlModule *GetInstance() { return unique_instance_; }

    /* ROS Topic Callback Functions */
    void    Get3DLidarCallback(const std_msgs::String::ConstPtr &msg);
    void 	setHeadJointCallback(const sensor_msgs::JointState::ConstPtr &msg);

    void    Initialize(const int control_cycle_msec);
    void    Process(std::map<std::string, Dynamixel *> dxls);
};

}


#endif /* HEAD_CONTROL_MODULE_H_ */
