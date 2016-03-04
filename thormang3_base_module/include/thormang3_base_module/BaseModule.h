/*
 * ThorManipulation.h
 *
 *  Created on: 2016. 1. 18.
 *      Author: zerom
 */

#ifndef BASEMODULE_H_
#define BASEMODULE_H_

#include <map>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/package.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <boost/thread.hpp>

#define EIGEN_NO_DEBUG
#define EIGEN_NO_STATIC_ASSERT
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include <fstream>

#include "robotis_framework_common/MotionModule.h"

#include "thormang3_base_module_msgs/BothWrench.h"
#include "thormang3_base_module_msgs/CalibrationWrench.h"
#include "robotis_controller_msgs/JointCtrlModule.h"

#include "RobotisCommon.h"
#include "RobotisState.h"
#include "JointState.h"
#include "Transformation.h"
#include "Trajectory.h"

namespace ROBOTIS
{

//using namespace ROBOTIS_BASE;

class BaseModule : public MotionModule
{
private:
    static BaseModule *unique_instance_;

    int                 control_cycle_msec_;
    boost::thread       queue_thread_;
    boost::thread       tra_gene_tread_;

    ros::Publisher      send_tra_pub_;
    ros::Publisher		set_ctrl_module_pub_;
    ros::Publisher		ini_ft_pub_;
    ros::Publisher		both_ft_pub_;

    std::map<std::string, int> joint_name_to_id;

    bool				has_goal_joints_;
    bool 				ini_pose_only_;

    std::string 		ft_data_path_;
    std::string			ft_calibration_data_path_;
    bool				has_ft_air_;
    bool				has_ft_gnd_;
    int 				ft_command_;
    int					ft_period_;
    int					ft_count_;
    int					ft_get_count_;

    Eigen::MatrixXd	ft_right_air;
    Eigen::MatrixXd	ft_left_air;
    Eigen::MatrixXd	ft_right_gnd;
    Eigen::MatrixXd	ft_left_gnd;

    Eigen::MatrixXd	ft_right_matrix;
    Eigen::MatrixXd	ft_right_unloaded;
    Eigen::MatrixXd	ft_left_matrix;
    Eigen::MatrixXd	ft_left_unloaded;

    BaseModule();

    void QueueThread();
    void setCtrlModule(std::string module);

    bool parseFTData(const std::string &path);
    void getFTAir();
    void getFTGround();
    void calcFT(Eigen::MatrixXd &ft_right, Eigen::MatrixXd &ft_left);
    void saveFTCalibrationData(const std::string &path);

    void publishFTData(int type, Eigen::MatrixXd &ft_right, Eigen::MatrixXd &ft_left);
    void publishFTCalibrationData();

    void parseIniPoseData(const std::string &path);

    enum
	{
    	FT_NONE = 0,
    	FT_AIR = 1,
		FT_GND = 2,
		FT_CALC = 3,
	};

public:
    virtual ~BaseModule();

    static BaseModule *GetInstance() { return unique_instance_; }

    /* ROS Topic Callback Functions */
    void    IniPoseMsgCallback( const std_msgs::String::ConstPtr& msg );
    void 	IniFtMsgCallback( const std_msgs::String::ConstPtr& msg );

    /* ROS Calculation Functions */
    void    IniposeTraGeneProc();

    void 	PoseGenProc(Eigen::MatrixXd _joint_angle_pose);

    /* ROS Framework Functions */
    void    Initialize(const int control_cycle_msec);
    void    Process(std::map<std::string, Dynamixel *> dxls);

    bool	IsRunning();
    /* Parameter */
//    ROBOTIS_BASE::RobotisData *Humanoid;
    ROBOTIS_BASE::RobotisState *Param;

};

}


#endif /* MANIPULATIONMODULE_H_ */
