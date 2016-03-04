/*
 * TestMotionModule.cpp
 *
 *  Created on: 2016. 1. 25.
 *      Author: zerom
 */

#include <stdio.h>
#include <eigen_conversions/eigen_msg.h>

#include "thormang3_walking_module/WalkingModule.h"

#define GRAVITY_ACCELERATION (9.81) // m/s^2

using namespace ROBOTIS;

WalkingMotionModule *WalkingMotionModule::unique_instance_ = new WalkingMotionModule();

WalkingMotionModule::WalkingMotionModule()
    : control_cycle_msec_(8)
{
    enable          = false;
    module_name     = "walking_module";
    control_mode    = POSITION_CONTROL;
    result["r_leg_hip_y"] = new DynamixelState();
    result["r_leg_hip_r"] = new DynamixelState();
    result["r_leg_hip_p"] = new DynamixelState();
    result["r_leg_kn_p"] = new DynamixelState();
    result["r_leg_an_p"] = new DynamixelState();
    result["r_leg_an_r"] = new DynamixelState();

    result["l_leg_hip_y"] = new DynamixelState();
    result["l_leg_hip_r"] = new DynamixelState();
    result["l_leg_hip_p"] = new DynamixelState();
    result["l_leg_kn_p" ] = new DynamixelState();
    result["l_leg_an_p" ] = new DynamixelState();
    result["l_leg_an_r" ] = new DynamixelState();

    previous_enable		= present_enable	= false;
    previous_running	= present_running	= false;

    gyro_x =  gyro_y = 0;
    orientation_roll = orientation_pitch = 0;
    r_foot_fx_N  = r_foot_fy_N  = r_foot_fz_N  = 0;
    r_foot_Tx_Nm = r_foot_Ty_Nm = r_foot_Tz_Nm = 0;
    l_foot_fx_N  = l_foot_fy_N  = l_foot_fz_N  = 0;
    l_foot_Tx_Nm = l_foot_Ty_Nm = l_foot_Tz_Nm = 0;

    total_mass_ = 42.0;
    r_foot_ft_scale_factor_ = l_foot_ft_scale_factor_ = 1.0;
    r_foot_ft_air_.resize(6, 1); l_foot_ft_air_.resize(6, 1);
    r_foot_ft_air_.fill(0); 	 l_foot_ft_air_.fill(0);
    r_foot_ft_gnd_.resize(6, 1); l_foot_ft_gnd_.resize(6, 1);
    r_foot_ft_gnd_.fill(0); 	 l_foot_ft_gnd_.fill(0);
    r_foot_ft_gnd_.coeffRef(2, 0) = 0.5*total_mass_ * GRAVITY_ACCELERATION;
    l_foot_ft_gnd_.coeffRef(2, 0) = 0.5*total_mass_ * GRAVITY_ACCELERATION;


    r_foot_ft_current_voltage_[0] = 0.5*3.3;
    r_foot_ft_current_voltage_[1] = 0.5*3.3;
    r_foot_ft_current_voltage_[2] = 0.5*3.3;
    r_foot_ft_current_voltage_[3] = 0.5*3.3;
    r_foot_ft_current_voltage_[4] = 0.5*3.3;
    r_foot_ft_current_voltage_[5] = 0.5*3.3;

    l_foot_ft_current_voltage_[0] = 0.5*3.3;
    l_foot_ft_current_voltage_[1] = 0.5*3.3;
    l_foot_ft_current_voltage_[2] = 0.5*3.3;
    l_foot_ft_current_voltage_[3] = 0.5*3.3;
    l_foot_ft_current_voltage_[4] = 0.5*3.3;
    l_foot_ft_current_voltage_[5] = 0.5*3.3;


    r_foot_ft_publish_checker_ = -1;
    l_foot_ft_publish_checker_ = -1;

    desired_matrix_g_to_cob_   = Eigen::MatrixXd::Identity(4,4);
    desired_matrix_g_to_rfoot_ = Eigen::MatrixXd::Identity(4,4);
    desired_matrix_g_to_lfoot_ = Eigen::MatrixXd::Identity(4,4);


    balance_update_with_loop_ = false;
    balance_update_duration_ = 1.0;
    balance_update_sys_time_ = 1.0;
    balance_update_polynomial_coeff_.resize(6, 1);

    double tf = balance_update_duration_;
	Eigen::MatrixXd A(6,6), B(6, 1);
	A <<  0.0,	 0.0,	 0.0,	0.0,	0.0, 1.0,
		  0.0,   0.0,    0.0,	0.0,   	1.0, 0.0,
		  0.0,   0.0,    0.0,	2.0,   	0.0, 0.0,
	         tf*tf*tf*tf*tf,	 tf*tf*tf*tf,	  tf*tf*tf,	    tf*tf,	 tf, 1.0,
		 5.0*tf*tf*tf*tf,    4.0*tf*tf*tf,    3.0*tf*tf,	2.0*tf,    	1.0, 0.0,
		20.0*tf*tf*tf,      12.0*tf*tf,       6.0*tf,		2.0,    	0.0, 0.0;

	B << 0, 0, 0, 1.0, 0, 0;

	balance_update_polynomial_coeff_ = A.inverse() * B;



	RX_PI_3x3.resize(3,3);
	RX_PI_3x3 << 1,  0,  0,
			 0, -1,  0,
			 0,  0, -1;

	RZ_PI_3x3.resize(3,3);
	RZ_PI_3x3 << -1, 0, 0,
			0, -1, 0,
			0,  0, 1;
}

WalkingMotionModule::~WalkingMotionModule()
{
    queue_thread_.join();
}

void WalkingMotionModule::Initialize(const int control_cycle_msec)
{
    queue_thread_ = boost::thread(boost::bind(&WalkingMotionModule::QueueThread, this));
    control_cycle_msec_ = control_cycle_msec;

    ForceTorqueSensorInitialize();

    PreviewControlWalking *prev_walking = PreviewControlWalking::GetInstance();
    prev_walking->SetInitialPose(0, -93.0*0.001, -630*0.001, 0, 0, 0,
    		0, 93.0*0.001, -630*0.001,   0, 0, 0,
			0,  0,   0, 0, 0, 0);

    prev_walking->SetFTScaleFactor(1.0, 1.0);
    prev_walking->SetInitForceTorque(0, 0, 0, 0, 0, 0,
    									  0, 0, 0, 0, 0, 0);

    prev_walking->WALK_STABILIZER_GAIN_RATIO = 0;
	prev_walking->IMU_GYRO_GAIN_RATIO = 0.0;//7.31*0.01;
	prev_walking->FORCE_MOMENT_DISTRIBUTION_RATIO = 0.0;

	prev_walking->BALANCE_X_GAIN     = +1.0*20.30*0.625*(prev_walking->FORCE_MOMENT_DISTRIBUTION_RATIO)*prev_walking->WALK_STABILIZER_GAIN_RATIO;
	prev_walking->BALANCE_Y_GAIN     = -1.0*20.30*0.75*(prev_walking->FORCE_MOMENT_DISTRIBUTION_RATIO)*prev_walking->WALK_STABILIZER_GAIN_RATIO;
	prev_walking->BALANCE_Z_GAIN     =    0.0*2.0*(prev_walking->FORCE_MOMENT_DISTRIBUTION_RATIO)*prev_walking->WALK_STABILIZER_GAIN_RATIO;

	prev_walking->BALANCE_PITCH_GAIN = -1.0*0.10*0.5 *(1.0 - prev_walking->FORCE_MOMENT_DISTRIBUTION_RATIO)*prev_walking->WALK_STABILIZER_GAIN_RATIO;
	prev_walking->BALANCE_ROLL_GAIN  = -1.0*0.10*0.75*(1.0 - prev_walking->FORCE_MOMENT_DISTRIBUTION_RATIO)*prev_walking->WALK_STABILIZER_GAIN_RATIO;

	prev_walking->HIP_ROLL_FEEDFORWARD_ANGLE_RAD = 0.0*M_PI/180.0;
	////////// Damping Controller
	prev_walking->BALANCE_ANKLE_ROLL_GAIN_BY_IMU = 0;
	prev_walking->BALANCE_ANKLE_PITCH_GAIN_BY_IMU = 0;

	prev_walking->BALANCE_X_GAIN_BY_FT = 0;
	prev_walking->BALANCE_Y_GAIN_BY_FT = 0;
	prev_walking->BALANCE_Z_GAIN_BY_FT = 0;


	prev_walking->BALANCE_RIGHT_ROLL_GAIN_BY_FT = 0;
	prev_walking->BALANCE_RIGHT_PITCH_GAIN_BY_FT = 0;

	prev_walking->BALANCE_LEFT_ROLL_GAIN_BY_FT = 0;
	prev_walking->BALANCE_LEFT_PITCH_GAIN_BY_FT = 0;

	//the below gain must be zero.
	prev_walking->BALANCE_HIP_ROLL_GAIN = 0;
	prev_walking->BALANCE_HIP_PITCH_GAIN = 0;
	prev_walking->AXIS_CONTROLLER_GAIN = 0;
	prev_walking->LANDING_CONTROLLER_GAIN = 0;

	//time constant
	prev_walking->BALANCE_ANKLE_ROLL_TIME_CONSTANT_BY_IMU	= 0.2;
	prev_walking->BALANCE_ANKLE_PITCH_TIME_CONSTANT_BY_IMU	= 0.2;

	prev_walking->BALANCE_X_TIME_CONSTANT					= 0.1;
	prev_walking->BALANCE_Y_TIME_CONSTANT					= 0.1;
	prev_walking->BALANCE_Z_TIME_CONSTANT					= 0.1;
	prev_walking->BALANCE_RIGHT_ANKLE_ROLL_TIME_CONSTANT	= 0.1;
	prev_walking->BALANCE_RIGHT_ANKLE_PITCH_TIME_CONSTANT	= 0.1;
	prev_walking->BALANCE_LEFT_ANKLE_ROLL_TIME_CONSTANT	 	= 0.1;
	prev_walking->BALANCE_LEFT_ANKLE_PITCH_TIME_CONSTANT	= 0.1;


	prev_walking->COB_X_MANUAL_ADJUSTMENT_M	= -10.0*0.001;

	prev_walking->Initialize();

	prev_walking->BALANCE_ENABLE = true;

	publish_mutex_.lock();
	desired_matrix_g_to_cob_   = prev_walking->matGtoCOB;
	desired_matrix_g_to_rfoot_ = prev_walking->matGtoRF;
	desired_matrix_g_to_lfoot_ = prev_walking->matGtoLF;
	publish_mutex_.unlock();

    result["r_leg_hip_y"]->goal_position = prev_walking->m_OutAngleRad[0];
    result["r_leg_hip_r"]->goal_position = prev_walking->m_OutAngleRad[1];
    result["r_leg_hip_p"]->goal_position = prev_walking->m_OutAngleRad[2];
    result["r_leg_kn_p"]->goal_position  = prev_walking->m_OutAngleRad[3];
    result["r_leg_an_p"]->goal_position  = prev_walking->m_OutAngleRad[4];
    result["r_leg_an_r"]->goal_position  = prev_walking->m_OutAngleRad[5];

    result["l_leg_hip_y"]->goal_position = prev_walking->m_OutAngleRad[6];
    result["l_leg_hip_r"]->goal_position = prev_walking->m_OutAngleRad[7];
    result["l_leg_hip_p"]->goal_position = prev_walking->m_OutAngleRad[8];
    result["l_leg_kn_p" ]->goal_position = prev_walking->m_OutAngleRad[9];
    result["l_leg_an_p" ]->goal_position = prev_walking->m_OutAngleRad[10];
    result["l_leg_an_r" ]->goal_position = prev_walking->m_OutAngleRad[11];

    prev_walking->Start();
    prev_walking->Process();

    previous_enable = enable;
    previous_running = IsRunning();
}

void	WalkingMotionModule::ForceTorqueSensorInitialize()
{
    ros::NodeHandle _ros_node;
    std::string _path = "";
    _path = ros::package::getPath("thormang3_walking_module") + "/config/ft_data.yaml";
    std::string _ft_data_path = _ros_node.param<std::string>("ft_data_path", _path);

    _path = ros::package::getPath("thormang3_walking_module") + "/config/ft_calibration_data.yaml";
    std::string _ft_calib_data_path = _ros_node.param<std::string>("ft_calibration_data_path", _path);

    r_foot_ft_sensor.Initialize(_ft_data_path, "ft_right_foot", "r_foot_ft_link" , "/robotis/sensor/ft_right_foot/raw", "/robotis/sensor/ft_right_foot/scaled");
    l_foot_ft_sensor.Initialize(_ft_data_path, "ft_left_foot",  "l_foot_ft_link",  "/robotis/sensor/ft_left_foot/raw",  "/robotis/sensor/ft_left_foot/scaled");

	YAML::Node doc;
	try
	{
		// load yaml
		doc = YAML::LoadFile(_ft_calib_data_path.c_str());
	}
	catch(const std::exception& e)
	{
		ROS_ERROR("Fail to load ft_calibration_data yaml file.");
		return ;
	}

	std::vector<double> _ft;
	_ft = doc["ft_right_foot_air"].as<std::vector<double> >();
	r_foot_ft_air_ = Eigen::Map<Eigen::MatrixXd>(_ft.data(), 6, 1);

	_ft.clear();
	_ft = doc["ft_right_foot_gnd"].as<std::vector<double> >();
	r_foot_ft_gnd_ = Eigen::Map<Eigen::MatrixXd>(_ft.data(), 6, 1);

	_ft.clear();
	_ft = doc["ft_left_foot_air"].as<std::vector<double> >();
	l_foot_ft_air_ = Eigen::Map<Eigen::MatrixXd>(_ft.data(), 6, 1);

	_ft.clear();
	_ft = doc["ft_left_foot_gnd"].as<std::vector<double> >();
	l_foot_ft_gnd_ = Eigen::Map<Eigen::MatrixXd>(_ft.data(), 6, 1);


	double scale = ( r_foot_ft_gnd_.coeff(2, 0) + l_foot_ft_gnd_.coeff(2, 0) - r_foot_ft_air_.coeff(2, 0) - l_foot_ft_air_.coeff(2, 0) ) / (total_mass_ * GRAVITY_ACCELERATION);
	r_foot_ft_scale_factor_ = l_foot_ft_scale_factor_ = scale;

//	ROS_INFO("[ft sensor scale factor]");
//	ROS_INFO_STREAM("right : " << r_foot_ft_scale_factor_);
//	ROS_INFO_STREAM("left : "  << l_foot_ft_scale_factor_);
//	std::cout <<"right : " << r_foot_ft_air_.transpose() <<std::endl;
//	std::cout <<"right : " << r_foot_ft_gnd_.transpose() <<std::endl;
//	std::cout <<"left  : " << l_foot_ft_air_.transpose() <<std::endl;
//	std::cout <<"left  : " << l_foot_ft_gnd_.transpose() <<std::endl;

	r_foot_ft_sensor.SetScaleParam(r_foot_ft_scale_factor_, r_foot_ft_air_);
	l_foot_ft_sensor.SetScaleParam(l_foot_ft_scale_factor_, l_foot_ft_air_);
}

void	WalkingMotionModule::FTSensorCalibrationDataCallback(const thormang3_base_module_msgs::CalibrationWrench::ConstPtr &msg)
{
	r_foot_ft_air_.coeffRef(0,0) = msg->air_right.force.x;
	r_foot_ft_air_.coeffRef(1,0) = msg->air_right.force.y;
	r_foot_ft_air_.coeffRef(2,0) = msg->air_right.force.z;
	r_foot_ft_air_.coeffRef(3,0) = msg->air_right.torque.x;
	r_foot_ft_air_.coeffRef(4,0) = msg->air_right.torque.y;
	r_foot_ft_air_.coeffRef(5,0) = msg->air_right.torque.z;

	r_foot_ft_gnd_.coeffRef(0,0) = msg->ground_right.force.x;
	r_foot_ft_gnd_.coeffRef(1,0) = msg->ground_right.force.y;
	r_foot_ft_gnd_.coeffRef(2,0) = msg->ground_right.force.z;
	r_foot_ft_gnd_.coeffRef(3,0) = msg->ground_right.torque.x;
	r_foot_ft_gnd_.coeffRef(4,0) = msg->ground_right.torque.y;
	r_foot_ft_gnd_.coeffRef(5,0) = msg->ground_right.torque.z;

	l_foot_ft_air_.coeffRef(0,0) = msg->air_left.force.x;
	l_foot_ft_air_.coeffRef(1,0) = msg->air_left.force.y;
	l_foot_ft_air_.coeffRef(2,0) = msg->air_left.force.z;
	l_foot_ft_air_.coeffRef(3,0) = msg->air_left.torque.x;
	l_foot_ft_air_.coeffRef(4,0) = msg->air_left.torque.y;
	l_foot_ft_air_.coeffRef(5,0) = msg->air_left.torque.z;

	l_foot_ft_gnd_.coeffRef(0,0) = msg->ground_left.force.x;
	l_foot_ft_gnd_.coeffRef(1,0) = msg->ground_left.force.y;
	l_foot_ft_gnd_.coeffRef(2,0) = msg->ground_left.force.z;
	l_foot_ft_gnd_.coeffRef(3,0) = msg->ground_left.torque.x;
	l_foot_ft_gnd_.coeffRef(4,0) = msg->ground_left.torque.y;
	l_foot_ft_gnd_.coeffRef(5,0) = msg->ground_left.torque.z;

	double scale = ( r_foot_ft_gnd_.coeff(2, 0) + l_foot_ft_gnd_.coeff(2, 0) - r_foot_ft_air_.coeff(2, 0) - l_foot_ft_air_.coeff(2, 0) ) / (total_mass_ * GRAVITY_ACCELERATION);
	r_foot_ft_scale_factor_ = l_foot_ft_scale_factor_ = scale;

	r_foot_ft_sensor.SetScaleParam(r_foot_ft_scale_factor_, r_foot_ft_air_);
	l_foot_ft_sensor.SetScaleParam(l_foot_ft_scale_factor_, l_foot_ft_air_);
}

void	WalkingMotionModule::QueueThread()
{
    ros::NodeHandle     _ros_node;
    ros::CallbackQueue  _callback_queue;

    _ros_node.setCallbackQueue(&_callback_queue);

    /* publish topics */
    robot_pose_pub	= _ros_node.advertise<thormang3_walking_module_msgs::RobotPose>("/robotis/walking/robot_pose", 1);
    status_msg_pub	= _ros_node.advertise<std_msgs::String>("/robotis/walking/status_message", 1);


    /* ROS Service Callback Functions */
    ros::ServiceServer get_ref_step_data_server	 = _ros_node.advertiseService("/robotis/walking/get_reference_step_data",
    																			&WalkingMotionModule::GetReferenceStepDataServiceCallback, this);
    ros::ServiceServer add_step_data_array_sever = _ros_node.advertiseService("/robotis/walking/add_step_data",		&WalkingMotionModule::AddStepDataServiceCallback,  		this);
    ros::ServiceServer walking_start_server      = _ros_node.advertiseService("/robotis/walking/walking_start",		&WalkingMotionModule::WalkingStartServiceCallback,		this);
    ros::ServiceServer is_running_server   		 = _ros_node.advertiseService("/robotis/walking/is_running",   		&WalkingMotionModule::IsRunningServiceCallback, 		this);
    ros::ServiceServer set_balance_param_server	 = _ros_node.advertiseService("/robotis/walking/set_balance_param",	&WalkingMotionModule::SetBalanceParamServiceCallback,	this);
    ros::ServiceServer remove_existing_step_data = _ros_node.advertiseService("/robotis/walking/remove_existing_step_data",
    																			&WalkingMotionModule::RemoveExistingStepDataServiceCallback, this);

    /* sensor topic subscribe */
    //ros::Subscriber imu_filter_sub	= _ros_node.subscribe("/robotis/sensor/imu/filter",	3, &WalkingMotionModule::IMUFilterOutputCallback,	this);
    ros::Subscriber imu_data_sub	= _ros_node.subscribe("/robotis/sensor/imu/imu",	3, &WalkingMotionModule::IMUDataOutputCallback,		this);

    ros::Subscriber ft_calibration_sub = _ros_node.subscribe("/robotis/base/both_ft_value", 1, &WalkingMotionModule::FTSensorCalibrationDataCallback, this);
    while(_ros_node.ok())
    {
        _callback_queue.callAvailable();
    }
}

void	WalkingMotionModule::PublishRobotPose(void)
{
	publish_mutex_.lock();
	robot_pose_msg_.global_to_center_of_body.position.x = desired_matrix_g_to_cob_.coeff(0, 3);
	robot_pose_msg_.global_to_center_of_body.position.y = desired_matrix_g_to_cob_.coeff(1, 3);
	robot_pose_msg_.global_to_center_of_body.position.z = desired_matrix_g_to_cob_.coeff(2, 3);
	Eigen::Quaterniond quaterniond_g_to_cob(desired_matrix_g_to_cob_.block<3, 3>(0, 0));


	robot_pose_msg_.global_to_right_foot.position.x = desired_matrix_g_to_rfoot_.coeff(0, 3);
	robot_pose_msg_.global_to_right_foot.position.y = desired_matrix_g_to_rfoot_.coeff(1, 3);
	robot_pose_msg_.global_to_right_foot.position.z = desired_matrix_g_to_rfoot_.coeff(2, 3);
	Eigen::Quaterniond quaterniond_g_to_rf(desired_matrix_g_to_rfoot_.block<3, 3>(0, 0));

	robot_pose_msg_.global_to_left_foot.position.x = desired_matrix_g_to_lfoot_.coeff(0, 3);
	robot_pose_msg_.global_to_left_foot.position.y = desired_matrix_g_to_lfoot_.coeff(1, 3);
	robot_pose_msg_.global_to_left_foot.position.z = desired_matrix_g_to_lfoot_.coeff(2, 3);
	Eigen::Quaterniond quaterniond_g_to_lf(desired_matrix_g_to_lfoot_.block<3, 3>(0, 0));
	publish_mutex_.unlock();

	tf::quaternionEigenToMsg(quaterniond_g_to_cob, robot_pose_msg_.global_to_center_of_body.orientation);
	tf::quaternionEigenToMsg(quaterniond_g_to_rf,  robot_pose_msg_.global_to_right_foot.orientation);
	tf::quaternionEigenToMsg(quaterniond_g_to_lf,  robot_pose_msg_.global_to_left_foot.orientation);

	robot_pose_pub.publish(robot_pose_msg_);

}

int		WalkingMotionModule::StepDataMsgToStepData(thormang3_walking_module_msgs::StepData& src, StepData& des)
{
	int copy_result = STEP_DATA_ERR::NO_ERROR;
	des.TimeData.bWalkingState			= src.time_data.walking_state;
	des.TimeData.dAbsStepTime			= src.time_data.abs_step_time;
	des.TimeData.dDSPratio				= src.time_data.dsp_ratio;
	des.TimeData.sigmoid_ratio_x		= 1.0;//msg->step_data[i].TimeData.front_pause_ratio_x;
	des.TimeData.sigmoid_ratio_y		= 1.0;//msg->step_data[i].TimeData.front_pause_ratio_y;
	des.TimeData.sigmoid_ratio_z		= 1.0;//msg->step_data[i].TimeData.front_pause_ratio_z;
	des.TimeData.sigmoid_ratio_roll		= 1.0;//msg->step_data[i].TimeData.front_pause_ratio_roll;
	des.TimeData.sigmoid_ratio_pitch	= 1.0;//msg->step_data[i].TimeData.front_pause_ratio_pitch;
	des.TimeData.sigmoid_ratio_yaw		= 1.0;//msg->step_data[i].TimeData.front_pause_ratio_yaw;

	des.TimeData.sigmoid_distortion_x		= 1.0;//msg->step_data[i].TimeData.back_pause_ratio_x;
	des.TimeData.sigmoid_distortion_y		= 1.0;//msg->step_data[i].TimeData.back_pause_ratio_y;
	des.TimeData.sigmoid_distortion_z		= 1.0;//msg->step_data[i].TimeData.back_pause_ratio_z;
	des.TimeData.sigmoid_distortion_roll	= 1.0;//msg->step_data[i].TimeData.back_pause_ratio_roll;
	des.TimeData.sigmoid_distortion_pitch	= 1.0;//msg->step_data[i].TimeData.back_pause_ratio_pitch;
	des.TimeData.sigmoid_distortion_yaw		= 1.0;//msg->step_data[i].TimeData.back_pause_ratio_yaw;


	des.PositionData.bMovingFoot		= src.position_data.moving_foot;
	des.PositionData.dShoulderSwingGain	= 0;
	des.PositionData.dElbowSwingGain	= 0;
	des.PositionData.dFootHeight		= src.position_data.foot_z_swap;
	des.PositionData.dWaistPitchAngle	= 0;
	des.PositionData.dWaistYawAngle		= src.position_data.torso_yaw_angle_rad;
	des.PositionData.dZ_Swap_Amplitude	= src.position_data.body_z_swap;

	des.PositionData.stBodyPosition.z			= src.position_data.body_pose.z;
	des.PositionData.stBodyPosition.roll		= src.position_data.body_pose.roll;
	des.PositionData.stBodyPosition.pitch		= src.position_data.body_pose.pitch;
	des.PositionData.stBodyPosition.yaw			= src.position_data.body_pose.yaw;
	des.PositionData.stRightFootPosition.x		= src.position_data.right_foot_pose.x;
	des.PositionData.stRightFootPosition.y		= src.position_data.right_foot_pose.y;
	des.PositionData.stRightFootPosition.z		= src.position_data.right_foot_pose.z;
	des.PositionData.stRightFootPosition.roll	= src.position_data.right_foot_pose.roll;
	des.PositionData.stRightFootPosition.pitch	= src.position_data.right_foot_pose.pitch;
	des.PositionData.stRightFootPosition.yaw	= src.position_data.right_foot_pose.yaw;
	des.PositionData.stLeftFootPosition.x		= src.position_data.left_foot_pose.x;
	des.PositionData.stLeftFootPosition.y		= src.position_data.left_foot_pose.y;
	des.PositionData.stLeftFootPosition.z		= src.position_data.left_foot_pose.z;
	des.PositionData.stLeftFootPosition.roll	= src.position_data.left_foot_pose.roll;
	des.PositionData.stLeftFootPosition.pitch	= src.position_data.left_foot_pose.pitch;
	des.PositionData.stLeftFootPosition.yaw		= src.position_data.left_foot_pose.yaw;

	if((src.time_data.walking_state != WalkingStateFlag::InWalkingStarting)
		&& (src.time_data.walking_state != WalkingStateFlag::InWalking)
		&& (src.time_data.walking_state != WalkingStateFlag::InWalkingEnding) )
		copy_result |= STEP_DATA_ERR::PROBLEM_IN_TIME_DATA;

	if((src.time_data.start_time_delay_ratio_x < 0)
		|| (src.time_data.start_time_delay_ratio_y < 0)
		|| (src.time_data.start_time_delay_ratio_z < 0)
		|| (src.time_data.start_time_delay_ratio_roll < 0)
		|| (src.time_data.start_time_delay_ratio_pitch < 0)
		|| (src.time_data.start_time_delay_ratio_yaw < 0) )
		copy_result |= STEP_DATA_ERR::PROBLEM_IN_TIME_DATA;

	if((src.time_data.finish_time_advance_ratio_x < 0)
		|| (src.time_data.finish_time_advance_ratio_y < 0)
		|| (src.time_data.finish_time_advance_ratio_z < 0)
		|| (src.time_data.finish_time_advance_ratio_roll < 0)
		|| (src.time_data.finish_time_advance_ratio_pitch < 0)
		|| (src.time_data.finish_time_advance_ratio_yaw < 0) )
		copy_result |= STEP_DATA_ERR::PROBLEM_IN_TIME_DATA;

	if(((src.time_data.start_time_delay_ratio_x + src.time_data.finish_time_advance_ratio_x) > 1.0)
		|| ((src.time_data.start_time_delay_ratio_y		+ src.time_data.finish_time_advance_ratio_y		) > 1.0)
		|| ((src.time_data.start_time_delay_ratio_z		+ src.time_data.finish_time_advance_ratio_z		) > 1.0)
		|| ((src.time_data.start_time_delay_ratio_roll	+ src.time_data.finish_time_advance_ratio_roll	) > 1.0)
		|| ((src.time_data.start_time_delay_ratio_pitch	+ src.time_data.finish_time_advance_ratio_pitch	) > 1.0)
		|| ((src.time_data.start_time_delay_ratio_yaw	+ src.time_data.finish_time_advance_ratio_yaw	) > 1.0) )
		copy_result |= STEP_DATA_ERR::PROBLEM_IN_TIME_DATA;

	if((src.position_data.moving_foot != MovingFootFlag::NFootMove)
		&& (src.position_data.moving_foot != MovingFootFlag::RFootMove)
		&& (src.position_data.moving_foot != MovingFootFlag::LFootMove))
		copy_result |= STEP_DATA_ERR::PROBLEM_IN_POSITION_DATA;

	if(src.position_data.foot_z_swap < 0)
		copy_result |= STEP_DATA_ERR::PROBLEM_IN_POSITION_DATA;

	return copy_result;
}

int		WalkingMotionModule::StepDataToStepDataMsg(StepData& src, thormang3_walking_module_msgs::StepData& des)
{
	des.time_data.walking_state   = src.TimeData.bWalkingState;
	des.time_data.abs_step_time   = src.TimeData.dAbsStepTime;
	des.time_data.dsp_ratio       = src.TimeData.dDSPratio;

	des.time_data.start_time_delay_ratio_x		= des.time_data.finish_time_advance_ratio_x		= 0;
	des.time_data.start_time_delay_ratio_y		= des.time_data.finish_time_advance_ratio_y		= 0;
	des.time_data.start_time_delay_ratio_z		= des.time_data.finish_time_advance_ratio_z		= 0;
	des.time_data.start_time_delay_ratio_roll	= des.time_data.finish_time_advance_ratio_roll	= 0;
	des.time_data.start_time_delay_ratio_pitch	= des.time_data.finish_time_advance_ratio_pitch	= 0;
	des.time_data.start_time_delay_ratio_yaw	= des.time_data.finish_time_advance_ratio_yaw	= 0;

	des.position_data.moving_foot          		= src.PositionData.bMovingFoot;
	des.position_data.foot_z_swap          		= src.PositionData.dFootHeight;
	des.position_data.torso_yaw_angle_rad  		= src.PositionData.dWaistYawAngle;
	des.position_data.body_z_swap          		= src.PositionData.dZ_Swap_Amplitude;

	des.position_data.body_pose.z             = src.PositionData.stBodyPosition.z;
	des.position_data.body_pose.roll          = src.PositionData.stBodyPosition.roll;
	des.position_data.body_pose.pitch         = src.PositionData.stBodyPosition.pitch;
	des.position_data.body_pose.yaw           = src.PositionData.stBodyPosition.yaw;
	des.position_data.right_foot_pose.x       = src.PositionData.stRightFootPosition.x;
	des.position_data.right_foot_pose.y       = src.PositionData.stRightFootPosition.y;
	des.position_data.right_foot_pose.z       = src.PositionData.stRightFootPosition.z;
	des.position_data.right_foot_pose.roll    = src.PositionData.stRightFootPosition.roll;
	des.position_data.right_foot_pose.pitch   = src.PositionData.stRightFootPosition.pitch;
	des.position_data.right_foot_pose.yaw     = src.PositionData.stRightFootPosition.yaw;
	des.position_data.left_foot_pose.x        = src.PositionData.stLeftFootPosition.x;
	des.position_data.left_foot_pose.y        = src.PositionData.stLeftFootPosition.y;
	des.position_data.left_foot_pose.z        = src.PositionData.stLeftFootPosition.z;
	des.position_data.left_foot_pose.roll     = src.PositionData.stLeftFootPosition.roll;
	des.position_data.left_foot_pose.pitch    = src.PositionData.stLeftFootPosition.pitch;
	des.position_data.left_foot_pose.yaw      = src.PositionData.stLeftFootPosition.yaw;

	return 0;
}


bool 	WalkingMotionModule::GetReferenceStepDataServiceCallback(thormang3_walking_module_msgs::GetReferenceStepData::Request &req,
																thormang3_walking_module_msgs::GetReferenceStepData::Response &res)
{
	PreviewControlWalking *prev_walking = PreviewControlWalking::GetInstance();

	StepData _refStepData;

	prev_walking->GetReferenceStepDatafotAddition(&_refStepData);

	StepDataToStepDataMsg(_refStepData, res.reference_step_data);

	return true;
}


bool 	WalkingMotionModule::AddStepDataServiceCallback(thormang3_walking_module_msgs::AddStepDataArray::Request &req,
														thormang3_walking_module_msgs::AddStepDataArray::Response &res)
{
	PreviewControlWalking *prev_walking = PreviewControlWalking::GetInstance();
	res.result = STEP_DATA_ERR::NO_ERROR;

	if(enable == false) {
		res.result |= STEP_DATA_ERR::NOT_ENABLED_WALKING_MODULE;
		status_msg.data = WALKING_STATUS_MSG::FAILED_TO_ADD_STEP_DATA_MSG;
		status_msg_pub.publish(status_msg);
		return true;
	}

	if(prev_walking->IsRunning() == true) {
		res.result |= STEP_DATA_ERR::ROBOT_IS_WALKING_NOW;
		status_msg.data = WALKING_STATUS_MSG::FAILED_TO_ADD_STEP_DATA_MSG;
		status_msg_pub.publish(status_msg);
		return true;
	}

	StepData _stepData, _refStepData;
	std::vector<StepData> req_stp;

	prev_walking->GetReferenceStepDatafotAddition(&_refStepData);

	for(int i = 0; i < req.step_data_array.size(); i++)
	{
		res.result |= StepDataMsgToStepData(req.step_data_array[i], _stepData);

		if(_stepData.TimeData.dAbsStepTime <= 0) {
			res.result |= STEP_DATA_ERR::PROBLEM_IN_TIME_DATA;
		}

		if(i != 0) {
			if(_stepData.TimeData.dAbsStepTime <= req_stp[req_stp.size() - 1].TimeData.dAbsStepTime) {
				res.result |= STEP_DATA_ERR::PROBLEM_IN_TIME_DATA;
			}
		}
		else {
			if(_stepData.TimeData.dAbsStepTime <= _refStepData.TimeData.dAbsStepTime) {
				res.result |= STEP_DATA_ERR::PROBLEM_IN_TIME_DATA;
			}
		}

		if(res.result != STEP_DATA_ERR::NO_ERROR) {
    		status_msg.data = WALKING_STATUS_MSG::FAILED_TO_ADD_STEP_DATA_MSG;
    		status_msg_pub.publish(status_msg);
			return true;
		}

		req_stp.push_back(_stepData);
	}


	if(req.remove_existing_step_data == true) {
		int _exist_num_of_step_data = prev_walking->GetNumofRemainingUnreservedStepData();
		if(_exist_num_of_step_data != 0)
			for(int _remove_count  = 0; _remove_count < _exist_num_of_step_data; _remove_count++)
				prev_walking->EraseLastStepData();
	}

	for(unsigned int _i = 0; _i <req_stp.size() ; _i++)
		prev_walking->AddStepData(req_stp[_i]);

	if( req.auto_start == true)
	{
		prev_walking->Start();
	}

	return true;
}
//
//
bool	WalkingMotionModule::WalkingStartServiceCallback(thormang3_walking_module_msgs::WalkingStart::Request &req,
														thormang3_walking_module_msgs::WalkingStart::Response &res)
{
	PreviewControlWalking *prev_walking = PreviewControlWalking::GetInstance();
	res.result = WALKING_START_ERR::NO_ERROR;

	if(enable == false) {
		res.result |= WALKING_START_ERR::NOT_ENABLED_WALKING_MODULE;
	}

	if(prev_walking->IsRunning() == true){
		res.result |= WALKING_START_ERR::ROBOT_IS_WALKING_NOW;
	}

	if(prev_walking->GetNumofRemainingUnreservedStepData() == 0){
		res.result |= WALKING_START_ERR::NO_STEP_DATA;
	}

	if(res.result == WALKING_START_ERR::NO_ERROR) {
		prev_walking->Start();
	}

	return true;
}

bool	WalkingMotionModule::IsRunningServiceCallback(thormang3_walking_module_msgs::IsRunning::Request &req,
														thormang3_walking_module_msgs::IsRunning::Response &res)
{
	bool is_running = IsRunning();
	res.is_running = is_running;

	return true;
}

bool	WalkingMotionModule::IsRunning(void)
{
	return PreviewControlWalking::GetInstance()->IsRunning();
}

bool	WalkingMotionModule::SetBalanceParamServiceCallback(thormang3_walking_module_msgs::SetBalanceParam::Request  &req,
															thormang3_walking_module_msgs::SetBalanceParam::Response &res)
{
	PreviewControlWalking *prev_walking = PreviewControlWalking::GetInstance();
	res.result = BALANCE_PARAM_ERR::NO_ERROR;

	if( enable == false)
		res.result |= BALANCE_PARAM_ERR::NOT_ENABLED_WALKING_MODULE;

	if( balance_update_with_loop_ == true)	{
		res.result |= BALANCE_PARAM_ERR::PREV_REQUEST_IS_NOT_FINISHED;
	}

	if( ( req.balance_param.foot_roll_angle_time_constant  <= 0.0 )
			|| ( req.balance_param.foot_pitch_angle_time_constant  <= 0.0 )
			|| ( req.balance_param.foot_x_force_time_constant      <= 0.0 )
			|| ( req.balance_param.foot_y_force_time_constant      <= 0.0 )
			|| ( req.balance_param.foot_z_force_time_constant      <= 0.0 )
			|| ( req.balance_param.foot_roll_torque_time_constant  <= 0.0 )
			|| ( req.balance_param.foot_pitch_torque_time_constant <= 0.0 ) )
	{
		res.result |= BALANCE_PARAM_ERR::TIME_CONST_IS_ZERO_OR_NEGATIVE;
	}

	if(res.result == BALANCE_PARAM_ERR::NO_ERROR) {
		if(req.updating_duration <= (control_cycle_msec_ * 0.001) )
		{
			// under 8ms apply immediately
			SetBalanceParam(req.balance_param);
    		status_msg.data = WALKING_STATUS_MSG::BALANCE_PARAM_SETTING_FINISH_MSG;
    		status_msg_pub.publish(status_msg);
		}
		else {
			balance_update_duration_ = req.updating_duration;
			balance_update_sys_time_ = 0.0;
			balance_update_polynomial_coeff_.resize(6, 1);

			double tf = balance_update_duration_;
			Eigen::MatrixXd A(6,6), B(6, 1);
			A <<  0.0,	 0.0,	 0.0,	0.0,	0.0, 1.0,
					0.0,   0.0,    0.0,	0.0,   	1.0, 0.0,
					0.0,   0.0,    0.0,	2.0,   	0.0, 0.0,
					tf*tf*tf*tf*tf,	 tf*tf*tf*tf,	  tf*tf*tf,	    tf*tf,	 tf, 1.0,
					5.0*tf*tf*tf*tf,    4.0*tf*tf*tf,    3.0*tf*tf,	2.0*tf,    	1.0, 0.0,
					20.0*tf*tf*tf,      12.0*tf*tf,       6.0*tf,		2.0,    	0.0, 0.0;

			B << 0, 0, 0, 1.0, 0, 0;
			balance_update_polynomial_coeff_ = A.inverse() * B;

			desired_balance_param_ = req.balance_param;

			previous_balance_param_.cob_x_offset_m					= prev_walking->COB_X_MANUAL_ADJUSTMENT_M;
			previous_balance_param_.cob_y_offset_m					= prev_walking->COB_Y_MANUAL_ADJUSTMENT_M;

			previous_balance_param_.hip_roll_swap_angle_rad 		= prev_walking->HIP_ROLL_FEEDFORWARD_ANGLE_RAD;

			previous_balance_param_.gyro_gain 						= prev_walking->WALK_STABILIZER_GAIN_RATIO;
			previous_balance_param_.foot_roll_angle_gain            = prev_walking->BALANCE_ANKLE_ROLL_GAIN_BY_IMU;
			previous_balance_param_.foot_pitch_angle_gain           = prev_walking->BALANCE_ANKLE_PITCH_GAIN_BY_IMU;

			previous_balance_param_.foot_x_force_gain               = prev_walking->BALANCE_X_GAIN_BY_FT;
			previous_balance_param_.foot_y_force_gain               = prev_walking->BALANCE_Y_GAIN_BY_FT;
			previous_balance_param_.foot_z_force_gain               = prev_walking->BALANCE_Z_GAIN_BY_FT;
			previous_balance_param_.foot_roll_torque_gain           = prev_walking->BALANCE_RIGHT_ROLL_GAIN_BY_FT;
			previous_balance_param_.foot_pitch_torque_gain          = prev_walking->BALANCE_RIGHT_PITCH_GAIN_BY_FT;


			previous_balance_param_.foot_roll_angle_time_constant   = prev_walking->BALANCE_ANKLE_ROLL_TIME_CONSTANT_BY_IMU;
			previous_balance_param_.foot_pitch_angle_time_constant  = prev_walking->BALANCE_ANKLE_PITCH_TIME_CONSTANT_BY_IMU;

			previous_balance_param_.foot_x_force_time_constant      = prev_walking->BALANCE_X_TIME_CONSTANT;
			previous_balance_param_.foot_y_force_time_constant      = prev_walking->BALANCE_Y_TIME_CONSTANT;
			previous_balance_param_.foot_z_force_time_constant      = prev_walking->BALANCE_Z_TIME_CONSTANT;
			previous_balance_param_.foot_roll_torque_time_constant  = prev_walking->BALANCE_RIGHT_ANKLE_ROLL_TIME_CONSTANT;
			previous_balance_param_.foot_pitch_torque_time_constant = prev_walking->BALANCE_RIGHT_ANKLE_PITCH_TIME_CONSTANT;

			balance_update_with_loop_ = true;

			status_msg.data = WALKING_STATUS_MSG::BALANCE_PARAM_SETTING_START_MSG;
			status_msg_pub.publish(status_msg);
		}
	}


	return true;
}
//
void	WalkingMotionModule::SetBalanceParam(thormang3_walking_module_msgs::BalanceParam& balance_param_msg)
{
	PreviewControlWalking *prev_walking = PreviewControlWalking::GetInstance();

	prev_walking->COB_X_MANUAL_ADJUSTMENT_M = balance_param_msg.cob_x_offset_m;
	prev_walking->COB_Y_MANUAL_ADJUSTMENT_M = balance_param_msg.cob_y_offset_m;
	prev_walking->HIP_ROLL_FEEDFORWARD_ANGLE_RAD = balance_param_msg.hip_roll_swap_angle_rad;

	prev_walking->WALK_STABILIZER_GAIN_RATIO = balance_param_msg.gyro_gain;
	prev_walking->BALANCE_X_GAIN     = +1.0*20.30*0.625*(prev_walking->FORCE_MOMENT_DISTRIBUTION_RATIO)*prev_walking->WALK_STABILIZER_GAIN_RATIO;
	prev_walking->BALANCE_Y_GAIN     = -1.0*20.30*0.75*(prev_walking->FORCE_MOMENT_DISTRIBUTION_RATIO)*prev_walking->WALK_STABILIZER_GAIN_RATIO;
	prev_walking->BALANCE_Z_GAIN     =    0.0*2.0*(prev_walking->FORCE_MOMENT_DISTRIBUTION_RATIO)*prev_walking->WALK_STABILIZER_GAIN_RATIO;

	prev_walking->BALANCE_PITCH_GAIN = -1.0*0.10*0.5 *(1.0 - prev_walking->FORCE_MOMENT_DISTRIBUTION_RATIO)*prev_walking->WALK_STABILIZER_GAIN_RATIO;
	prev_walking->BALANCE_ROLL_GAIN  = -1.0*0.10*0.75*(1.0 - prev_walking->FORCE_MOMENT_DISTRIBUTION_RATIO)*prev_walking->WALK_STABILIZER_GAIN_RATIO;

	prev_walking->BALANCE_ANKLE_ROLL_GAIN_BY_IMU  = balance_param_msg.foot_roll_angle_gain;
	prev_walking->BALANCE_ANKLE_PITCH_GAIN_BY_IMU = balance_param_msg.foot_pitch_angle_gain;

	prev_walking->BALANCE_X_GAIN_BY_FT				= balance_param_msg.foot_x_force_gain;
	prev_walking->BALANCE_Y_GAIN_BY_FT				= balance_param_msg.foot_y_force_gain;
	prev_walking->BALANCE_Z_GAIN_BY_FT				= balance_param_msg.foot_z_force_gain;
	prev_walking->BALANCE_RIGHT_ROLL_GAIN_BY_FT		= balance_param_msg.foot_roll_torque_gain;
	prev_walking->BALANCE_LEFT_ROLL_GAIN_BY_FT		= balance_param_msg.foot_roll_torque_gain;
	prev_walking->BALANCE_RIGHT_PITCH_GAIN_BY_FT	= balance_param_msg.foot_pitch_torque_gain;
	prev_walking->BALANCE_LEFT_PITCH_GAIN_BY_FT		= balance_param_msg.foot_pitch_torque_gain;

	prev_walking->BALANCE_ANKLE_ROLL_TIME_CONSTANT_BY_IMU  = balance_param_msg.foot_roll_angle_time_constant;
	prev_walking->BALANCE_ANKLE_PITCH_TIME_CONSTANT_BY_IMU = balance_param_msg.foot_pitch_angle_time_constant;

	prev_walking->BALANCE_X_TIME_CONSTANT					= balance_param_msg.foot_x_force_time_constant;
	prev_walking->BALANCE_Y_TIME_CONSTANT					= balance_param_msg.foot_y_force_time_constant;
	prev_walking->BALANCE_Z_TIME_CONSTANT					= balance_param_msg.foot_z_force_time_constant;
	prev_walking->BALANCE_RIGHT_ANKLE_ROLL_TIME_CONSTANT	= balance_param_msg.foot_roll_torque_time_constant;
	prev_walking->BALANCE_LEFT_ANKLE_ROLL_TIME_CONSTANT		= balance_param_msg.foot_roll_torque_time_constant;
	prev_walking->BALANCE_RIGHT_ANKLE_PITCH_TIME_CONSTANT	= balance_param_msg.foot_pitch_torque_time_constant;
	prev_walking->BALANCE_LEFT_ANKLE_PITCH_TIME_CONSTANT	= balance_param_msg.foot_pitch_torque_time_constant;
}

bool	WalkingMotionModule::RemoveExistingStepDataServiceCallback(thormang3_walking_module_msgs::RemoveExistingStepData::Request  &req,
																	thormang3_walking_module_msgs::RemoveExistingStepData::Response &res)
{
	PreviewControlWalking *prev_walking = PreviewControlWalking::GetInstance();

	res.result = REMOVE_STEP_DATA_ERR::NO_ERROR;

	if(IsRunning())
		res.result |= REMOVE_STEP_DATA_ERR::ROBOT_IS_WALKING_NOW;
	else {
		int _exist_num_of_step_data = prev_walking->GetNumofRemainingUnreservedStepData();
		if(_exist_num_of_step_data != 0)
			for(int _remove_count  = 0; _remove_count < _exist_num_of_step_data; _remove_count++)
				prev_walking->EraseLastStepData();
	}
	return true;
}


//void	WalkingMotionModule::IMUFilterOutputCallback(const imu_3dm_gx4::FilterOutput::ConstPtr &msg)
//{
//	Eigen::Quaterniond imu_quat;
//	tf::quaternionMsgToEigen(msg->orientation, imu_quat);
//
//	Eigen::MatrixXd imu_mat = (RX_PI_3x3*(imu_quat.toRotationMatrix()))*RZ_PI_3x3;
//
//	double roll  = atan2( imu_mat.coeff(2,1), imu_mat.coeff(2,2));
////	double pitch = atan2(-imu_mat.coeff(2,0), sqrt(imu_mat.coeff(2,1)*imu_mat.coeff(2,1) + imu_mat.coeff(2,2)*imu_mat.coeff(2,2)) );
//	double pitch = atan2(-imu_mat.coeff(2,0), sqrt(powDI(imu_mat.coeff(2,1), 2) + powDI(imu_mat.coeff(2,2), 2)));
//	double yaw   = atan2( imu_mat.coeff(1,0), imu_mat.coeff(0,0));
//	//ROS_INFO("aa");
//	//std::cout << imu_rotation << std::endl ;
//
////	PreviewControlWalking::GetInstance()->current_imu_roll_rad = roll;
////	PreviewControlWalking::GetInstance()->current_imu_pitch_rad = pitch;
//}

void	WalkingMotionModule::IMUDataOutputCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
//	Eigen::MatrixXd gyro(4, 1);
//	gyro(0, 0) = msg->angular_velocity.x;
//	gyro(1, 0) = msg->angular_velocity.y;
//	gyro(2, 0) = 0;
//	gyro(3, 0) = 1;

	PreviewControlWalking::GetInstance()->current_gyro_roll_rad_per_sec  = -1.0*(msg->angular_velocity.x);
	PreviewControlWalking::GetInstance()->current_gyro_pitch_rad_per_sec = -1.0*(msg->angular_velocity.y);

//	Eigen::MatrixXd acc(4, 1);
//	acc(0, 0) = msg->linear_acceleration.x;
//	acc(1, 0) = msg->linear_acceleration.y;
//	acc(2, 0) = 0;
//	acc(3, 0) = 1;

	//ROS_INFO("bb");
	//ROS_INFO("%f %f %f %f", gyro_x, gyro_y, acc_x, acc_y);

	Eigen::Quaterniond imu_quat;
	tf::quaternionMsgToEigen(msg->orientation, imu_quat);

	Eigen::MatrixXd imu_mat = (RX_PI_3x3*(imu_quat.toRotationMatrix()))*RZ_PI_3x3;

	double roll  = atan2( imu_mat.coeff(2,1), imu_mat.coeff(2,2));
//	double pitch = atan2(-imu_mat.coeff(2,0), sqrt(imu_mat.coeff(2,1)*imu_mat.coeff(2,1) + imu_mat.coeff(2,2)*imu_mat.coeff(2,2)) );
	double pitch = atan2(-imu_mat.coeff(2,0), sqrt(powDI(imu_mat.coeff(2,1), 2) + powDI(imu_mat.coeff(2,2), 2)));
	double yaw   = atan2( imu_mat.coeff(1,0), imu_mat.coeff(0,0));

//	std::cout << "----------------------------------------------------------------" <<std::endl;
//	std::cout << "[gyro]           x : " << -1.0*(msg->angular_velocity.x) << " y : " << -1.0*(msg->angular_velocity.y) << std::endl;
//	std::cout << "[orientation] roll : " << roll << " pitch : " << pitch << "yaw : " << yaw << std::endl;
//	std::cout << "----------------------------------------------------------------" <<std::endl;
	//ROS_INFO("aa");
	//std::cout << imu_rotation << std::endl ;

	PreviewControlWalking::GetInstance()->current_imu_roll_rad = roll;
	PreviewControlWalking::GetInstance()->current_imu_pitch_rad = pitch;


}

void	WalkingMotionModule::Process(std::map<std::string, Dynamixel *> dxls)
{
	present_enable = enable;
    PreviewControlWalking *prev_walking = PreviewControlWalking::GetInstance();

	if(previous_enable != present_enable)
	{
		if(present_enable == true)
		{
			status_msg.data = WALKING_STATUS_MSG::WALKING_MODULE_IS_ENABLED_MSG;
		}
		else {
			status_msg.data = WALKING_STATUS_MSG::WALKING_MODULE_IS_DISABLED_MSG;

			prev_walking->HIP_ROLL_FEEDFORWARD_ANGLE_RAD = 0.0;

			prev_walking->WALK_STABILIZER_GAIN_RATIO = 0.0;
			prev_walking->BALANCE_X_GAIN     = +1.0*20.30*0.625*(prev_walking->FORCE_MOMENT_DISTRIBUTION_RATIO)*prev_walking->WALK_STABILIZER_GAIN_RATIO;
			prev_walking->BALANCE_Y_GAIN     = -1.0*20.30*0.75*(prev_walking->FORCE_MOMENT_DISTRIBUTION_RATIO)*prev_walking->WALK_STABILIZER_GAIN_RATIO;
			prev_walking->BALANCE_Z_GAIN     =    0.0*2.0*(prev_walking->FORCE_MOMENT_DISTRIBUTION_RATIO)*prev_walking->WALK_STABILIZER_GAIN_RATIO;

			prev_walking->BALANCE_PITCH_GAIN = -1.0*0.10*0.5 *(1.0 - prev_walking->FORCE_MOMENT_DISTRIBUTION_RATIO)*prev_walking->WALK_STABILIZER_GAIN_RATIO;
			prev_walking->BALANCE_ROLL_GAIN  = -1.0*0.10*0.75*(1.0 - prev_walking->FORCE_MOMENT_DISTRIBUTION_RATIO)*prev_walking->WALK_STABILIZER_GAIN_RATIO;

			prev_walking->BALANCE_ANKLE_ROLL_GAIN_BY_IMU  = 0.0;
			prev_walking->BALANCE_ANKLE_PITCH_GAIN_BY_IMU = 0.0;

			prev_walking->BALANCE_X_GAIN_BY_FT				= 0.0;
			prev_walking->BALANCE_Y_GAIN_BY_FT				= 0.0;
			prev_walking->BALANCE_Z_GAIN_BY_FT				= 0.0;
			prev_walking->BALANCE_RIGHT_ROLL_GAIN_BY_FT		= 0.0;
			prev_walking->BALANCE_LEFT_ROLL_GAIN_BY_FT		= 0.0;
			prev_walking->BALANCE_RIGHT_PITCH_GAIN_BY_FT	= 0.0;
			prev_walking->BALANCE_LEFT_PITCH_GAIN_BY_FT		= 0.0;

		}

		status_msg_pub.publish(status_msg);
	}
	previous_enable = present_enable;


	bool _exist_r_leg_an_r = false;
	bool _exist_r_leg_an_p = false;
	bool _exist_l_leg_an_r = false;
	bool _exist_l_leg_an_p = false;

	for(std::map<std::string, DynamixelState *>::iterator state_iter = result.begin(); state_iter != result.end(); state_iter++)
	{
		std::string _joint_name = state_iter->first;

        Dynamixel *_dxl = NULL;
        std::map<std::string, Dynamixel*>::iterator _dxl_it = dxls.find(_joint_name);
        if(_dxl_it != dxls.end())
            _dxl = _dxl_it->second;
        else
            continue;

		if(state_iter->first == "r_leg_an_r")
		{
			r_foot_ft_current_voltage_[0] = (dxls["r_leg_an_r"]->dxl_state->ext_port_data[0])*3.3/4095.0;
			r_foot_ft_current_voltage_[1] = (dxls["r_leg_an_r"]->dxl_state->ext_port_data[1])*3.3/4095.0;
			r_foot_ft_current_voltage_[2] = (dxls["r_leg_an_r"]->dxl_state->ext_port_data[2])*3.3/4095.0;
			r_foot_ft_current_voltage_[3] = (dxls["r_leg_an_r"]->dxl_state->ext_port_data[3])*3.3/4095.0;
			_exist_r_leg_an_r = true;
		}
		else if(state_iter->first == "r_leg_an_p")
		{
			r_foot_ft_current_voltage_[4] = (dxls["r_leg_an_p"]->dxl_state->ext_port_data[0])*3.3/4095.0;
			r_foot_ft_current_voltage_[5] = (dxls["r_leg_an_p"]->dxl_state->ext_port_data[1])*3.3/4095.0;
			_exist_r_leg_an_p = true;
		}
		else if(state_iter->first == "l_leg_an_r")
		{
			l_foot_ft_current_voltage_[0] = (dxls["l_leg_an_r"]->dxl_state->ext_port_data[0])*3.3/4095.0;
			l_foot_ft_current_voltage_[1] = (dxls["l_leg_an_r"]->dxl_state->ext_port_data[1])*3.3/4095.0;
			l_foot_ft_current_voltage_[2] = (dxls["l_leg_an_r"]->dxl_state->ext_port_data[2])*3.3/4095.0;
			l_foot_ft_current_voltage_[3] = (dxls["l_leg_an_r"]->dxl_state->ext_port_data[3])*3.3/4095.0;
			_exist_l_leg_an_r = true;
		}
		else if(state_iter->first == "l_leg_an_p")
		{
			l_foot_ft_current_voltage_[4] = (dxls["l_leg_an_p"]->dxl_state->ext_port_data[0])*3.3/4095.0;
			l_foot_ft_current_voltage_[5] = (dxls["l_leg_an_p"]->dxl_state->ext_port_data[1])*3.3/4095.0;
			_exist_l_leg_an_p = true;
		}
		else
			continue;
	}







//	printf(" r: %f %f %f %f %f %f\n", r_foot_ft_current_voltage_[0],
//															   r_foot_ft_current_voltage_[1],
//															   r_foot_ft_current_voltage_[2],
//															   r_foot_ft_current_voltage_[3],
//															   r_foot_ft_current_voltage_[4],
//															   r_foot_ft_current_voltage_[5]);
//
//	printf(" l: %f %f %f %f %f %f\n", l_foot_ft_current_voltage_[0],
//															   l_foot_ft_current_voltage_[1],
//															   l_foot_ft_current_voltage_[2],
//															   l_foot_ft_current_voltage_[3],
//															   l_foot_ft_current_voltage_[4],
//															   l_foot_ft_current_voltage_[5]);

	if( _exist_r_leg_an_r && _exist_r_leg_an_p) {
	    r_foot_ft_publish_checker_++;

	    if((r_foot_ft_publish_checker_ % 3) == 0)
	    {
	    	r_foot_ft_publish_checker_ = 0;
	    	r_foot_ft_sensor.SetCurrentVoltageOutputPublishForceTorque(	r_foot_ft_current_voltage_[0],
	    																r_foot_ft_current_voltage_[1],
	    																r_foot_ft_current_voltage_[2],
	    																r_foot_ft_current_voltage_[3],
	    																r_foot_ft_current_voltage_[4],
	    																r_foot_ft_current_voltage_[5]);
	    }
	    else {
	    	r_foot_ft_sensor.SetCurrentVoltageOutput(	r_foot_ft_current_voltage_[0],
	    											 	r_foot_ft_current_voltage_[1],
														r_foot_ft_current_voltage_[2],
	    												r_foot_ft_current_voltage_[3],
	    												r_foot_ft_current_voltage_[4],
	    												r_foot_ft_current_voltage_[5]);
	    }
	    r_foot_ft_sensor.GetCurrentForceTorqueScaled(&r_foot_fx_N, &r_foot_fy_N, &r_foot_fz_N, &r_foot_Tx_Nm, &r_foot_Ty_Nm, &r_foot_Tz_Nm);

	}
	if( _exist_l_leg_an_r && _exist_l_leg_an_p) {
		l_foot_ft_publish_checker_++;

	    if((l_foot_ft_publish_checker_ % 3) == 0)
	    {
	    	l_foot_ft_publish_checker_ = 0;
	    	l_foot_ft_sensor.SetCurrentVoltageOutputPublishForceTorque(	l_foot_ft_current_voltage_[0],
	    																l_foot_ft_current_voltage_[1],
	    																l_foot_ft_current_voltage_[2],
	    																l_foot_ft_current_voltage_[3],
	    																l_foot_ft_current_voltage_[4],
	    																l_foot_ft_current_voltage_[5]);
	    }
	    else {
	    	l_foot_ft_sensor.SetCurrentVoltageOutput(	l_foot_ft_current_voltage_[0],
	    												l_foot_ft_current_voltage_[1],
	    												l_foot_ft_current_voltage_[2],
	    												l_foot_ft_current_voltage_[3],
	    												l_foot_ft_current_voltage_[4],
	    												l_foot_ft_current_voltage_[5]);
	    }
		l_foot_ft_sensor.GetCurrentForceTorqueScaled(&l_foot_fx_N, &l_foot_fy_N, &l_foot_fz_N, &l_foot_Tx_Nm, &l_foot_Ty_Nm, &l_foot_Tz_Nm);
	}


	r_foot_fx_N = sign(r_foot_fx_N) * fmin( fabs(r_foot_fx_N), 2000.0);
	r_foot_fy_N = sign(r_foot_fy_N) * fmin( fabs(r_foot_fy_N), 2000.0);
	r_foot_fz_N = sign(r_foot_fz_N) * fmin( fabs(r_foot_fz_N), 2000.0);
	r_foot_Tx_Nm = sign(r_foot_Tx_Nm) *fmin(fabs(r_foot_Tx_Nm), 300.0);
	r_foot_Ty_Nm = sign(r_foot_Ty_Nm) *fmin(fabs(r_foot_Ty_Nm), 300.0);
	r_foot_Tz_Nm = sign(r_foot_Tz_Nm) *fmin(fabs(r_foot_Tz_Nm), 300.0);

	l_foot_fx_N = sign(l_foot_fx_N) * fmin( fabs(l_foot_fx_N), 2000.0);
	l_foot_fy_N = sign(l_foot_fy_N) * fmin( fabs(l_foot_fy_N), 2000.0);
	l_foot_fz_N = sign(l_foot_fz_N) * fmin( fabs(l_foot_fz_N), 2000.0);
	l_foot_Tx_Nm = sign(l_foot_Tx_Nm) *fmin(fabs(l_foot_Tx_Nm), 300.0);
	l_foot_Ty_Nm = sign(l_foot_Ty_Nm) *fmin(fabs(l_foot_Ty_Nm), 300.0);
	l_foot_Tz_Nm = sign(l_foot_Tz_Nm) *fmin(fabs(l_foot_Tz_Nm), 300.0);


    if(enable == false)
        return;



    if(balance_update_with_loop_ == true)
    {
    	balance_update_sys_time_ += control_cycle_msec_ * 0.001;
    	if(balance_update_sys_time_ >= balance_update_duration_ ) {
    		balance_update_sys_time_ = balance_update_duration_;
    		balance_update_with_loop_ = false;
    		SetBalanceParam(desired_balance_param_);
    		status_msg.data = WALKING_STATUS_MSG::BALANCE_PARAM_SETTING_FINISH_MSG;
    		status_msg_pub.publish(status_msg);
    	}
    	else {
    		double current_update_gain =  balance_update_polynomial_coeff_.coeff(0,0) * powDI(balance_update_sys_time_ , 5)
    									+ balance_update_polynomial_coeff_.coeff(1,0) * powDI(balance_update_sys_time_ , 4)
										+ balance_update_polynomial_coeff_.coeff(2,0) * powDI(balance_update_sys_time_ , 3)
										+ balance_update_polynomial_coeff_.coeff(3,0) * powDI(balance_update_sys_time_ , 2)
										+ balance_update_polynomial_coeff_.coeff(4,0) * powDI(balance_update_sys_time_ , 1)
										+ balance_update_polynomial_coeff_.coeff(5,0) ;

    		current_balance_param_.cob_x_offset_m                  = current_update_gain*(desired_balance_param_.cob_x_offset_m      			 - previous_balance_param_.cob_x_offset_m			      ) + previous_balance_param_.cob_x_offset_m;
    		current_balance_param_.cob_y_offset_m                  = current_update_gain*(desired_balance_param_.cob_y_offset_m      			 - previous_balance_param_.cob_y_offset_m			      ) + previous_balance_param_.cob_y_offset_m;

    		current_balance_param_.hip_roll_swap_angle_rad		   = current_update_gain*(desired_balance_param_.hip_roll_swap_angle_rad         - previous_balance_param_.hip_roll_swap_angle_rad        ) + previous_balance_param_.hip_roll_swap_angle_rad;

    		current_balance_param_.gyro_gain					   = current_update_gain*(desired_balance_param_.gyro_gain						 - previous_balance_param_.gyro_gain					  ) + previous_balance_param_.gyro_gain;
    		current_balance_param_.foot_roll_angle_gain            = current_update_gain*(desired_balance_param_.foot_roll_angle_gain            - previous_balance_param_.foot_roll_angle_gain           ) + previous_balance_param_.foot_roll_angle_gain;
    		current_balance_param_.foot_pitch_angle_gain           = current_update_gain*(desired_balance_param_.foot_pitch_angle_gain           - previous_balance_param_.foot_pitch_angle_gain          ) + previous_balance_param_.foot_pitch_angle_gain;

    		current_balance_param_.foot_x_force_gain               = current_update_gain*(desired_balance_param_.foot_x_force_gain               - previous_balance_param_.foot_x_force_gain              ) + previous_balance_param_.foot_x_force_gain;
    		current_balance_param_.foot_y_force_gain               = current_update_gain*(desired_balance_param_.foot_y_force_gain               - previous_balance_param_.foot_y_force_gain              ) + previous_balance_param_.foot_y_force_gain;
    		current_balance_param_.foot_z_force_gain               = current_update_gain*(desired_balance_param_.foot_z_force_gain               - previous_balance_param_.foot_z_force_gain              ) + previous_balance_param_.foot_z_force_gain;
    		current_balance_param_.foot_roll_torque_gain           = current_update_gain*(desired_balance_param_.foot_roll_torque_gain           - previous_balance_param_.foot_roll_torque_gain          ) + previous_balance_param_.foot_roll_torque_gain;
    		current_balance_param_.foot_pitch_torque_gain          = current_update_gain*(desired_balance_param_.foot_pitch_torque_gain          - previous_balance_param_.foot_pitch_torque_gain         ) + previous_balance_param_.foot_pitch_torque_gain;


    		current_balance_param_.foot_roll_angle_time_constant   = current_update_gain*(desired_balance_param_.foot_roll_angle_time_constant   - previous_balance_param_.foot_roll_angle_time_constant  ) + previous_balance_param_.foot_roll_angle_time_constant;
    		current_balance_param_.foot_pitch_angle_time_constant  = current_update_gain*(desired_balance_param_.foot_pitch_angle_time_constant  - previous_balance_param_.foot_pitch_angle_time_constant ) + previous_balance_param_.foot_pitch_angle_time_constant;

    		current_balance_param_.foot_x_force_time_constant      = current_update_gain*(desired_balance_param_.foot_x_force_time_constant      - previous_balance_param_.foot_x_force_time_constant     ) + previous_balance_param_.foot_x_force_time_constant;
    		current_balance_param_.foot_y_force_time_constant      = current_update_gain*(desired_balance_param_.foot_y_force_time_constant      - previous_balance_param_.foot_y_force_time_constant     ) + previous_balance_param_.foot_y_force_time_constant;
    		current_balance_param_.foot_z_force_time_constant      = current_update_gain*(desired_balance_param_.foot_z_force_time_constant      - previous_balance_param_.foot_z_force_time_constant     ) + previous_balance_param_.foot_z_force_time_constant;
    		current_balance_param_.foot_roll_torque_time_constant  = current_update_gain*(desired_balance_param_.foot_roll_torque_time_constant  - previous_balance_param_.foot_roll_torque_time_constant ) + previous_balance_param_.foot_roll_torque_time_constant;
    		current_balance_param_.foot_pitch_torque_time_constant = current_update_gain*(desired_balance_param_.foot_pitch_torque_time_constant - previous_balance_param_.foot_pitch_torque_time_constant) + previous_balance_param_.foot_pitch_torque_time_constant;


    		SetBalanceParam(current_balance_param_);
    	}
    }

    prev_walking->current_right_fx_N  = r_foot_fx_N;
    prev_walking->current_right_fy_N  = r_foot_fy_N;
    prev_walking->current_right_fz_N  = r_foot_fz_N;
    prev_walking->current_right_Tx_Nm = r_foot_Tx_Nm;
    prev_walking->current_right_Ty_Nm = r_foot_Ty_Nm;
    prev_walking->current_right_Tz_Nm = r_foot_Tz_Nm;

    prev_walking->current_left_fx_N  = l_foot_fx_N;
    prev_walking->current_left_fy_N  = l_foot_fy_N;
    prev_walking->current_left_fz_N  = l_foot_fz_N;
    prev_walking->current_left_Tx_Nm = l_foot_Tx_Nm;
    prev_walking->current_left_Ty_Nm = l_foot_Ty_Nm;
    prev_walking->current_left_Tz_Nm = l_foot_Tz_Nm;


	prev_walking->Process();

	publish_mutex_.lock();
	desired_matrix_g_to_cob_   = prev_walking->matGtoCOB;
	desired_matrix_g_to_rfoot_ = prev_walking->matGtoRF;
	desired_matrix_g_to_lfoot_ = prev_walking->matGtoLF;
	publish_mutex_.unlock();

    result["r_leg_hip_y"]->goal_position = prev_walking->m_OutAngleRad[0];
    result["r_leg_hip_r"]->goal_position = prev_walking->m_OutAngleRad[1];
    result["r_leg_hip_p"]->goal_position = prev_walking->m_OutAngleRad[2];
    result["r_leg_kn_p" ]->goal_position = prev_walking->m_OutAngleRad[3];
    result["r_leg_an_p" ]->goal_position = prev_walking->m_OutAngleRad[4];
    result["r_leg_an_r" ]->goal_position = prev_walking->m_OutAngleRad[5];

    result["l_leg_hip_y"]->goal_position = prev_walking->m_OutAngleRad[6];
    result["l_leg_hip_r"]->goal_position = prev_walking->m_OutAngleRad[7];
    result["l_leg_hip_p"]->goal_position = prev_walking->m_OutAngleRad[8];
    result["l_leg_kn_p" ]->goal_position = prev_walking->m_OutAngleRad[9];
    result["l_leg_an_p" ]->goal_position = prev_walking->m_OutAngleRad[10];
    result["l_leg_an_r" ]->goal_position = prev_walking->m_OutAngleRad[11];

    present_running = IsRunning();
    if(previous_running != present_running) {
    	if(present_running == true) {
    		status_msg.data = WALKING_STATUS_MSG::WALKING_START_MSG;
    		status_msg_pub.publish(status_msg);
    	}
    	else {
    		status_msg.data = WALKING_STATUS_MSG::WALKING_FINISH_MSG;
    		status_msg_pub.publish(status_msg);
    	}
    }
    previous_running = present_running;
}
