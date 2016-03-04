/*
 * HeadControlModule.cpp
 *
 *  Created on: 2016. 1. 27.
 *      Author: kayman
 */

#include <stdio.h>
#include "thormang3_head_control_module/HeadControlModule.h"

using namespace ROBOTIS;

HeadControlModule *HeadControlModule::unique_instance_ = new HeadControlModule();

HeadControlModule::HeadControlModule()
: control_cycle_msec_(0)
, is_moving_(false)
, is_direct_control_(false)
, tra_count_(0)
, tra_size_(0)
, current_state_(NONE)
, original_position_lidar_(0.0)
, moving_time_(3.0)
, DEBUG(false)
{
	enable = false;
	module_name = "head_control_module";
	control_mode = POSITION_CONTROL;

	result["head_y"]   	= new DynamixelState();
	result["head_p"]   	= new DynamixelState();

	using_joint_name_["head_y"] = 0;
	using_joint_name_["head_p"] = 1;

	target_position_ 	= Eigen::MatrixXd::Zero(1, result.size());
	current_position_ 	= Eigen::MatrixXd::Zero(1, result.size());
	goal_position_ 		= Eigen::MatrixXd::Zero(1, result.size());
	goal_velocity_ 		= Eigen::MatrixXd::Zero(1, result.size());
	goal_acceleration_ 	= Eigen::MatrixXd::Zero(1, result.size());
}

HeadControlModule::~HeadControlModule()
{
	queue_thread_.join();
}

void HeadControlModule::Initialize(const int control_cycle_msec)
{
	queue_thread_ = boost::thread(boost::bind(&HeadControlModule::QueueThread, this));

	control_cycle_msec_ = control_cycle_msec;

	ros::NodeHandle     _ros_node;

	/* publish topics */
	moving_head_pub_ = _ros_node.advertise<std_msgs::String>("/robotis/sensor/move_lidar", 10);		// todo : change topic name
}

void HeadControlModule::QueueThread()
{
	ros::NodeHandle     _ros_node;
	ros::CallbackQueue  _callback_queue;

	_ros_node.setCallbackQueue(&_callback_queue);

	/* subscribe topics */
	ros::Subscriber get_3d_lidar_sub = _ros_node.subscribe("/robotis/head_control/move_lidar", 1, &HeadControlModule::Get3DLidarCallback, this);
	ros::Subscriber set_head_joint_sub = _ros_node.subscribe("/robotis/head_control/joint_control", 1, &HeadControlModule::setHeadJointCallback, this);

	while(_ros_node.ok())
	{
		_callback_queue.callAvailable();
	}
}

void HeadControlModule::Get3DLidarCallback(const std_msgs::String::ConstPtr &msg)
{
	if(enable == false || is_moving_ == true)
		return;

	if(DEBUG) fprintf(stderr, "TOPIC CALLBACK : get_3d_lidar\n");

	if(current_state_ == NONE)
		beforeMoveLidar();
	else
		ROS_INFO("Head is used.");
}

void HeadControlModule::setHeadJointCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
	if(enable == false)
	{
		ROS_INFO("Head module is not enable.");
		return;
	}

	if(is_moving_ == true && is_direct_control_ == false)
	{
		ROS_INFO("Head is moving now.");
		return;
	}

	// moving time
	moving_time_ = 1.0;								// default : 2 sec

	// set target joint angle
	target_position_ = goal_position_;				// default

	for(int ix = 0; ix < msg->name.size(); ix++)
	{
		std::string _joint_name = msg->name[ix];
		std::map<std::string, int>::iterator _iter = using_joint_name_.find(_joint_name);

		if(_iter != using_joint_name_.end())
		{
			// set target position
			target_position_(0, _iter->second) = msg->position[ix];

			// set time
			int _moving_time = fabs(goal_position_(0, _iter->second) - target_position_(0, _iter->second)) / 0.45;
			if(_moving_time > moving_time_) moving_time_ = _moving_time;

			if(DEBUG) std::cout << "joint : "  << _joint_name << ", Index : " << _iter->second << ", Angle : " << msg->position[ix] << ", Time : " << moving_time_ << std::endl;
		}
	}

	// set init joint vel, accel
	goal_velocity_ = Eigen::MatrixXd::Zero(1, result.size());
	goal_acceleration_ = Eigen::MatrixXd::Zero(1, result.size());

	if(is_moving_ == true && is_direct_control_ == true)
	{
		goal_velocity_ = calc_joint_vel_tra_.block(tra_count_, 0, 1, result.size());
		goal_acceleration_ = calc_joint_accel_tra_.block(tra_count_, 0, 1, result.size());
	}

	// set mode
	is_direct_control_ = true;

	// generate trajectory
	tra_gene_thread_ = new boost::thread(boost::bind(&HeadControlModule::jointTraGeneProc, this));
	delete tra_gene_thread_;
}

void HeadControlModule::Process(std::map<std::string, Dynamixel *> dxls)
{

	if(enable == false)
		return;

	tra_lock_.lock();

	// get joint data
	for(std::map<std::string, DynamixelState *>::iterator state_iter = result.begin(); state_iter != result.end(); state_iter++)
	{
		std::string _joint_name = state_iter->first;
		int _index = using_joint_name_[_joint_name];

		Dynamixel *_dxl = NULL;
		std::map<std::string, Dynamixel*>::iterator _dxl_it = dxls.find(_joint_name);
		if(_dxl_it != dxls.end())
			_dxl = _dxl_it->second;
		else
			continue;

		current_position_(0, _index) 	= _dxl->dxl_state->present_position;
		goal_position_(0, _index)		= _dxl->dxl_state->goal_position;
	}

	// process
	if(tra_size_ != 0)
	{
		// start of steps
		if(tra_count_ == 0)
		{
			is_moving_ = true;

			// set current lidar mode
			if(is_direct_control_ == false) current_state_ = (current_state_ + 1) % MODE_COUNT;

			if(current_state_ == START_MOVE)
				publishLidarMoveTopic("start");
		}

		// end of steps
		if(tra_count_ >= tra_size_)
		{
			calc_joint_tra_ = goal_position_;
			// target_position_ = goal_position_;
			tra_size_ = 0;
			tra_count_ = 0;
			is_moving_ = false;

			// handle lidar state
			switch(current_state_)
			{
			case BEFORE_START:
				// generate start trajectory
				startMoveLidar();
				break;

			case START_MOVE:
				publishLidarMoveTopic("end");
				current_state_ = END_MOVE;

				// generate next trajectory
				afterMoveLidar();
				break;

			case AFTER_MOVE:
				current_state_ = NONE;
				break;

			default:
				break;
			}

			is_direct_control_ = false;
			if(DEBUG) std::cout << "Trajectory End" << std::endl;
		}
		else
		{
			goal_position_ = calc_joint_tra_.block(tra_count_, 0, 1, result.size());
			tra_count_ += 1;
		}
	}

	tra_lock_.unlock();

	// set joint data
	for(std::map<std::string, DynamixelState *>::iterator state_iter = result.begin(); state_iter != result.end(); state_iter++)
	{
		std::string _joint_name = state_iter->first;
		int _index = using_joint_name_[_joint_name];

		result[_joint_name]->goal_position = goal_position_(0, _index);
	}
}

void HeadControlModule::beforeMoveLidar()
{
	// angle and moving time
	original_position_lidar_ = goal_position_(0, using_joint_name_["head_p"]);
	double _start_angle = -10 * M_PI / 180;
	moving_time_ = 1.0;

	// set target joint angle
	target_position_ = goal_position_;
	target_position_(0, using_joint_name_["head_p"]) = _start_angle;
	// set init joint vel, accel
	goal_velocity_ = Eigen::MatrixXd::Zero(1, result.size());
	goal_acceleration_ = Eigen::MatrixXd::Zero(1, result.size());

	// generate trajectory
	tra_gene_thread_ = new boost::thread(boost::bind(&HeadControlModule::jointTraGeneProc, this));
	delete tra_gene_thread_;

	ROS_INFO("Go to Lidar start position");
}

void HeadControlModule::startMoveLidar()
{
	// angle and moving time
	double _target_angle = 85 * M_PI / 180;
	moving_time_ = 8.0;								// 4 sec

	// set target joint angle
	target_position_ = goal_position_;
	target_position_(0, using_joint_name_["head_p"]) = _target_angle;

	// set init joint vel, accel
	goal_velocity_ = Eigen::MatrixXd::Zero(1, result.size());
	goal_acceleration_ = Eigen::MatrixXd::Zero(1, result.size());

	// generate trajectory
	tra_gene_thread_ = new boost::thread(boost::bind(&HeadControlModule::jointTraGeneProc, this));
	delete tra_gene_thread_;

	ROS_INFO("Go to Lidar end position");
}

void HeadControlModule::afterMoveLidar()
{
	// angle and moving time
	moving_time_ = 2.0;

	// set target joint angle
	target_position_ = goal_position_;
	target_position_(0, using_joint_name_["head_p"]) = original_position_lidar_;

	// set init joint vel, accel
	goal_velocity_ = Eigen::MatrixXd::Zero(1, result.size());
	goal_acceleration_ = Eigen::MatrixXd::Zero(1, result.size());

	// generate trajectory
	tra_gene_thread_ = new boost::thread(boost::bind(&HeadControlModule::jointTraGeneProc, this));
	delete tra_gene_thread_;

	ROS_INFO("Go to Lidar before position");
}

bool HeadControlModule::isMoving()
{
	return is_moving_;
}

void HeadControlModule::publishLidarMoveTopic(std::string msg_data)
{
	std_msgs::String _msg;
	_msg.data = msg_data;

	moving_head_pub_.publish(_msg);
}

//void HeadControlModule::directControlQueue()
//{
//	if(has_queue_ele_ == false) return;
//
//	// set mode
//	current_state_ = DIRECT_CONTROL;
//
//	for(std::map<std::string, DynamixelState *>::iterator state_iter = result.begin(); state_iter != result.end(); state_iter++)
//	{
//		std::string _joint_name = state_iter->first;
//		int _index = using_joint_name_[_joint_name];
//
//		std::cout << "joint : "  << _joint_name << ", Index : " << _index << ", Angle : " << target_position_(0, _index) << ", Time : " << moving_time_ << std::endl;
//	}
//
//	// generate trajectory
//	tra_gene_thread_ = boost::thread(boost::bind(&HeadControlModule::jointTraGeneProc, this));
//
//	ROS_INFO("Make Head joint trajectory");
//
//	has_queue_ele_ = false;
//}

/*
   simple minimum jerk trajectory

   pos_start : position at initial state
   vel_start : velocity at initial state
   accel_start : acceleration at initial state

   pos_end : position at final state
   vel_end : velocity at final state
   accel_end : acceleration at final state

   smp_time : sampling time

   mov_time : movement time
 */
Eigen::MatrixXd HeadControlModule::minimumJerkTra( double pos_start , double vel_start , double accel_start,
		double pos_end ,   double vel_end ,   double accel_end,
		double smp_time ,  double mov_time )
{
	Eigen::MatrixXd ___C( 3 , 3 );
	Eigen::MatrixXd __C( 3 , 1 );

	___C << powDI( mov_time , 3 )			, powDI( mov_time , 4 )      , powDI( mov_time , 5 ),
			3 * powDI( mov_time , 2 ) 		, 4 * powDI( mov_time , 3 )  , 5 * powDI( mov_time , 4 ),
			6 * mov_time              		, 12 * powDI( mov_time , 2 ) , 20 * powDI( mov_time , 3 );

	__C << pos_end - pos_start - vel_start * mov_time - accel_start * pow( mov_time , 2 ) / 2,
			vel_end - vel_start - accel_start * mov_time,
			accel_end - accel_start ;

	Eigen::MatrixXd _C = ___C.inverse() * __C;

	double _time_steps;

	_time_steps = mov_time / smp_time;
	int __time_steps = round( _time_steps + 1 );

	Eigen::MatrixXd _time = Eigen::MatrixXd::Zero( __time_steps , 1 );
	Eigen::MatrixXd _tra = Eigen::MatrixXd::Zero( __time_steps , 3 );

	for ( int step = 0; step < __time_steps; step++ )
		_time.coeffRef( step , 0 ) = step * smp_time;

	for ( int step = 0; step < __time_steps; step++ )
	{
		// position
		_tra.coeffRef( step , 0 ) =
				pos_start +
				vel_start * _time.coeff( step , 0 ) +
				0.5 * accel_start * powDI( _time.coeff( step , 0 ) , 2 ) +
				_C.coeff( 0 , 0 ) * powDI( _time.coeff( step , 0 ) , 3 ) +
				_C.coeff( 1 , 0 ) * powDI( _time.coeff( step , 0 ) , 4 ) +
				_C.coeff( 2 , 0 ) * powDI( _time.coeff( step , 0 ) , 5 );
		// velocity
		_tra.coeffRef( step , 1 ) =
				vel_start +
				accel_start * _time.coeff( step , 0 ) +
				3 * _C.coeff( 0 , 0 ) * powDI( _time.coeff( step , 0 ) , 2 ) +
				4 * _C.coeff( 1 , 0 ) * powDI( _time.coeff( step , 0 ) , 3 ) +
				5 * _C.coeff( 2 , 0 ) * powDI( _time.coeff( step , 0 ) , 4 );
		// accel
		_tra.coeffRef( step , 2 ) =
				accel_start +
				6 * _C.coeff( 0 , 0 ) * _time.coeff( step , 0 ) +
				12 * _C.coeff( 1 , 0 ) * powDI( _time.coeff( step , 0 ) , 2 ) +
				20 * _C.coeff( 2 , 0 ) * powDI( _time.coeff( step , 0 ) , 3 );
	}

	return _tra;
}

void HeadControlModule::jointTraGeneProc()
{
	tra_lock_.lock();

	double _smp_time = control_cycle_msec_ * 0.001;		// ms -> s
	int _all_time_steps = int( moving_time_ / _smp_time ) + 1;

	calc_joint_tra_.resize( _all_time_steps , result.size() );
	calc_joint_vel_tra_.resize( _all_time_steps , result.size() );
	calc_joint_accel_tra_.resize( _all_time_steps , result.size() );

	for(std::map<std::string, DynamixelState *>::iterator state_iter = result.begin(); state_iter != result.end(); state_iter++)
	{
		std::string _joint_name = state_iter->first;
		int _index = using_joint_name_[_joint_name];

		double _ini_value = goal_position_(0, _index);
		double _ini_vel = goal_velocity_(0, _index);
		double _ini_accel = goal_acceleration_(0, _index);

		double _tar_value = target_position_(0, _index);

		Eigen::MatrixXd tra = minimumJerkTra( _ini_value , _ini_vel , _ini_accel ,
				_tar_value , 0.0 , 0.0 ,
				_smp_time , moving_time_ );

		calc_joint_tra_.block( 0 , _index , _all_time_steps , 1 ) = tra.block(0, 0, _all_time_steps, 1);
		calc_joint_vel_tra_.block( 0 , _index , _all_time_steps , 1 ) = tra.block(0, 1, _all_time_steps, 1);
		calc_joint_accel_tra_.block( 0 , _index , _all_time_steps , 1 ) = tra.block(0, 2, _all_time_steps, 1);
	}

	tra_size_ = calc_joint_tra_.rows();
	tra_count_ = 0;

	if(DEBUG) ROS_INFO("[ready] make trajectory : %d, %d", tra_size_, tra_count_);

	// init value
	// moving_time_ = 0;

	tra_lock_.unlock();
}

