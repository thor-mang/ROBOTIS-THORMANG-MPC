/*
 * ThorManipulation.cpp
 *
 *  Created on: 2016. 1. 18.
 *      Author: zerom
 */

#include <stdio.h>
#include "thormang3_base_module/BaseModule.h"

using namespace ROBOTIS;

BaseModule *BaseModule::unique_instance_ = new BaseModule();

BaseModule::BaseModule()
: control_cycle_msec_(0)
, has_goal_joints_(false)
, has_ft_air_(false)
, has_ft_gnd_(false)
, ft_command_(FT_NONE)
, ft_count_(0)
, ini_pose_only_(false)
{
	enable          = false;
    module_name     = "base_module";
	control_mode    = POSITION_CONTROL;

	/* arm */
	result["r_arm_sh_p1"]   = new DynamixelState();
	result["l_arm_sh_p1"]   = new DynamixelState();
	result["r_arm_sh_r"]    = new DynamixelState();
	result["l_arm_sh_r"]    = new DynamixelState();
	result["r_arm_sh_p2"]   = new DynamixelState();
	result["l_arm_sh_p2"]   = new DynamixelState();
	result["r_arm_el_y"]    = new DynamixelState();
	result["l_arm_el_y"]    = new DynamixelState();
	result["r_arm_wr_r"]    = new DynamixelState();
	result["l_arm_wr_r"]    = new DynamixelState();
	result["r_arm_wr_y"]    = new DynamixelState();
	result["l_arm_wr_y"]    = new DynamixelState();
	result["r_arm_wr_p"]    = new DynamixelState();
	result["l_arm_wr_p"]    = new DynamixelState();

    /* gripper */
    result["r_arm_grip"]    = new DynamixelState();
    result["l_arm_grip"]    = new DynamixelState();

	/* body */
	result["torso_y"]       = new DynamixelState();

	/* leg */
	result["r_leg_hip_y"]   = new DynamixelState();
	result["r_leg_hip_r"]   = new DynamixelState();
	result["r_leg_hip_p"]   = new DynamixelState();
	result["r_leg_kn_p"]    = new DynamixelState();
	result["r_leg_an_p"]    = new DynamixelState();
	result["r_leg_an_r"]    = new DynamixelState();

	result["l_leg_hip_y"]   = new DynamixelState();
	result["l_leg_hip_r"]   = new DynamixelState();
	result["l_leg_hip_p"]   = new DynamixelState();
	result["l_leg_kn_p"]    = new DynamixelState();
	result["l_leg_an_p"]    = new DynamixelState();
	result["l_leg_an_r"]    = new DynamixelState();

	/* head */
	result["head_y"]        = new DynamixelState();
	result["head_p"]        = new DynamixelState();

    /* arm */
    joint_name_to_id["r_arm_sh_p1"] = 1;
    joint_name_to_id["l_arm_sh_p1"] = 2;
    joint_name_to_id["r_arm_sh_r"]  = 3;
    joint_name_to_id["l_arm_sh_r"]  = 4;
    joint_name_to_id["r_arm_sh_p2"] = 5;
    joint_name_to_id["l_arm_sh_p2"] = 6;
    joint_name_to_id["r_arm_el_y"]  = 7;
    joint_name_to_id["l_arm_el_y"]  = 8;
    joint_name_to_id["r_arm_wr_r"]  = 9;
    joint_name_to_id["l_arm_wr_r"]  = 10;
    joint_name_to_id["r_arm_wr_y"]  = 11;
    joint_name_to_id["l_arm_wr_y"]  = 12;
    joint_name_to_id["r_arm_wr_p"]  = 13;
    joint_name_to_id["l_arm_wr_p"]  = 14;

    /* leg */
    joint_name_to_id["r_leg_hip_y"]   = 15;
    joint_name_to_id["l_leg_hip_y"]   = 16;
    joint_name_to_id["r_leg_hip_r"]   = 17;
    joint_name_to_id["l_leg_hip_r"]   = 18;
    joint_name_to_id["r_leg_hip_p"]   = 19;
    joint_name_to_id["l_leg_hip_p"]   = 20;
    joint_name_to_id["r_leg_kn_p"]    = 21;
    joint_name_to_id["l_leg_kn_p"]    = 22;
    joint_name_to_id["r_leg_an_p"]    = 23;
    joint_name_to_id["l_leg_an_p"]    = 24;
    joint_name_to_id["r_leg_an_r"]    = 25;
    joint_name_to_id["l_leg_an_r"]    = 26;

    /* body */
    joint_name_to_id["torso_y"]     = 27;

    /* head */
    joint_name_to_id["head_y"]        = 28;
    joint_name_to_id["head_p"]        = 29;

    /* gripper */
    joint_name_to_id["r_arm_grip"]  = 31;
    joint_name_to_id["l_arm_grip"]  = 30;

	Param = new ROBOTIS_BASE::RobotisState();
}

BaseModule::~BaseModule()
{
	queue_thread_.join();
}

void BaseModule::Initialize(const int control_cycle_msec)
{
	control_cycle_msec_ = control_cycle_msec;
	ft_period_ = 2 * 1000/ control_cycle_msec;
	queue_thread_ = boost::thread(boost::bind(&BaseModule::QueueThread, this));

	ros::NodeHandle _ros_node;

	/* publish topics */
	send_tra_pub_           = _ros_node.advertise<std_msgs::String>("/robotis/base/send_tra", 1);
	set_ctrl_module_pub_	= _ros_node.advertise<robotis_controller_msgs::JointCtrlModule>("/robotis/set_ctrl_module", 1);
	ini_ft_pub_				= _ros_node.advertise<thormang3_base_module_msgs::BothWrench>("/robotis/base/ini_ft_value", 1);
	both_ft_pub_		= _ros_node.advertise<thormang3_base_module_msgs::CalibrationWrench>("/robotis/base/both_ft_value", 1);

    std::string _path = ros::package::getPath("thormang3_base_module") + "/data/ft_data.yaml";
	ft_data_path_ = _ros_node.param<std::string>("ft_data_path", _path);
    _path = ros::package::getPath("thormang3_base_module") + "/data/ft_calibration_data.yaml";
	ft_calibration_data_path_ = _ros_node.param<std::string>("ft_calibration_data_path", _path);
	parseFTData(ft_data_path_);
}

void BaseModule::parseIniPoseData( const std::string &path )
{
    YAML::Node doc;
    try
    {
        // load yaml
        doc = YAML::LoadFile( path.c_str() );
    }
    catch(const std::exception& e)
    {
        ROS_ERROR("Fail to load yaml file.");
        return ;
    }

    // parse movement time
    double _mov_time;
    _mov_time = doc["mov_time"].as< double >();

    Param->mov_time = _mov_time;

    // parse via-point number
    int _via_num;
    _via_num = doc["via_num"].as< int >();

    Param->via_num = _via_num;

    // parse via-point time
    std::vector<double> _via_time;
    _via_time = doc["via_time"].as< std::vector<double> >();

    Param->via_time.resize( _via_num , 1 );
    for( int num = 0; num < _via_num; num++ )
        Param->via_time.coeffRef( num , 0 ) = _via_time[ num ];

    // parse via-point pose
    Param->joint_via_pose.resize( _via_num , MAX_JOINT_ID + 1 );
    Param->joint_via_dpose.resize( _via_num , MAX_JOINT_ID + 1 );
    Param->joint_via_ddpose.resize( _via_num , MAX_JOINT_ID + 1 );

    Param->joint_via_pose.fill( 0.0 );
    Param->joint_via_dpose.fill( 0.0 );
    Param->joint_via_ddpose.fill( 0.0 );

    YAML::Node _via_pose_node = doc["via_pose"];
    for(YAML::iterator _it = _via_pose_node.begin() ; _it != _via_pose_node.end() ; ++_it)
    {
        int _id;
        std::vector<double> _value;

        _id = _it->first.as<int>();
        _value = _it->second.as< std::vector<double> >();

        for( int num = 0; num < _via_num; num++ )
            Param->joint_via_pose.coeffRef( num , _id ) = _value[ num ] * deg2rad;
    }

    // parse target pose
    YAML::Node _tar_pose_node = doc["tar_pose"];
    for(YAML::iterator _it = _tar_pose_node.begin() ; _it != _tar_pose_node.end() ; ++_it)
    {
        int _id;
        double _value;

        _id = _it->first.as<int>();
        _value = _it->second.as<double>();

        Param->joint_ini_pose.coeffRef( _id , 0 ) = _value * deg2rad;
    }

    Param->all_time_steps = int( Param->mov_time / Param->smp_time ) + 1;
    Param->calc_joint_tra.resize( Param->all_time_steps , MAX_JOINT_ID + 1 );
}

void BaseModule::QueueThread()
{
	ros::NodeHandle     _ros_node;
	ros::CallbackQueue  _callback_queue;

	_ros_node.setCallbackQueue(&_callback_queue);

	/* subscribe topics */

	// for gui
	ros::Subscriber ini_pose_msg_sub = _ros_node.subscribe("/robotis/base/ini_pose", 5, &BaseModule::IniPoseMsgCallback, this);
	ros::Subscriber ini_ft_command_sub = _ros_node.subscribe("/robotis/base/ini_ft_command", 5, &BaseModule::IniFtMsgCallback, this);

	while(_ros_node.ok())
	{
		_callback_queue.callAvailable();
	}
}

void BaseModule::IniPoseMsgCallback( const std_msgs::String::ConstPtr& msg )
{
	if ( Param->is_moving == false )
	{
		if ( msg->data == "ini_pose")
		{
			// set module of all joints -> this module
			setCtrlModule(module_name);

			// wait to change module and to get goal position for init
			while(enable == false || has_goal_joints_ == false) usleep(8 * 1000);

            // parse initial pose
            std::string _ini_pose_path = ros::package::getPath("thormang3_base_module") + "/data/ini_pose.yaml";
            parseIniPoseData( _ini_pose_path );

			// generate trajectory
			tra_gene_tread_ = boost::thread(boost::bind(&BaseModule::IniposeTraGeneProc, this));
		}
	}
	else
		ROS_INFO("previous task is alive");

	return;
}

void BaseModule::IniFtMsgCallback(const std_msgs::String::ConstPtr& msg)
{
	if ( enable == true && ft_command_ == FT_NONE)
	{
		std::string _command = msg->data;

		if(_command == "ft_air")
			getFTAir();
		else if(_command == "ft_gnd")
			getFTGround();
		else if(_command == "ft_send")
			publishFTCalibrationData();
		else if(_command == "ft_save")
			saveFTCalibrationData(ft_calibration_data_path_);
	}
	else
		ROS_INFO("previous task is alive or not enable");

	return;
}

void BaseModule::IniposeTraGeneProc()
{
	for ( int id = 1; id <= MAX_JOINT_ID; id++ )
	{
		double ini_value = ROBOTIS_BASE::JointState::goal_joint_state[ id ].position;
		double tar_value = Param->joint_ini_pose.coeff( id , 0 );

        Eigen::MatrixXd tra;

        if ( Param->via_num == 0 )
        {
            tra = ROBOTIS_BASE::minimum_jerk_tra( ini_value , 0.0 , 0.0 ,
                                                  tar_value , 0.0 , 0.0 ,
                                                  Param->smp_time , Param->mov_time );
        }
        else
        {
            Eigen::MatrixXd via_value = Param->joint_via_pose.col( id );
            Eigen::MatrixXd d_via_value = Param->joint_via_dpose.col( id );
            Eigen::MatrixXd dd_via_value = Param->joint_via_ddpose.col( id );

            tra = ROBOTIS_BASE::minimum_jerk_tra_vian_qdqddq( Param->via_num,
                                                              ini_value , 0.0 , 0.0 ,
                                                              via_value , d_via_value, dd_via_value,
                                                              tar_value , 0.0 , 0.0 ,
                                                              Param->smp_time , Param->via_time , Param->mov_time );
        }

		Param->calc_joint_tra.block( 0 , id , Param->all_time_steps , 1 ) = tra;
	}

	Param->is_moving = true;
	Param->cnt = 0;
	ROS_INFO("[start] send trajectory");
}

void BaseModule::PoseGenProc(Eigen::MatrixXd _joint_angle_pose)
{
	setCtrlModule(module_name);
	while(enable == false || has_goal_joints_ == false) usleep(8 * 1000);

	Param->mov_time = 5.0;
	Param->all_time_steps = int( Param->mov_time / Param->smp_time ) + 1;

	Param->calc_joint_tra.resize( Param->all_time_steps , MAX_JOINT_ID + 1 );

	Param->joint_pose = _joint_angle_pose;

	for ( int id = 1; id <= MAX_JOINT_ID; id++ )
	{
		double ini_value = ROBOTIS_BASE::JointState::goal_joint_state[ id ].position;
		double tar_value = Param->joint_pose.coeff( id , 0 );

		ROS_INFO_STREAM("[ID : " << id << "] ini_value : " << ini_value << "  tar_value : " << tar_value);


        Eigen::MatrixXd tra = ROBOTIS_BASE::minimum_jerk_tra( ini_value , 0.0 , 0.0 ,
                                                              tar_value , 0.0 , 0.0 ,
                                                              Param->smp_time , Param->mov_time );


		Param->calc_joint_tra.block( 0 , id , Param->all_time_steps , 1 ) = tra;
	}

	Param->is_moving = true;
	Param->cnt = 0;
	ini_pose_only_ = true;
	ROS_INFO("[start] send trajectory");
}

bool BaseModule::IsRunning()
{
	return Param->is_moving;
}

void BaseModule::Process(std::map<std::string, Dynamixel *> dxls)
{
	if(enable == false)
		return;

	Eigen::MatrixXd _ft_right_data = Eigen::MatrixXd::Zero(6, 1);
	Eigen::MatrixXd _ft_left_data = Eigen::MatrixXd::Zero(6, 1);

	/*----- write curr position -----*/
	for(std::map<std::string, DynamixelState *>::iterator state_iter = result.begin(); state_iter != result.end(); state_iter++)
	{
		std::string _joint_name = state_iter->first;

        Dynamixel *_dxl = NULL;
        std::map<std::string, Dynamixel*>::iterator _dxl_it = dxls.find(_joint_name);
        if(_dxl_it != dxls.end())
            _dxl = _dxl_it->second;
        else
            continue;

		double _joint_curr_position = _dxl->dxl_state->present_position;
		double _joint_goal_position = _dxl->dxl_state->goal_position;

		for ( int id = 1; id <= MAX_JOINT_ID; id++ )
        {
            ROBOTIS_BASE::JointState::curr_joint_state[ joint_name_to_id[_joint_name] ].position = _joint_curr_position;
            ROBOTIS_BASE::JointState::goal_joint_state[ joint_name_to_id[_joint_name] ].position = _joint_goal_position;
		}

		// ft calibration
		if(ft_count_ > 0)
		{
			if(state_iter->first == "r_leg_an_p")
			{
				_ft_right_data.coeffRef(4, 0) = _dxl->dxl_state->ext_port_data[0];
				_ft_right_data.coeffRef(5, 0) = _dxl->dxl_state->ext_port_data[1];
			}
			else if(state_iter->first == "r_leg_an_r")
			{
				_ft_right_data.coeffRef(0, 0) = _dxl->dxl_state->ext_port_data[0];
				_ft_right_data.coeffRef(1, 0) = _dxl->dxl_state->ext_port_data[1];
				_ft_right_data.coeffRef(2, 0) = _dxl->dxl_state->ext_port_data[2];
				_ft_right_data.coeffRef(3, 0) = _dxl->dxl_state->ext_port_data[3];
			}
			else if(state_iter->first == "l_leg_an_p")
			{
				_ft_left_data.coeffRef(4, 0) = _dxl->dxl_state->ext_port_data[0];
				_ft_left_data.coeffRef(5, 0) = _dxl->dxl_state->ext_port_data[1];
			}
			else if(state_iter->first == "l_leg_an_r")
			{
				_ft_left_data.coeffRef(0, 0) = _dxl->dxl_state->ext_port_data[0];
				_ft_left_data.coeffRef(1, 0) = _dxl->dxl_state->ext_port_data[1];
				_ft_left_data.coeffRef(2, 0) = _dxl->dxl_state->ext_port_data[2];
				_ft_left_data.coeffRef(3, 0) = _dxl->dxl_state->ext_port_data[3];
			}
		}
	}

	/* -----        ft       ----- */
	if(ft_count_ > 0)
	{
		if(ft_command_ == FT_AIR)
		{
			ft_right_air += _ft_right_data;
			ft_left_air += _ft_left_data;
		}
		else if(ft_command_ == FT_GND)
		{
			ft_right_gnd += _ft_right_data;
			ft_left_gnd += _ft_left_data;
		}

		ft_count_ -= 1;
		ft_get_count_ += 1;
	}

	// complete to average ft data for 2sec
	if(ft_count_ <= 0 && ft_command_ != FT_NONE)
	{
		ROS_INFO("FT END Time");
		if(ft_command_ == FT_AIR)
		{
			ft_right_air /= ft_period_;
			ft_left_air /= ft_period_;

			std::cout << "period : " << ft_period_ << ", count : " << ft_get_count_ << std::endl;

			// post process
			calcFT(ft_right_air, ft_left_air);
			has_ft_air_ = true;

			// publish to user
			publishFTData(ft_command_, ft_right_air, ft_left_air);
		}
		else if(ft_command_ == FT_GND)
		{
			ft_right_gnd /= ft_period_;
			ft_left_gnd /= ft_period_;

			// post process
			calcFT(ft_right_gnd, ft_left_gnd);
			has_ft_gnd_ = true;

			// publish to user
			publishFTData(ft_command_, ft_right_gnd, ft_left_gnd);
		}

		ft_command_ = FT_NONE;
	}

	has_goal_joints_ = true;

	/* ----- send trajectory ----- */

	if ( Param->is_moving == true )
	{
		if ( Param->cnt == 1 )
		{
			Param->send_tra_msg.data = "start";
			send_tra_pub_.publish( Param->send_tra_msg );
		}

		for ( int id = 1; id <= MAX_JOINT_ID; id++ )
			ROBOTIS_BASE::JointState::goal_joint_state[ id ].position = Param->calc_joint_tra( Param->cnt , id );
	}

	/*----- set joint data -----*/

	for(std::map<std::string, DynamixelState *>::iterator state_iter = result.begin(); state_iter != result.end(); state_iter++)
	{
		std::string _joint_name = state_iter->first;

        for ( int id = 1; id <= MAX_JOINT_ID; id++ )
            result[ _joint_name ]->goal_position = ROBOTIS_BASE::JointState::goal_joint_state[ joint_name_to_id[_joint_name] ].position;
	}

	/*---------- initialize count number ----------*/

	Param->cnt++;

	if ( (Param->cnt >= Param->all_time_steps ) && (Param->is_moving == true ) )
	{
		ROS_INFO("[end] send trajectory");

		Param->send_tra_msg.data = "end";
		send_tra_pub_.publish( Param->send_tra_msg );

		Param->is_moving = false;
		Param->cnt = 0;

		// set all joints -> none
		if(ini_pose_only_ == true)
		{
			setCtrlModule("none");
			ini_pose_only_ = false;
		}
	}
}

void BaseModule::setCtrlModule(std::string module)
{
	robotis_controller_msgs::JointCtrlModule _control_msg;

	std::map<std::string, DynamixelState *>::iterator _joint_iter;

	for(_joint_iter = result.begin(); _joint_iter != result.end(); ++_joint_iter)
	{
		_control_msg.joint_name.push_back(_joint_iter->first);
		_control_msg.module_name.push_back(module);
	}

	set_ctrl_module_pub_.publish(_control_msg);
}

bool BaseModule::parseFTData(const std::string &path)
{
	YAML::Node doc;
	try
	{
		// load yaml
		doc = YAML::LoadFile(path.c_str());
	}
	catch(const std::exception& e)
	{
		ROS_ERROR("Fail to load ft_data yaml file.");
		return false;
	}

	std::vector<double> _ft;
    _ft = doc["ft_right_foot"].as< std::vector<double> >();
	ft_right_matrix = Eigen::Map<Eigen::MatrixXd>(_ft.data(), 6, 6);
	ft_right_matrix.transposeInPlace();
	std::cout << ft_right_matrix <<std::endl;

	_ft.clear();
	_ft = doc["ft_right_foot_unload"].as<std::vector<double> >();
	ft_right_unloaded = Eigen::Map<Eigen::MatrixXd>(_ft.data(), 6, 1);

	_ft.clear();
	_ft = doc["ft_left_foot"].as<std::vector<double> >();
	ft_left_matrix = Eigen::Map<Eigen::MatrixXd>(_ft.data(), 6, 6);
	ft_left_matrix.transposeInPlace();
	std::cout << ft_left_matrix <<std::endl;

	_ft.clear();
	_ft = doc["ft_left_foot_unload"].as<std::vector<double> >();
	ft_left_unloaded = Eigen::Map<Eigen::MatrixXd>(_ft.data(), 6, 1);

	return true;
}

void BaseModule::getFTAir()
{
	// start to average ft data in process
	ft_right_air = Eigen::MatrixXd::Zero(6, 1);
	ft_left_air = Eigen::MatrixXd::Zero(6, 1);

	ft_command_ = FT_AIR;
	ft_count_ = ft_period_;
	ft_get_count_ = 0;
}

void BaseModule::getFTGround()
{
	// start to average ft data in process
	ft_right_gnd = Eigen::MatrixXd::Zero(6, 1);
	ft_left_gnd = Eigen::MatrixXd::Zero(6, 1);

	ft_command_ = FT_GND;
	ft_count_ = ft_period_;
	ft_get_count_ = 0;
}

void BaseModule::calcFT(Eigen::MatrixXd &ft_right, Eigen::MatrixXd &ft_left)
{
	// calc
	ft_right = ft_right_matrix * (ft_right * 3.3 / 4095 - ft_right_unloaded);
	ft_left = ft_left_matrix * (ft_left * 3.3 / 4095 - ft_left_unloaded);

	// publish to walking_module
	//	publishFTData(ft_command_);
}

void BaseModule::saveFTCalibrationData(const std::string &path)
{
	if(has_ft_air_ == false || has_ft_gnd_ == false) return;

	YAML::Emitter _out;

	_out << YAML::BeginMap;

	// air - right
	std::vector<double> _ft_calibration;
	for(int ix = 0; ix < 6; ix++)
		_ft_calibration.push_back(ft_right_air.coeffRef(ix, 0));
	_out << YAML::Key << "ft_right_foot_air" << YAML::Value << _ft_calibration;

	// ground - right
	_ft_calibration.clear();
	for(int ix = 0; ix < 6; ix++)
		_ft_calibration.push_back(ft_right_gnd.coeffRef(ix, 0));
	_out << YAML::Key << "ft_right_foot_gnd" << YAML::Value << _ft_calibration;

	// air - left
	_ft_calibration.clear();
	for(int ix = 0; ix < 6; ix++)
		_ft_calibration.push_back(ft_left_air.coeffRef(ix, 0));
	_out << YAML::Key << "ft_left_foot_air" << YAML::Value << _ft_calibration;

	// ground - left
	_ft_calibration.clear();
	for(int ix = 0; ix < 6; ix++)
		_ft_calibration.push_back(ft_left_gnd.coeffRef(ix, 0));
	_out << YAML::Key << "ft_left_foot_gnd" << YAML::Value << _ft_calibration;

	_out << YAML::EndMap;

	// output to file
	std::ofstream fout(path.c_str());
	fout << _out.c_str();

	ROS_INFO("Save FT foot calibration data");
}

void BaseModule::publishFTData(int type, Eigen::MatrixXd &ft_right, Eigen::MatrixXd &ft_left)
{
	thormang3_base_module_msgs::BothWrench _wrench_msg;

	switch(type)
	{
	case FT_AIR:
		if(has_ft_air_ == false) return;
		_wrench_msg.name = "ft_air";
		break;

	case FT_GND:
		if(has_ft_gnd_ == false) return;
		_wrench_msg.name = "ft_gnd";
		break;

	default:
		return;
	}

	_wrench_msg.right.force.x = ft_right.coeffRef(0, 0);
	_wrench_msg.right.force.y = ft_right.coeffRef(1, 0);
	_wrench_msg.right.force.z = ft_right.coeffRef(2, 0);
	_wrench_msg.right.torque.x = ft_right.coeffRef(3, 0);
	_wrench_msg.right.torque.y = ft_right.coeffRef(4, 0);
	_wrench_msg.right.torque.z = ft_right.coeffRef(5, 0);

	_wrench_msg.left.force.x = ft_left.coeffRef(0, 0);
	_wrench_msg.left.force.y = ft_left.coeffRef(1, 0);
	_wrench_msg.left.force.z = ft_left.coeffRef(2, 0);
	_wrench_msg.left.torque.x = ft_left.coeffRef(3, 0);
	_wrench_msg.left.torque.y = ft_left.coeffRef(4, 0);
	_wrench_msg.left.torque.z = ft_left.coeffRef(5, 0);

	ini_ft_pub_.publish(_wrench_msg);
}

void BaseModule::publishFTCalibrationData()
{
	if(has_ft_air_ == false || has_ft_gnd_ == false) return;

	// publish
	thormang3_base_module_msgs::CalibrationWrench _both_ft_msg;

	_both_ft_msg.air_right.force.x = ft_right_air.coeffRef(0, 0);
	_both_ft_msg.air_right.force.y = ft_right_air.coeffRef(1, 0);
	_both_ft_msg.air_right.force.z = ft_right_air.coeffRef(2, 0);
	_both_ft_msg.air_right.torque.x = ft_right_air.coeffRef(3, 0);
	_both_ft_msg.air_right.torque.y = ft_right_air.coeffRef(4, 0);
	_both_ft_msg.air_right.torque.z = ft_right_air.coeffRef(5, 0);

	_both_ft_msg.air_left.force.x = ft_left_air.coeffRef(0, 0);
	_both_ft_msg.air_left.force.y = ft_left_air.coeffRef(1, 0);
	_both_ft_msg.air_left.force.z = ft_left_air.coeffRef(2, 0);
	_both_ft_msg.air_left.torque.x = ft_left_air.coeffRef(3, 0);
	_both_ft_msg.air_left.torque.y = ft_left_air.coeffRef(4, 0);
	_both_ft_msg.air_left.torque.z = ft_left_air.coeffRef(5, 0);

	_both_ft_msg.ground_right.force.x = ft_right_gnd.coeffRef(0, 0);
	_both_ft_msg.ground_right.force.y = ft_right_gnd.coeffRef(1, 0);
	_both_ft_msg.ground_right.force.z = ft_right_gnd.coeffRef(2, 0);
	_both_ft_msg.ground_right.torque.x = ft_right_gnd.coeffRef(3, 0);
	_both_ft_msg.ground_right.torque.y = ft_right_gnd.coeffRef(4, 0);
	_both_ft_msg.ground_right.torque.z = ft_right_gnd.coeffRef(5, 0);

	_both_ft_msg.ground_left.force.x = ft_left_gnd.coeffRef(0, 0);
	_both_ft_msg.ground_left.force.y = ft_left_gnd.coeffRef(1, 0);
	_both_ft_msg.ground_left.force.z = ft_left_gnd.coeffRef(2, 0);
	_both_ft_msg.ground_left.torque.x = ft_left_gnd.coeffRef(3, 0);
	_both_ft_msg.ground_left.torque.y = ft_left_gnd.coeffRef(4, 0);
	_both_ft_msg.ground_left.torque.z = ft_left_gnd.coeffRef(5, 0);

	ROS_INFO("Send FT data to walking_module");

	both_ft_pub_.publish(_both_ft_msg);

	// init value
	// has_ft_air_ = false;
	// has_ft_gnd_ = false;
}
