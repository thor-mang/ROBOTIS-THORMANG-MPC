/*
 * ThorManipulation.cpp
 *
 *  Created on: 2016. 1. 18.
 *      Author: zerom
 */

#include <stdio.h>
#include "thormang3_base_module/base_module.h"

#define EXT_PORT_DATA_1 "external_port_data_1"
#define EXT_PORT_DATA_2 "external_port_data_2"
#define EXT_PORT_DATA_3 "external_port_data_3"
#define EXT_PORT_DATA_4 "external_port_data_4"

using namespace thormang3;

BaseModule::BaseModule()
    : control_cycle_msec_(0),
      has_goal_joints_(false),
      ini_pose_only_(false)
{
    enable_         = false;
    module_name_    = "base_module";
    control_mode_   = robotis_framework::PositionControl;

    /* arm */
    result_["r_arm_sh_p1"]  = new robotis_framework::DynamixelState();
    result_["l_arm_sh_p1"]  = new robotis_framework::DynamixelState();
    result_["r_arm_sh_r"]   = new robotis_framework::DynamixelState();
    result_["l_arm_sh_r"]   = new robotis_framework::DynamixelState();
    result_["r_arm_sh_p2"]  = new robotis_framework::DynamixelState();
    result_["l_arm_sh_p2"]  = new robotis_framework::DynamixelState();
    result_["r_arm_el_y"]   = new robotis_framework::DynamixelState();
    result_["l_arm_el_y"]   = new robotis_framework::DynamixelState();
    result_["r_arm_wr_r"]   = new robotis_framework::DynamixelState();
    result_["l_arm_wr_r"]   = new robotis_framework::DynamixelState();
    result_["r_arm_wr_y"]   = new robotis_framework::DynamixelState();
    result_["l_arm_wr_y"]   = new robotis_framework::DynamixelState();
    result_["r_arm_wr_p"]   = new robotis_framework::DynamixelState();
    result_["l_arm_wr_p"]   = new robotis_framework::DynamixelState();

    /* gripper */
    result_["r_arm_grip"]   = new robotis_framework::DynamixelState();
    result_["l_arm_grip"]   = new robotis_framework::DynamixelState();

    /* body */
    result_["torso_y"]      = new robotis_framework::DynamixelState();

    /* leg */
    result_["r_leg_hip_y"]  = new robotis_framework::DynamixelState();
    result_["r_leg_hip_r"]  = new robotis_framework::DynamixelState();
    result_["r_leg_hip_p"]  = new robotis_framework::DynamixelState();
    result_["r_leg_kn_p"]   = new robotis_framework::DynamixelState();
    result_["r_leg_an_p"]   = new robotis_framework::DynamixelState();
    result_["r_leg_an_r"]   = new robotis_framework::DynamixelState();

    result_["l_leg_hip_y"]  = new robotis_framework::DynamixelState();
    result_["l_leg_hip_r"]  = new robotis_framework::DynamixelState();
    result_["l_leg_hip_p"]  = new robotis_framework::DynamixelState();
    result_["l_leg_kn_p"]   = new robotis_framework::DynamixelState();
    result_["l_leg_an_p"]   = new robotis_framework::DynamixelState();
    result_["l_leg_an_r"]   = new robotis_framework::DynamixelState();

    /* head */
    result_["head_y"]       = new robotis_framework::DynamixelState();
    result_["head_p"]       = new robotis_framework::DynamixelState();

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

    Robotis = new RobotisState();
    JointState = new BaseJointState();
}

BaseModule::~BaseModule()
{
    queue_thread_.join();
}

void BaseModule::initialize(const int control_cycle_msec, robotis_framework::Robot *robot)
{
    control_cycle_msec_ = control_cycle_msec;
    queue_thread_ = boost::thread(boost::bind(&BaseModule::QueueThread, this));

    ros::NodeHandle _ros_node;

    /* publish topics */
    status_msg_pub_         = _ros_node.advertise<robotis_controller_msgs::StatusMsg>("/robotis/status", 1);
    set_ctrl_module_pub_	= _ros_node.advertise<std_msgs::String>("/robotis/enable_ctrl_module", 1);
}

void BaseModule::ParseIniPoseData( const std::string &path )
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

    Robotis->mov_time = _mov_time;

    // parse via-point number
    int _via_num;
    _via_num = doc["via_num"].as< int >();

    Robotis->via_num = _via_num;

    // parse via-point time
    std::vector<double> _via_time;
    _via_time = doc["via_time"].as< std::vector<double> >();

    Robotis->via_time.resize( _via_num , 1 );
    for( int num = 0; num < _via_num; num++ )
        Robotis->via_time.coeffRef( num , 0 ) = _via_time[ num ];

    // parse via-point pose
    Robotis->joint_via_pose.resize( _via_num , MAX_JOINT_ID + 1 );
    Robotis->joint_via_dpose.resize( _via_num , MAX_JOINT_ID + 1 );
    Robotis->joint_via_ddpose.resize( _via_num , MAX_JOINT_ID + 1 );

    Robotis->joint_via_pose.fill( 0.0 );
    Robotis->joint_via_dpose.fill( 0.0 );
    Robotis->joint_via_ddpose.fill( 0.0 );

    YAML::Node _via_pose_node = doc["via_pose"];
    for(YAML::iterator _it = _via_pose_node.begin() ; _it != _via_pose_node.end() ; ++_it)
    {
        int _id;
        std::vector<double> _value;

        _id = _it->first.as<int>();
        _value = _it->second.as< std::vector<double> >();

        for( int num = 0; num < _via_num; num++ )
            Robotis->joint_via_pose.coeffRef( num , _id ) = _value[ num ] * DEGREE2RADIAN;
    }

    // parse target pose
    YAML::Node _tar_pose_node = doc["tar_pose"];
    for(YAML::iterator _it = _tar_pose_node.begin() ; _it != _tar_pose_node.end() ; ++_it)
    {
        int _id;
        double _value;

        _id = _it->first.as<int>();
        _value = _it->second.as<double>();

        Robotis->joint_ini_pose.coeffRef( _id , 0 ) = _value * DEGREE2RADIAN;
    }

    Robotis->all_time_steps = int( Robotis->mov_time / Robotis->smp_time ) + 1;
    Robotis->calc_joint_tra.resize( Robotis->all_time_steps , MAX_JOINT_ID + 1 );
}

void BaseModule::QueueThread()
{
    ros::NodeHandle     _ros_node;
    ros::CallbackQueue  _callback_queue;

    _ros_node.setCallbackQueue(&_callback_queue);

    /* subscribe topics */

    // for gui
    ros::Subscriber ini_pose_msg_sub = _ros_node.subscribe("/robotis/base/ini_pose", 5, &BaseModule::IniPoseMsgCallback, this);

    while(_ros_node.ok())
    {
        _callback_queue.callAvailable();
        usleep(100);
    }
}

void BaseModule::IniPoseMsgCallback( const std_msgs::String::ConstPtr& msg )
{
    if ( Robotis->is_moving == false )
    {
        if ( msg->data == "ini_pose")
        {
            // set module of all joints -> this module
            SetCtrlModule(module_name_);

            // wait to change module and to get goal position for init
            while(enable_ == false || has_goal_joints_ == false) usleep(8 * 1000);

            // parse initial pose
            std::string _ini_pose_path = ros::package::getPath("thormang3_base_module") + "/data/ini_pose.yaml";
            ParseIniPoseData( _ini_pose_path );

            // generate trajectory
            tra_gene_tread_ = boost::thread(boost::bind(&BaseModule::IniposeTraGeneProc, this));
        }
    }
    else
        ROS_INFO("previous task is alive");

    return;
}

void BaseModule::IniposeTraGeneProc()
{
    for ( int id = 1; id <= MAX_JOINT_ID; id++ )
    {
        double ini_value = JointState->goal_joint_state[ id ].position;
        double tar_value = Robotis->joint_ini_pose.coeff( id , 0 );

        Eigen::MatrixXd tra;

        if ( Robotis->via_num == 0 )
        {
            tra = robotis_framework::calcMinimumJerkTra(ini_value , 0.0 , 0.0 ,
                                                        tar_value , 0.0 , 0.0 ,
                                                        Robotis->smp_time , Robotis->mov_time );
        }
        else
        {
            Eigen::MatrixXd via_value = Robotis->joint_via_pose.col( id );
            Eigen::MatrixXd d_via_value = Robotis->joint_via_dpose.col( id );
            Eigen::MatrixXd dd_via_value = Robotis->joint_via_ddpose.col( id );

            tra = robotis_framework::calcMinimumJerkTraWithViaPoints(Robotis->via_num,
                                                                     ini_value , 0.0 , 0.0 ,
                                                                     via_value , d_via_value, dd_via_value,
                                                                     tar_value , 0.0 , 0.0 ,
                                                                     Robotis->smp_time , Robotis->via_time , Robotis->mov_time );
        }

        Robotis->calc_joint_tra.block( 0 , id , Robotis->all_time_steps , 1 ) = tra;
    }

    Robotis->is_moving = true;
    Robotis->cnt = 0;
    ROS_INFO("[start] send trajectory");
}

void BaseModule::PoseGenProc(Eigen::MatrixXd _joint_angle_pose)
{
    SetCtrlModule(module_name_);
    while(enable_ == false || has_goal_joints_ == false) usleep(8 * 1000);

    Robotis->mov_time = 5.0;
    Robotis->all_time_steps = int( Robotis->mov_time / Robotis->smp_time ) + 1;

    Robotis->calc_joint_tra.resize( Robotis->all_time_steps , MAX_JOINT_ID + 1 );

    Robotis->joint_pose = _joint_angle_pose;

    for ( int id = 1; id <= MAX_JOINT_ID; id++ )
    {
        double ini_value = JointState->goal_joint_state[ id ].position;
        double tar_value = Robotis->joint_pose.coeff( id , 0 );

        ROS_INFO_STREAM("[ID : " << id << "] ini_value : " << ini_value << "  tar_value : " << tar_value);


        Eigen::MatrixXd tra = robotis_framework::calcMinimumJerkTra(ini_value , 0.0 , 0.0 ,
                                                                    tar_value , 0.0 , 0.0 ,
                                                                    Robotis->smp_time , Robotis->mov_time );


        Robotis->calc_joint_tra.block( 0 , id , Robotis->all_time_steps , 1 ) = tra;
    }

    Robotis->is_moving = true;
    Robotis->cnt = 0;
    ini_pose_only_ = true;
    ROS_INFO("[start] send trajectory");
}

void 	BaseModule::PoseGenProc(std::map<std::string, double>& joint_angle_pose)
{
    SetCtrlModule(module_name_);
    while(enable_ == false || has_goal_joints_ == false) usleep(8 * 1000);

	Eigen::MatrixXd _target_pose = Eigen::MatrixXd::Zero( MAX_JOINT_ID + 1 , 1 );

	for(std::map<std::string, double>::iterator _it = joint_angle_pose.begin(); _it != joint_angle_pose.end(); _it++)
	{
		std::string _joint_name = _it->first;
		double _joint_angle_rad = _it->second;

		std::map<std::string, int>::iterator _joint_name_to_id_it = joint_name_to_id.find(_joint_name);
		if(_joint_name_to_id_it != joint_name_to_id.end()) {
			_target_pose.coeffRef(_joint_name_to_id_it->second, 0) = _joint_angle_rad;
		}
	}

	Robotis->joint_pose = _target_pose;

    Robotis->mov_time = 5.0;
    Robotis->all_time_steps = int( Robotis->mov_time / Robotis->smp_time ) + 1;

    Robotis->calc_joint_tra.resize( Robotis->all_time_steps , MAX_JOINT_ID + 1 );

    for ( int id = 1; id <= MAX_JOINT_ID; id++ )
    {
        double ini_value = JointState->goal_joint_state[ id ].position;
        double tar_value = Robotis->joint_pose.coeff( id , 0 );

        ROS_INFO_STREAM("[ID : " << id << "] ini_value : " << ini_value << "  tar_value : " << tar_value);


        Eigen::MatrixXd tra = robotis_framework::calcMinimumJerkTra(ini_value , 0.0 , 0.0 ,
                                                                    tar_value , 0.0 , 0.0 ,
                                                                    Robotis->smp_time , Robotis->mov_time );


        Robotis->calc_joint_tra.block( 0 , id , Robotis->all_time_steps , 1 ) = tra;
    }

    Robotis->is_moving = true;
    Robotis->cnt = 0;
    ini_pose_only_ = true;
    ROS_INFO("[start] send trajectory");
}

bool BaseModule::isRunning()
{
    return Robotis->is_moving;
}

void BaseModule::process(std::map<std::string, robotis_framework::Dynamixel *> dxls, std::map<std::string, double> sensors)
{
    if(enable_ == false)
        return;

    /*----- write curr position -----*/
    for(std::map<std::string, robotis_framework::DynamixelState *>::iterator state_iter = result_.begin(); state_iter != result_.end(); state_iter++)
    {
        std::string _joint_name = state_iter->first;

        robotis_framework::Dynamixel *_dxl = NULL;
        std::map<std::string, robotis_framework::Dynamixel*>::iterator _dxl_it = dxls.find(_joint_name);
        if(_dxl_it != dxls.end())
            _dxl = _dxl_it->second;
        else
            continue;

        double _joint_curr_position = _dxl->dxl_state_->present_position_;
        double _joint_goal_position = _dxl->dxl_state_->goal_position_;

        JointState->curr_joint_state[ joint_name_to_id[_joint_name] ].position = _joint_curr_position;
        JointState->goal_joint_state[ joint_name_to_id[_joint_name] ].position = _joint_goal_position;
    }

    has_goal_joints_ = true;

    /* ----- send trajectory ----- */

    if ( Robotis->is_moving == true )
    {
        if ( Robotis->cnt == 1 )
            PublishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Start Init Pose");


        for ( int id = 1; id <= MAX_JOINT_ID; id++ )
            JointState->goal_joint_state[ id ].position = Robotis->calc_joint_tra( Robotis->cnt , id );

        Robotis->cnt++;
    }

    /*----- set joint data -----*/

    for(std::map<std::string, robotis_framework::DynamixelState *>::iterator state_iter = result_.begin(); state_iter != result_.end(); state_iter++)
    {
        std::string _joint_name = state_iter->first;

        result_[ _joint_name ]->goal_position_ = JointState->goal_joint_state[ joint_name_to_id[_joint_name] ].position;
    }

    /*---------- initialize count number ----------*/

    if ( (Robotis->cnt >= Robotis->all_time_steps ) && (Robotis->is_moving == true ) )
    {
        ROS_INFO("[end] send trajectory");

        PublishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Finish Init Pose");

        Robotis->is_moving = false;
        Robotis->cnt = 0;

        // set all joints -> none
        if(ini_pose_only_ == true)
        {
            SetCtrlModule("none");
            ini_pose_only_ = false;
        }
    }
}

void BaseModule::stop()
{
    return;
}

void BaseModule::SetCtrlModule(std::string module)
{
    std_msgs::String _control_msg;
    _control_msg.data = module_name_;

    set_ctrl_module_pub_.publish(_control_msg);
}

void BaseModule::PublishStatusMsg(unsigned int type, std::string msg)
{
    robotis_controller_msgs::StatusMsg _status;
    _status.header.stamp = ros::Time::now();
    _status.type = type;
    _status.module_name = "Base";
    _status.status_msg = msg;

    status_msg_pub_.publish(_status);
}
