/*
 * ThorManipulation.cpp
 *
 *  Created on: 2016. 1. 18.
 *      Author: zerom
 */

#include <stdio.h>
#include "thormang3_manipulation_module/ManipulationModule.h"

using namespace ROBOTIS;

ManipulationModule *ManipulationModule::unique_instance_ = new ManipulationModule();

ManipulationModule::ManipulationModule()
    : control_cycle_msec_(0)
{
    enable          = false;
    module_name     = "manipulation_module";
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

    /* body */
    result["torso_y"]       = new DynamixelState();

    /* gripper */
    result["r_arm_grip"]    = new DynamixelState();
    result["l_arm_grip"]    = new DynamixelState();

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

    /* body */
    joint_name_to_id["torso_y"]     = 27;

    /* gripper */
    joint_name_to_id["r_arm_grip"]  = 31;
    joint_name_to_id["l_arm_grip"]  = 30;

    /* etc */
    joint_name_to_id["r_arm_end"] = 35;
    joint_name_to_id["l_arm_end"] = 34;

    Humanoid = new ROBOTIS_MANIPULATION::RobotisData( ROBOTIS_MANIPULATION::WHOLE_BODY );
    Param = new ROBOTIS_MANIPULATION::RobotisState();
}

ManipulationModule::~ManipulationModule()
{
    queue_thread_.join();
}

void ManipulationModule::Initialize(const int control_cycle_msec)
{
    control_cycle_msec_ = control_cycle_msec;
    queue_thread_ = boost::thread(boost::bind(&ManipulationModule::QueueThread, this));

    ros::NodeHandle _ros_node;

    /* publish topics */

    // for gui
    send_tra_pub_           = _ros_node.advertise<std_msgs::String>("/robotis/manipulation/send_tra", 1);
    
    std::string _path = ros::package::getPath("thormang3_manipulation_module") + "/config/ik_weight.yaml";
    parseData( _path );

}

void ManipulationModule::parseData( const std::string &path )
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
        return;
    }

    YAML::Node _ik_weight_node = doc["weight_value"];
    for(YAML::iterator _it = _ik_weight_node.begin(); _it != _ik_weight_node.end() ; ++_it)
    {
        int _id = _it->first.as<int>();
        double _value = _it->second.as<double>();

        Param->ik_weight.coeffRef( _id , 0 ) = _value;
    }
}

void ManipulationModule::parseIniPoseData( const std::string &path )
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

void ManipulationModule::QueueThread()
{
    ros::NodeHandle     _ros_node;
    ros::CallbackQueue  _callback_queue;

    _ros_node.setCallbackQueue(&_callback_queue);

    /* subscribe topics */

    // for gui
    ros::Subscriber ini_pose_msg_sub = _ros_node.subscribe("/robotis/manipulation/ini_pose_msg", 5, &ManipulationModule::IniPoseMsgCallback, this);
    ros::Subscriber joint_des_msg_sub = _ros_node.subscribe("/robotis/manipulation/des_joint_msg", 5, &ManipulationModule::JointDesMsgCallback, this);
    ros::Subscriber ik_msg_sub = _ros_node.subscribe("/robotis/manipulation/ik_msg", 5, &ManipulationModule::IkMsgCallback, this);

    ros::Subscriber demo_msg_sub = _ros_node.subscribe("/robotis/manipulation/demo_msg", 5, &ManipulationModule::DemoMsgCallback, this);
    ros::Subscriber bimanual_msg_sub = _ros_node.subscribe("/robotis/manipulation/bimanual_msg", 5, &ManipulationModule::BiManualCallback, this);

    ros::ServiceServer get_joint_pose_server = _ros_node.advertiseService("/robotis/manipulation/get_joint_pose", &ManipulationModule::GetJointPoseCallback, this);
    ros::ServiceServer get_kinematics_pose_server = _ros_node.advertiseService("/robotis/manipulation/get_kinematics_pose", &ManipulationModule::GetKinematicsPoseCallback, this);

    while(_ros_node.ok())
    {
        _callback_queue.callAvailable();
    }
}

void ManipulationModule::IniPoseMsgCallback( const std_msgs::String::ConstPtr& msg )
{
    if(enable == false)
        return;

    if ( Param->is_moving == false )
    {
        if ( msg->data == "ini_pose")
        {
            // parse initial pose
            std::string _ini_pose_path = ros::package::getPath("thormang3_manipulation_module") + "/config/ini_pose.yaml";
            parseIniPoseData( _ini_pose_path );

            tra_gene_tread_ = new boost::thread(boost::bind(&ManipulationModule::IniposeTraGeneProc, this));
            delete tra_gene_tread_;
        }
    }
    else
        ROS_INFO("previous task is alive");

    return;
}

bool ManipulationModule::GetJointPoseCallback( thormang3_manipulation_module_msgs::GetJointPose::Request &req , thormang3_manipulation_module_msgs::GetJointPose::Response &res )
{
    for ( int _name_index = 1; _name_index <= MAX_JOINT_ID; _name_index++ )
    {
        if( Humanoid->robotis_joint[ _name_index ]->name == req.joint_name )
        {
            res.joint_value = ROBOTIS_MANIPULATION::JointState::goal_joint_state[ _name_index ].position ;
            return true;
        }
    }

    return false;
}

bool ManipulationModule::GetKinematicsPoseCallback( thormang3_manipulation_module_msgs::GetKinematicsPose::Request &req , thormang3_manipulation_module_msgs::GetKinematicsPose::Response &res )
{
    int _end_index;

    if ( req.group_name == "left_arm" )
        _end_index = id_l_arm_end;
    else if ( req.group_name == "right_arm")
        _end_index = id_r_arm_end;
    else if ( req.group_name == "left_arm_with_torso" )
        _end_index = id_l_arm_end;
    else if ( req.group_name == "right_arm_with_torso" )
        _end_index = id_r_arm_end;
    else
        return false;

    res.group_pose.position.x = Humanoid->robotis_joint[ _end_index ]->position.coeff(0,0);
    res.group_pose.position.y = Humanoid->robotis_joint[ _end_index ]->position.coeff(1,0);
    res.group_pose.position.z = Humanoid->robotis_joint[ _end_index ]->position.coeff(2,0);

    Eigen::Quaterniond _quaternion = ROBOTIS_MANIPULATION::rotation2quaternion( Humanoid->robotis_joint[ _end_index ]->orientation );

    res.group_pose.orientation.w = _quaternion.w();
    res.group_pose.orientation.x = _quaternion.x();
    res.group_pose.orientation.y = _quaternion.y();
    res.group_pose.orientation.z = _quaternion.z();

    return  true;
}

void ManipulationModule::IkMsgCallback( const thormang3_manipulation_module_msgs::KinematicsPose::ConstPtr& msg )
{
    if(enable == false)
        return;

    Param->goal_kinematics_pose_msg = *msg;

    if ( Param->goal_kinematics_pose_msg.name == "left_arm" )
    {
        Param->ik_id_start = id_l_arm_start;
        Param->ik_id_end = id_l_arm_end;
    }
    else if ( Param->goal_kinematics_pose_msg.name == "right_arm" )
    {
        Param->ik_id_start = id_r_arm_start;
        Param->ik_id_end = id_r_arm_end;
    }
    else if ( Param->goal_kinematics_pose_msg.name == "left_arm_with_torso" )
    {
        Param->ik_id_start = id_torso;
        Param->ik_id_end = id_l_arm_end;
    }
    else if ( Param->goal_kinematics_pose_msg.name == "right_arm_with_torso" )
    {
        Param->ik_id_start = id_torso;
        Param->ik_id_end = id_r_arm_end;
    }

    if ( Param->is_moving == false )
    {
        tra_gene_tread_ = new boost::thread(boost::bind(&ManipulationModule::TaskTraGeneProc, this));
        delete tra_gene_tread_;
    }
    else
        ROS_INFO("previous task is alive");

    return;
}

void ManipulationModule::DemoMsgCallback( const thormang3_manipulation_module_msgs::DemoPose::ConstPtr& msg )
{
    Param->goal_demo_pose_msg = *msg;

    if ( Param->goal_demo_pose_msg.name == "left_arm" )
    {
        Param->ik_id_start = id_l_arm_start;
        Param->ik_id_end = id_l_arm_end;
    }
    else if ( Param->goal_demo_pose_msg.name == "right_arm" )
    {
        Param->ik_id_start = id_r_arm_start;
        Param->ik_id_end = id_r_arm_end;
    }
    else if ( Param->goal_demo_pose_msg.name == "left_arm_with_torso" )
    {
        Param->ik_id_start = id_torso;
        Param->ik_id_end = id_l_arm_end;
    }
    else if ( Param->goal_demo_pose_msg.name == "right_arm_with_torso" )
    {
        Param->ik_id_start = id_torso;
        Param->ik_id_end = id_r_arm_end;
    }

    if ( Param->is_moving == false )
    {
        if ( Param->goal_demo_pose_msg.demo == "ini_pose" )
        {
            tra_gene_tread_ = new boost::thread(boost::bind(&ManipulationModule::DemoIniTraGeneProc, this));
            delete tra_gene_tread_;
        }
        else if ( Param->goal_demo_pose_msg.demo == "line" )
        {
            tra_gene_tread_ = new boost::thread(boost::bind(&ManipulationModule::DemoLineTraGeneProc, this));
            delete tra_gene_tread_;
        }
        else if ( Param->goal_demo_pose_msg.demo == "circle" )
        {
            tra_gene_tread_ = new boost::thread(boost::bind(&ManipulationModule::DemoCircleTraGeneProc, this));
            delete tra_gene_tread_;
        }
        else
            ROS_INFO("there is no demo");
    }
    else
        ROS_INFO("previous task is alive");

}

void ManipulationModule::BiManualCallback( const std_msgs::String::ConstPtr& msg )
{
    if ( msg->data == "ini_pose" )
    {
        tra_gene_tread_ = new boost::thread(boost::bind(&ManipulationModule::BiManualIniTraGeneProc, this));
        delete tra_gene_tread_;
    }
    else if ( msg->data == "demo" )
    {
        tra_gene_tread_ = new boost::thread(boost::bind(&ManipulationModule::BiManualDemoTraGeneProc, this));
        delete tra_gene_tread_;
    }
    else
        ROS_INFO("previous task is alive");

    return;
}

void ManipulationModule::JointDesMsgCallback( const thormang3_manipulation_module_msgs::JointPose::ConstPtr& msg )
{
    Param->goal_joint_pose_msg = *msg;

    if ( Param->is_moving == false )
    {
        tra_gene_tread_ = new boost::thread(boost::bind(&ManipulationModule::JointTraGeneProc, this));
        delete tra_gene_tread_;
    }
    else
        ROS_INFO("previous task is alive");

    return;
}

void ManipulationModule::IniposeTraGeneProc()
{
    for ( int id = 1; id <= MAX_JOINT_ID; id++ )
    {
        double ini_value = ROBOTIS_MANIPULATION::JointState::goal_joint_state[ id ].position;
        double tar_value = Param->joint_ini_pose.coeff( id , 0 );

        Eigen::MatrixXd tra = ROBOTIS_MANIPULATION::minimum_jerk_tra( ini_value , 0.0 , 0.0 ,
                                                                      tar_value , 0.0 , 0.0 ,
                                                                      Param->smp_time , Param->mov_time );

        Param->calc_joint_tra.block( 0 , id , Param->all_time_steps , 1 ) = tra;
    }

    Param->cnt = 0;
    Param->is_moving = true;

    ROS_INFO("[start] send trajectory");
}

void ManipulationModule::JointTraGeneProc()
{
    ROS_INFO("%s", Param->goal_joint_pose_msg.name.c_str() );
    ROS_INFO("%f", Param->goal_joint_pose_msg.value );

    /* set movement time */
    double _tol = 10 * deg2rad; // rad per sec
    double _mov_time = 2.0;

    int _ctrl_id = joint_name_to_id[ Param->goal_joint_pose_msg.name ];

    double _ini_value = ROBOTIS_MANIPULATION::JointState::goal_joint_state[ _ctrl_id ].position;
    double _tar_value = Param->goal_joint_pose_msg.value;
    double _diff = fabs ( _tar_value - _ini_value );

    Param->mov_time =  _diff / _tol;
    int _all_time_steps = int( floor( ( Param->mov_time / Param->smp_time ) + 1.0 ) );
    Param->mov_time = double ( _all_time_steps - 1 ) * Param->smp_time;

    if ( Param->mov_time < _mov_time )
        Param->mov_time = _mov_time;

    Param->all_time_steps = int( Param->mov_time / Param->smp_time ) + 1;

    Param->calc_joint_tra.resize( Param->all_time_steps , MAX_JOINT_ID + 1 );

    /* calculate joint trajectory */
    for ( int id = 1; id <= MAX_JOINT_ID; id++ )
    {
        double ini_value = ROBOTIS_MANIPULATION::JointState::goal_joint_state[ id ].position;
        double tar_value = ROBOTIS_MANIPULATION::JointState::goal_joint_state[ id ].position;

        if( Humanoid->robotis_joint[ id ]->name == Param->goal_joint_pose_msg.name )
            tar_value = Param->goal_joint_pose_msg.value;

        Eigen::MatrixXd tra = ROBOTIS_MANIPULATION::minimum_jerk_tra( ini_value , 0.0 , 0.0 ,
                                                                      tar_value , 0.0 , 0.0 ,
                                                                      Param->smp_time , Param->mov_time );

        Param->calc_joint_tra.block( 0 , id , Param->all_time_steps , 1 ) = tra;
    }

    Param->cnt = 0;
    Param->is_moving = true;

    ROS_INFO("[start] send trajectory");
}

void ManipulationModule::TaskTraGeneProc()
{
    /* set movement time */
    double _tol = 0.1; // m per sec
    double _mov_time = 2.0;

    double _diff = sqrt( pow( Humanoid->robotis_joint[ Param->ik_id_end ]->position.coeff( 0 , 0 ) - Param->goal_kinematics_pose_msg.pose.position.x , 2  ) +
                         pow( Humanoid->robotis_joint[ Param->ik_id_end ]->position.coeff( 1 , 0 ) - Param->goal_kinematics_pose_msg.pose.position.y , 2  ) +
                         pow( Humanoid->robotis_joint[ Param->ik_id_end ]->position.coeff( 2 , 0 ) - Param->goal_kinematics_pose_msg.pose.position.z , 2  ) );

    Param->mov_time = _diff / _tol;
    int _all_time_steps = int( floor( ( Param->mov_time / Param->smp_time ) + 1.0 ) );
    Param->mov_time = double ( _all_time_steps - 1 ) * Param->smp_time;

    if ( Param->mov_time < _mov_time )
        Param->mov_time = _mov_time;

    Param->all_time_steps = int( Param->mov_time / Param->smp_time ) + 1;

    Param->calc_task_tra.resize( Param->all_time_steps , 3 );

     /* calculate trajectory */
    for ( int dim = 0; dim < 3; dim++ )
    {
        double ini_value = Humanoid->robotis_joint[ Param->ik_id_end ]->position.coeff( dim , 0 );
        double tar_value;
        if ( dim == 0 )
            tar_value = Param->goal_kinematics_pose_msg.pose.position.x;
        else if ( dim == 1 )
            tar_value = Param->goal_kinematics_pose_msg.pose.position.y;
        else if ( dim == 2 )
            tar_value = Param->goal_kinematics_pose_msg.pose.position.z;

        Eigen::MatrixXd tra = ROBOTIS_MANIPULATION::minimum_jerk_tra( ini_value , 0.0 , 0.0 ,
                                                                      tar_value , 0.0 , 0.0 ,
                                                                      Param->smp_time , Param->mov_time );

        Param->calc_task_tra.block( 0 , dim , Param->all_time_steps , 1 ) = tra;
    }

    Param->cnt = 0;
    Param->is_moving = true;
    Param->ik_solve = true;

    ROS_INFO("[start] send trajectory");
}

void ManipulationModule::DemoIniTraGeneProc()
{
    ROS_INFO("figure demo : go initial pose");

    std::string _demo_path = ros::package::getPath("thormang3_manipulation_module") + "/config/figure_demo.yaml";

    YAML::Node doc;
    try
    {
        // load yaml
        doc = YAML::LoadFile( _demo_path.c_str() );
    }
    catch(const std::exception& e)
    {
        ROS_ERROR("Fail to load yaml file.");
        return ;
    }

    YAML::Node _ini_pose_node = doc["demo_ini_pose"];

    Param->mov_time = _ini_pose_node["mov_time"].as< double >();
    Param->all_time_steps = int( Param->mov_time / Param->smp_time ) + 1;

    Param->calc_task_tra.resize( Param->all_time_steps , 3 );

    std::vector<double> _tar_value;

    if ( Param->goal_demo_pose_msg.name == "left_arm" )
        _tar_value = _ini_pose_node["left_position"].as< std::vector<double> >();
    else if ( Param->goal_demo_pose_msg.name == "right_arm" )
        _tar_value = _ini_pose_node["right_position"].as< std::vector<double> >();
    else if ( Param->goal_demo_pose_msg.name == "left_arm_with_torso" )
        _tar_value = _ini_pose_node["left_position"].as< std::vector<double> >();
    else if ( Param->goal_demo_pose_msg.name == "right_arm_with_torso" )
        _tar_value = _ini_pose_node["right_position"].as< std::vector<double> >();

    /* calculate trajectory */
   for ( int dim = 0; dim < 3; dim++ )
   {
       double ini_value = Humanoid->robotis_joint[ Param->ik_id_end ]->position.coeff( dim , 0 );
       double tar_value = _tar_value[ dim ];

       Eigen::MatrixXd tra = ROBOTIS_MANIPULATION::minimum_jerk_tra( ini_value , 0.0 , 0.0 ,
                                                                     tar_value , 0.0 , 0.0 ,
                                                                     Param->smp_time , Param->mov_time );

       Param->calc_task_tra.block( 0 , dim , Param->all_time_steps , 1 ) = tra;
   }

   Param->cnt = 0;
   Param->is_moving = true;
   Param->ik_solve = true;

   Param->demo_solve = true;

   ROS_INFO("[start] send trajectory");
}

void ManipulationModule::DemoLineTraGeneProc()
{
    ROS_INFO("figure demo : draw line trajectory");

    std::string _demo_path = ros::package::getPath("thormang3_manipulation_module") + "/config/figure_demo.yaml";

    YAML::Node doc;
    try
    {
        // load yaml
        doc = YAML::LoadFile( _demo_path.c_str() );
    }
    catch(const std::exception& e)
    {
        ROS_ERROR("Fail to load yaml file.");
        return ;
    }

    YAML::Node _line_node = doc["demo_line"];

    Param->mov_time = _line_node["mov_time"].as< double >();
    Param->all_time_steps = int( Param->mov_time / Param->smp_time ) + 1;

    Param->calc_task_tra.resize( Param->all_time_steps , 3 );

    Param->via_num = _line_node["via_num"].as< int >();

    std::vector<double> _via_time;
    _via_time = _line_node["via_time"].as< std::vector<double> >();

    Param->via_time.resize( Param->via_num , 1 );

    for( int num = 0; num < Param->via_num; num++ )
        Param->via_time.coeffRef( num , 0 ) = _via_time[ num ];

    Param->task_via_pose.resize( Param->via_num , 3 );

    Param->task_via_dpose.resize( Param->via_num , 3 );
    Param->task_via_dpose.fill( 0.0 );

    Param->task_via_ddpose.resize( Param->via_num , 3 );
    Param->task_via_ddpose.fill( 0.0 );

    if ( Param->goal_demo_pose_msg.name == "left_arm" || Param->goal_demo_pose_msg.name == "left_arm_with_torso" )
    {
        YAML::Node _via_pose_node = _line_node["left_via_position"];
        for(YAML::iterator _it = _via_pose_node.begin() ; _it != _via_pose_node.end() ; ++_it)
        {
            int _num;
            std::vector<double> _value;

            _num = _it->first.as<int>();
            _value = _it->second.as< std::vector<double> >();

            for( int dim = 0; dim < 3; dim++ )
                Param->task_via_pose.coeffRef( _num - 1 , dim ) = _value[ dim ];
        }
    }
    else if ( Param->goal_demo_pose_msg.name == "right_arm" || Param->goal_demo_pose_msg.name == "right_arm_with_torso" )
    {
        YAML::Node _via_pose_node = _line_node["right_via_position"];
        for(YAML::iterator _it = _via_pose_node.begin() ; _it != _via_pose_node.end() ; ++_it)
        {
            int _num;
            std::vector<double> _value;

            _num = _it->first.as<int>();
            _value = _it->second.as< std::vector<double> >();

            for( int dim = 0; dim < 3; dim++ )
                Param->task_via_pose.coeffRef( _num - 1 , dim ) = _value[ dim ];
        }
    }

    std::vector<double> _tar_value;

    if ( Param->goal_demo_pose_msg.name == "left_arm" )
        _tar_value = _line_node["left_tar_position"].as< std::vector<double> >();
    else if ( Param->goal_demo_pose_msg.name == "right_arm" )
        _tar_value = _line_node["right_tar_position"].as< std::vector<double> >();
    else if ( Param->goal_demo_pose_msg.name == "left_arm_with_torso" )
        _tar_value = _line_node["left_tar_position"].as< std::vector<double> >();
    else if ( Param->goal_demo_pose_msg.name == "right_arm_with_torso" )
        _tar_value = _line_node["right_tar_position"].as< std::vector<double> >();

    /* calculate trajectory */
   for ( int dim = 0; dim < 3; dim++ )
   {
       double ini_value = Humanoid->robotis_joint[ Param->ik_id_end ]->position.coeff( dim , 0 );
       double tar_value = _tar_value[ dim ];

       Eigen::MatrixXd via_value = Param->task_via_pose.col( dim );
       Eigen::MatrixXd d_via_value = Param->task_via_dpose.col( dim );
       Eigen::MatrixXd dd_via_value = Param->task_via_ddpose.col( dim );

       Eigen::MatrixXd tra = ROBOTIS_MANIPULATION::minimum_jerk_tra_vian_qdqddq( Param->via_num ,
                                                                                 ini_value , 0.0 , 0.0 ,
                                                                                 via_value , d_via_value , dd_via_value ,
                                                                                 tar_value , 0.0 , 0.0 ,
                                                                                 Param->smp_time , Param->via_time , Param->mov_time );

       Param->calc_task_tra.block( 0 , dim , Param->all_time_steps , 1 ) = tra;
   }

   Param->cnt = 0;
   Param->is_moving = true;
   Param->ik_solve = true;

   Param->demo_solve = true;

   ROS_INFO("[start] send trajectory");
}

void ManipulationModule::DemoCircleTraGeneProc()
{
    ROS_INFO("figure demo : draw circle trajectory");

    std::string _demo_path = ros::package::getPath("thormang3_manipulation_module") + "/config/figure_demo.yaml";

    YAML::Node doc;
    try
    {
        // load yaml
        doc = YAML::LoadFile( _demo_path.c_str() );
    }
    catch(const std::exception& e)
    {
        ROS_ERROR("Fail to load yaml file.");
        return ;
    }

    YAML::Node _circle_node = doc["demo_circle"];

    Param->mov_time = _circle_node["mov_time"].as< double >();
    Param->all_time_steps = int( Param->mov_time / Param->smp_time ) + 1;

    Param->calc_task_tra.resize( Param->all_time_steps , 3 );

    double rotation_angle;
    rotation_angle = _circle_node["rot_angle"].as< double >() * deg2rad;

    if ( Param->goal_demo_pose_msg.name == "right_arm" )
        rotation_angle *= -1.0;
    else if ( Param->goal_demo_pose_msg.name == "right_arm_with_torso" )
        rotation_angle *= -1.0;

    double cross_ratio;
    cross_ratio = _circle_node["cross_ratio"].as< double >();

    std::vector<double> _normal_vector;
    _normal_vector = _circle_node["normal_vector"].as< std::vector<double> >();

    Eigen::MatrixXd normal_vector = Eigen::MatrixXd::Zero( _normal_vector.size() , 1 );
    for( int ix = 0; ix < _normal_vector.size(); ix++ )
        normal_vector.coeffRef( ix , 0 ) = _normal_vector[ ix ];

    Eigen::MatrixXd start_point = Humanoid->robotis_joint[ Param->ik_id_end ]->position;

    std::vector<double> _radius;
    _radius = _circle_node["radius"].as< std::vector<double> >();

    Eigen::MatrixXd center_point = Eigen::MatrixXd::Zero( _radius.size() , 1 );
    for( int ix = 0; ix < _radius.size(); ix++ )
        center_point.coeffRef( ix , 0 ) = start_point.coeffRef( ix , 0 ) + _radius[ ix ];

    // generate trajectory
    Eigen::MatrixXd tra = ROBOTIS_MANIPULATION::arc3d_tra( Param->smp_time, Param->mov_time,
                                                           center_point, normal_vector, start_point,
                                                           rotation_angle, cross_ratio );

    Param->calc_task_tra = tra;

    Param->cnt = 0;
    Param->is_moving = true;
    Param->ik_solve = true;

    Param->demo_solve = true;

    ROS_INFO("[start] send trajectory");
}

void ManipulationModule::BiManualIniTraGeneProc()
{
    ROS_INFO("bi-maual movement : ini_pose");

    std::string _demo_path = ros::package::getPath("thormang3_manipulation_module") + "/config/bimanual_demo.yaml";

    YAML::Node doc;
    try
    {
        // load yaml
        doc = YAML::LoadFile( _demo_path.c_str() );
    }
    catch(const std::exception& e)
    {
        ROS_ERROR("Fail to load yaml file.");
        return ;
    }

    YAML::Node _ini_pose_node = doc["ini_pose"];

    Param->mov_time = _ini_pose_node["mov_time"].as< double >();
    Param->all_time_steps = int( Param->mov_time / Param->smp_time ) + 1;

    Param->calc_l_arm_task_tra.resize( Param->all_time_steps , 3 );
    Param->calc_r_arm_task_tra.resize( Param->all_time_steps , 3 );

    std::vector<double> _l_arm_tar_value;
    _l_arm_tar_value = _ini_pose_node["left_position"].as< std::vector<double> >();

    std::vector<double> _r_arm_tar_value;
    _r_arm_tar_value = _ini_pose_node["right_position"].as< std::vector<double> >();

    /* calculate trajectory */
   for ( int dim = 0; dim < 3; dim++ )
   {
       double l_arm_ini_value = Humanoid->robotis_joint[ id_l_arm_end ]->position.coeff( dim , 0 );
       double l_arm_tar_value = _l_arm_tar_value[ dim ];

       Eigen::MatrixXd l_arm_tra = ROBOTIS_MANIPULATION::minimum_jerk_tra( l_arm_ini_value , 0.0 , 0.0 ,
                                                                           l_arm_tar_value , 0.0 , 0.0 ,
                                                                           Param->smp_time , Param->mov_time );

       Param->calc_l_arm_task_tra.block( 0 , dim , Param->all_time_steps , 1 ) = l_arm_tra;

       double r_arm_ini_value = Humanoid->robotis_joint[ id_r_arm_end ]->position.coeff( dim , 0 );
       double r_arm_tar_value = _r_arm_tar_value[ dim ];

       Eigen::MatrixXd r_arm_tra = ROBOTIS_MANIPULATION::minimum_jerk_tra( r_arm_ini_value , 0.0 , 0.0 ,
                                                                           r_arm_tar_value , 0.0 , 0.0 ,
                                                                           Param->smp_time , Param->mov_time );

       Param->calc_r_arm_task_tra.block( 0 , dim , Param->all_time_steps , 1 ) = r_arm_tra;
   }

   std::vector<double> _l_arm_tar_rpy;
   _l_arm_tar_rpy = _ini_pose_node["left_orientation"].as< std::vector<double> >();
   for ( int ix = 0; ix < _l_arm_tar_rpy.size(); ix++ )
       _l_arm_tar_rpy[ ix ] *= deg2rad;

   std::vector<double> _r_arm_tar_rpy;
   _r_arm_tar_rpy = _ini_pose_node["right_orientation"].as< std::vector<double> >();
   for ( int ix = 0; ix < _r_arm_tar_rpy.size(); ix++ )
       _r_arm_tar_rpy[ ix ] *= deg2rad;

   Param->ik_l_arm_end_rotation = ROBOTIS_MANIPULATION::rpy2rotation( _l_arm_tar_rpy[0] , _l_arm_tar_rpy[1] , _l_arm_tar_rpy[2] );
   Param->ik_r_arm_end_rotation = ROBOTIS_MANIPULATION::rpy2rotation( _r_arm_tar_rpy[0] , _r_arm_tar_rpy[1] , _r_arm_tar_rpy[2] );

   Param->cnt = 0;
   Param->is_moving = true;
   Param->ik_bi_solve = true;
}

void ManipulationModule::BiManualDemoTraGeneProc()
{
    ROS_INFO("bi-maual movement : demo");

    std::string _demo_path = ros::package::getPath("thormang3_manipulation_module") + "/config/bimanual_demo.yaml";

    YAML::Node doc;
    try
    {
        // load yaml
        doc = YAML::LoadFile( _demo_path.c_str() );
    }
    catch(const std::exception& e)
    {
        ROS_ERROR("Fail to load yaml file.");
        return ;
    }

    YAML::Node _bimanual_node = doc["bimanual"];

    Param->mov_time = _bimanual_node["mov_time"].as< double >();
    Param->all_time_steps = int( Param->mov_time / Param->smp_time ) + 1;

    Param->calc_l_arm_task_tra.resize( Param->all_time_steps , 3 );
    Param->calc_r_arm_task_tra.resize( Param->all_time_steps , 3 );

    Param->via_num = _bimanual_node["via_num"].as< int >();

    std::vector<double> _via_time;
    _via_time = _bimanual_node["via_time"].as< std::vector<double> >();

    Param->via_time.resize( Param->via_num , 1 );

    for( int num = 0; num < Param->via_num; num++ )
        Param->via_time.coeffRef( num , 0 ) = _via_time[ num ];

    std::vector<double> _l_arm_tar_value;
    _l_arm_tar_value = _bimanual_node["left_tar_position"].as< std::vector<double> >();

    std::vector<double> _r_arm_tar_value;
    _r_arm_tar_value = _bimanual_node["right_tar_position"].as< std::vector<double> >();

    Param->l_arm_task_via_pose.resize( Param->via_num , 3 );

    Param->l_arm_task_via_dpose.resize( Param->via_num , 3 );
    Param->l_arm_task_via_dpose.fill( 0.0 );

    Param->l_arm_task_via_ddpose.resize( Param->via_num , 3 );
    Param->l_arm_task_via_ddpose.fill( 0.0 );

    YAML::Node _l_arm_via_pose_node = _bimanual_node["left_via_position"];
    for(YAML::iterator _it = _l_arm_via_pose_node.begin() ; _it != _l_arm_via_pose_node.end() ; ++_it)
    {
        int _num;
        std::vector<double> _value;

        _num = _it->first.as<int>();
        _value = _it->second.as< std::vector<double> >();

        for( int dim = 0; dim < 3; dim++ )
            Param->l_arm_task_via_pose.coeffRef( _num - 1 , dim ) = _value[ dim ];
    }

    Param->r_arm_task_via_pose.resize( Param->via_num , 3 );

    Param->r_arm_task_via_dpose.resize( Param->via_num , 3 );
    Param->r_arm_task_via_dpose.fill( 0.0 );

    Param->r_arm_task_via_ddpose.resize( Param->via_num , 3 );
    Param->r_arm_task_via_ddpose.fill( 0.0 );

    YAML::Node _r_arm_via_pose_node = _bimanual_node["right_via_position"];
    for(YAML::iterator _it = _r_arm_via_pose_node.begin() ; _it != _r_arm_via_pose_node.end() ; ++_it)
    {
        int _num;
        std::vector<double> _value;

        _num = _it->first.as<int>();
        _value = _it->second.as< std::vector<double> >();

        for( int dim = 0; dim < 3; dim++ )
            Param->r_arm_task_via_pose.coeffRef( _num - 1 , dim ) = _value[ dim ];
    }

    /* calculate trajectory */
   for ( int dim = 0; dim < 3; dim++ )
   {
       double l_arm_ini_value = Humanoid->robotis_joint[ id_l_arm_end ]->position.coeff( dim , 0 );
       double l_arm_tar_value = _l_arm_tar_value[ dim ];

       Eigen::MatrixXd l_arm_via_value = Param->l_arm_task_via_pose.col( dim );
       Eigen::MatrixXd l_arm_d_via_value = Param->l_arm_task_via_dpose.col( dim );
       Eigen::MatrixXd l_arm_dd_via_value = Param->l_arm_task_via_ddpose.col( dim );

       Eigen::MatrixXd l_arm_tra = ROBOTIS_MANIPULATION::minimum_jerk_tra_vian_qdqddq( Param->via_num ,
                                                                                       l_arm_ini_value , 0.0 , 0.0 ,
                                                                                       l_arm_via_value , l_arm_d_via_value , l_arm_dd_via_value ,
                                                                                       l_arm_tar_value , 0.0 , 0.0 ,
                                                                                       Param->smp_time , Param->via_time , Param->mov_time );

       Param->calc_l_arm_task_tra.block( 0 , dim , Param->all_time_steps , 1 ) = l_arm_tra;

       double r_arm_ini_value = Humanoid->robotis_joint[ id_r_arm_end ]->position.coeff( dim , 0 );
       double r_arm_tar_value = _r_arm_tar_value[ dim ];

       Eigen::MatrixXd r_arm_via_value = Param->r_arm_task_via_pose.col( dim );
       Eigen::MatrixXd r_arm_d_via_value = Param->r_arm_task_via_dpose.col( dim );
       Eigen::MatrixXd r_arm_dd_via_value = Param->r_arm_task_via_ddpose.col( dim );

       Eigen::MatrixXd r_arm_tra = ROBOTIS_MANIPULATION::minimum_jerk_tra_vian_qdqddq( Param->via_num ,
                                                                                       r_arm_ini_value , 0.0 , 0.0 ,
                                                                                       r_arm_via_value , r_arm_d_via_value , r_arm_dd_via_value ,
                                                                                       r_arm_tar_value , 0.0 , 0.0 ,
                                                                                       Param->smp_time , Param->via_time , Param->mov_time );

       Param->calc_r_arm_task_tra.block( 0 , dim , Param->all_time_steps , 1 ) = r_arm_tra;

   }

   std::vector<double> _l_arm_tar_rpy;
   _l_arm_tar_rpy = _bimanual_node["left_tar_orientation"].as< std::vector<double> >();
   for ( int ix = 0; ix < _l_arm_tar_rpy.size(); ix++ )
       _l_arm_tar_rpy[ ix ] *= deg2rad;

   std::vector<double> _r_arm_tar_rpy;
   _r_arm_tar_rpy = _bimanual_node["right_tar_orientation"].as< std::vector<double> >();
   for ( int ix = 0; ix < _r_arm_tar_rpy.size(); ix++ )
       _r_arm_tar_rpy[ ix ] *= deg2rad;

   Param->ik_l_arm_end_rotation = ROBOTIS_MANIPULATION::rpy2rotation( _l_arm_tar_rpy[0] , _l_arm_tar_rpy[1] , _l_arm_tar_rpy[2] );
   Param->ik_r_arm_end_rotation = ROBOTIS_MANIPULATION::rpy2rotation( _r_arm_tar_rpy[0] , _r_arm_tar_rpy[1] , _r_arm_tar_rpy[2] );

   Param->cnt = 0;
   Param->is_moving = true;
   Param->ik_bi_solve = true;
}




void ManipulationModule::Process(std::map<std::string, Dynamixel *> dxls)
{
    if(enable == false)
    {
        Param->is_moving = false;
        Param->ik_solve = false;
        Param->cnt = 0;
		
		Param->demo_solve = false;
		Param->ik_bi_solve = false;
        return;
    }

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

        ROBOTIS_MANIPULATION::JointState::curr_joint_state[ joint_name_to_id[_joint_name] ].position = _joint_curr_position;
        ROBOTIS_MANIPULATION::JointState::goal_joint_state[ joint_name_to_id[_joint_name] ].position = _joint_goal_position;
    }

     /*----- forward kinematics -----*/

    for ( int id = 1; id <= MAX_JOINT_ID; id++ )
        Humanoid->robotis_joint[ id ]->joint_angle = ROBOTIS_MANIPULATION::JointState::goal_joint_state[ id ].position;

    Humanoid->forwardKinematics( 0 );

    /* ----- send trajectory ----- */

    if ( Param->is_moving == true )
    {
        if ( Param->cnt == 0 )
        {
            Param->send_tra_msg.data = "start";
            send_tra_pub_.publish( Param->send_tra_msg );

            Param->ik_start_rotation = Humanoid->robotis_joint[ Param->ik_id_end ]->orientation;

            Param->ik_l_arm_start_rotation = Humanoid->robotis_joint[ id_l_arm_end ]->orientation;
            Param->ik_r_arm_start_rotation = Humanoid->robotis_joint[ id_r_arm_end ]->orientation;
        }

        if ( Param->ik_solve == true )
        {
            /* ----- inverse kinematics ----- */
            if ( Param->demo_solve == true )
                Param->setInverseKinematicsDemo( Param->cnt , Param->ik_start_rotation );
            else
                Param->setInverseKinematics( Param->cnt , Param->ik_start_rotation );

            int max_iter = 30;
            double ik_tol = 1e-3;
            bool ik_success = Humanoid->inverseKinematics( Param->ik_id_start , Param->ik_id_end ,
                                                           Param->ik_target_position , Param->ik_target_rotation ,
                                                           max_iter , ik_tol , Param->ik_weight );

            if ( ik_success == true )
            {
                for ( int id = 1; id <= MAX_JOINT_ID; id++ )
                    ROBOTIS_MANIPULATION::JointState::goal_joint_state[ id ].position = Humanoid->robotis_joint[ id ]->joint_angle;
            }
            else
            {
                ROS_INFO("----- ik failed -----");
                ROS_INFO("[end] send trajectory");

                Param->send_tra_msg.data = "end";
                send_tra_pub_.publish( Param->send_tra_msg );

                Param->is_moving = false;
                Param->ik_solve = false;
                Param->cnt = 0;

                Param->demo_solve = false;
                Param->ik_bi_solve = false;
            }
        }
        else if ( Param->ik_bi_solve == true )
        {
            Param->setInverseKinematicsBiManual( Param->cnt ,
                                                 Param->ik_l_arm_start_rotation , Param->ik_r_arm_start_rotation,
                                                 Param->ik_l_arm_end_rotation , Param->ik_r_arm_end_rotation);

            int max_iter = 30;
            double ik_tol = 1e-3;
            bool l_arm_ik_success = Humanoid->inverseKinematics( id_l_arm_start , id_l_arm_end ,
                                                                 Param->ik_l_arm_target_position , Param->ik_l_arm_target_rotation ,
                                                                 max_iter , ik_tol , Param->ik_weight );

            bool r_arm_ik_success = Humanoid->inverseKinematics( id_r_arm_start , id_r_arm_end ,
                                                                 Param->ik_r_arm_target_position , Param->ik_r_arm_target_rotation ,
                                                                 max_iter , ik_tol , Param->ik_weight );

            if ( (l_arm_ik_success == true) && (r_arm_ik_success == true) )
            {
                for ( int id = 1; id <= MAX_JOINT_ID; id++ )
                    ROBOTIS_MANIPULATION::JointState::goal_joint_state[ id ].position = Humanoid->robotis_joint[ id ]->joint_angle;
            }
            else
            {
                ROS_INFO("----- ik failed -----");
                ROS_INFO("[end] send trajectory");

                Param->send_tra_msg.data = "end";
                send_tra_pub_.publish( Param->send_tra_msg );

                Param->is_moving = false;
                Param->ik_solve = false;
                Param->cnt = 0;

                Param->demo_solve = false;
                Param->ik_bi_solve = false;
            }
        }
        else
        {
            for ( int id = 1; id <= MAX_JOINT_ID; id++ )
                ROBOTIS_MANIPULATION::JointState::goal_joint_state[ id ].position = Param->calc_joint_tra( Param->cnt , id );
        }

        Param->cnt++;
    }

    /*----- set joint data -----*/

    for(std::map<std::string, DynamixelState *>::iterator state_iter = result.begin(); state_iter != result.end(); state_iter++)
    {
        std::string _joint_name = state_iter->first;
        result[_joint_name]->goal_position = ROBOTIS_MANIPULATION::JointState::goal_joint_state[ joint_name_to_id[_joint_name] ].position;
    }

    /*---------- initialize count number ----------*/     

    if ( Param->cnt >= Param->all_time_steps && Param->is_moving == true )
    {
        ROS_INFO("[end] send trajectory");

        Param->send_tra_msg.data = "end";
        send_tra_pub_.publish( Param->send_tra_msg );

        Param->is_moving = false;
        Param->ik_solve = false;
        Param->cnt = 0;

        Param->demo_solve = false;
        Param->ik_bi_solve = false;
    }
}




