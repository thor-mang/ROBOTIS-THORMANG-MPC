/*
 * store automatically joint, COB, feet values (published by thormang3_walking_module) in table form in several text files.
 *
 * These files can be opened with excel, scilab or a similar software so these values can be easily ploted
 *
 * How to use:
 * 1A) If you want to use the default joint fb gain values:
 *      - launch the simulation (>> thor sim)
 *      - run the program (>> rosrun my_visualization_for_walking data_storage_tool_node)
 *      - start the walking mode through the demo GUI (after clicking on init pose) (>> thor ui demo)
 *      - start the walking
 * 1B) If you want to set your own gain values:
 *      - launch the simulation
 *      - start the walking mode through the demo GUI
 *      - run the program
 *      - set the gains using the corresponding ROS service, for instance using rqt or the following command line:
 *        >> rosservice call /johnny5/robotis/walking/joint_feedback_gain "{updating_duration: 0.0 , feedback_gain: [1.0 , 1.5 , 0.15 , 0.15 , 0.05 , 0.05 , 1.0 , 1.5 , 0.15 , 0.15 , 0.05 , 0.05 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0]}"
 *                                                                                                                   r_..._p_gain                            l_..._p_gain                           r_..._d_gain                         l_..._d_gain
 *       - start the walking
 * 2) After the walking, the program will stop automatically.
 *    You can relaunch it without shutting down the simulation
 *    but you will need to set the gains again, using either the ROS service
 *    or Robotis Demo GUI (Walking -> Balance Control -> Apply).
 */


#include "ros/ros.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>


#include "my_visualization_for_walking/data_storage_tool.h"
#include "robotis_controller_msgs/StatusMsg.h"
#include "thormang3_walking_module_msgs/JointFeedBackGain.h"
#include "thormang3_foot_step_generator/FootStepCommand.h"


std::string walking_status= "";
std::string walking_direction;
int steps_number;
float front_step_length;
float side_step_length;
float rotation_angle;
thormang3_walking_module_msgs::JointFeedBackGain joint_fb_gain_msg;
bool got_msg_joint_feedback_gain;
bool got_msg_walking_infos;

std::string currentDateTime()
{
    time_t now = time(0);
    struct tm tstruct;
    char buf[80];
    tstruct = *localtime(&now);
    strftime(buf, sizeof(buf), "%Y%m%d_%H%M", &tstruct);

    return buf;
}

void getWalkingStatus(const robotis_controller_msgs::StatusMsg::ConstPtr& msg)
{
  walking_status = msg->status_msg;
}

void getJointFBGainMsg(const thormang3_walking_module_msgs::JointFeedBackGain::ConstPtr& msg)
{
 joint_fb_gain_msg = *msg;

 got_msg_joint_feedback_gain = true;
}

void getWalkingInfos(const thormang3_foot_step_generator::FootStepCommand::ConstPtr& msg)
{
  walking_direction = msg->command;
  steps_number = msg->step_num;
  front_step_length = msg->step_length;
  side_step_length = msg->side_step_length;
  rotation_angle = msg->step_angle_rad;

  got_msg_walking_infos = true;
}

void writeInfos(std::string filepath, std::string date_time)
{
  //----Open file-----
  std::ofstream file;
  char *filepath_char = new char[filepath.length() + 1];
  std::strcpy(filepath_char, filepath.c_str());
  file.open(filepath_char);
  delete[] filepath_char;

  //-----Write data into file-----
  file << "RECORDING DATE AND TIME:" << " " << date_time << "\n";
  file << "\n";

  file << "WALKING INFOS:" << "\n";
  file << "Walking direction" << " " << walking_direction << "\n";
  file << "Number of steps" << " " << steps_number << "\n";
  file << "Front step length" << " " << front_step_length << "\n";
  file << "Side step length" << " " << side_step_length << "\n";
  file << "Rotation angle" << " " << rotation_angle << "\n";
  file << "\n";

  file << "JOINT FB GAINS:" << " " << "P GAIN" << " " << "D GAIN" << "\n";

  file << "r_leg_hip_y" << " " << joint_fb_gain_msg.r_leg_hip_y_p_gain << " " << joint_fb_gain_msg.r_leg_hip_y_d_gain << "\n";
  file << "r_leg_hip_r" << " " << joint_fb_gain_msg.r_leg_hip_r_p_gain << " " << joint_fb_gain_msg.r_leg_hip_r_d_gain << "\n";
  file << "r_leg_hip_p" << " " << joint_fb_gain_msg.r_leg_hip_p_p_gain << " " << joint_fb_gain_msg.r_leg_hip_p_d_gain << "\n";
  file << "r_leg_kn_p" << " " << joint_fb_gain_msg.r_leg_kn_p_p_gain << " " << joint_fb_gain_msg.r_leg_kn_p_d_gain << "\n";
  file << "r_leg_an_p" << " " << joint_fb_gain_msg.r_leg_an_p_p_gain <<  " " << joint_fb_gain_msg.r_leg_an_p_d_gain << "\n";
  file << "r_leg_an_r" << " " << joint_fb_gain_msg.r_leg_an_r_p_gain << " " << joint_fb_gain_msg.r_leg_an_r_d_gain << "\n";

  file << "l_leg_hip_y" << " " << joint_fb_gain_msg.l_leg_hip_y_p_gain << " " << joint_fb_gain_msg.l_leg_hip_y_d_gain << "\n";
  file << "l_leg_hip_r" << " " << joint_fb_gain_msg.l_leg_hip_r_p_gain << " " << joint_fb_gain_msg.l_leg_hip_r_d_gain << "\n";
  file << "l_leg_hip_p" << " " << joint_fb_gain_msg.l_leg_hip_p_p_gain << " " << joint_fb_gain_msg.l_leg_hip_p_d_gain << "\n";
  file << "l_leg_kn_p" << " " << joint_fb_gain_msg.l_leg_kn_p_p_gain << " " << joint_fb_gain_msg.l_leg_kn_p_d_gain << "\n";
  file << "l_leg_an_p" << " " << joint_fb_gain_msg.l_leg_an_p_p_gain <<  " " << joint_fb_gain_msg.l_leg_an_p_d_gain << "\n";
  file << "l_leg_an_r" << " " << joint_fb_gain_msg.l_leg_an_r_p_gain << " " << joint_fb_gain_msg.l_leg_an_r_d_gain << "\n";

  //----Close file-----
  file.close();
}



int main(int argc, char **argv)
{
  //----->>> TO SET <<<<-----
  bool withJointSub = true;
  bool withCOBFeetSub = true;
  bool withFTSub = true;

  //-----INITIALIZATION-----
  std::string current_date_and_time = currentDateTime();
  std::string common_path = "./Documents/DataStorageTool_files/";

  std::string common_infos_filename = current_date_and_time + "_0_common_infos";
  std::string joint_filename = current_date_and_time + "_joint_values";
  std::string cob_feet_filename = current_date_and_time + "_cob_feet_values";
  std::string force_torque_filename = current_date_and_time + "_force_torque_values";

  DataStorageTool joint_data_storage_tool(common_path +joint_filename, DataStorageTool::JOINT);
  DataStorageTool cob_feet_data_storage_tool(common_path + cob_feet_filename, DataStorageTool::COB_FEET);
  DataStorageTool force_torque_data_storage_tool(common_path + force_torque_filename, DataStorageTool::FORCE_TORQUE);

  ros::init(argc, argv, "data_storage_tool_node");
  ros::NodeHandle nh;

  ROS_INFO_STREAM("#### Created following files: ####");
  ROS_INFO_STREAM("Path: " << common_path);
  ROS_INFO_STREAM("Filenames: ");
  ROS_INFO_STREAM("- " << common_infos_filename);
  if(withJointSub)
  {ROS_INFO_STREAM("- " << joint_filename);}
  if(withCOBFeetSub)
  {ROS_INFO_STREAM("- " << cob_feet_filename);}
  if (withFTSub)
  {ROS_INFO_STREAM("- " << force_torque_filename);}
  ROS_INFO_STREAM("##################################");


  //-----STORE COMMON WALKING INFOS AND GAINS-----
  ROS_INFO_STREAM("!!! PLEASE UPDATE JOINT FEEDBACK GAINS !!!");
  ros::Subscriber joint_feedback_gain_subscriber = nh.subscribe("/johnny5/robotis/walking/joint_fb_gain",1000,&getJointFBGainMsg);
  while (!got_msg_joint_feedback_gain && ros::ok())
  {
    ros::spinOnce();
  }

  ROS_INFO_STREAM("Waiting for walking command...");
  ros::Subscriber walking_infos_subscriber = nh.subscribe("/johnny5/robotis/thormang3_foot_step_generator/walking_command",1000,&getWalkingInfos);
  ros::Subscriber walking_status_subscriber = nh.subscribe("/johnny5/robotis/status",1000,&getWalkingStatus);
  while (walking_status != "Walking_Started" && ros::ok())
  {
    ros::spinOnce();
  }


  //-----STORE DATA FROM WALKING START TO WALKING STOP-----
  ROS_INFO_STREAM("Storing data...");
  ros::Subscriber joint_sub;
  ros::Subscriber cob_feet_sub;
  ros::Subscriber force_torque_sub;
  if (withJointSub)
  {joint_sub = nh.subscribe("/johnny5/robotis/walking/walking_joint_states", 1000, &DataStorageTool::storeJointValues, &joint_data_storage_tool);}
  if (withCOBFeetSub)
  {cob_feet_sub = nh.subscribe("/johnny5/robotis/walking/robot_pose", 1000, &DataStorageTool::storeCOBFeetValues, &cob_feet_data_storage_tool);}
  if (withFTSub)
  {force_torque_sub = nh.subscribe("/johnny5/robotis/walking/force_torque_states", 1000, &DataStorageTool::storeForceTorqueValues, &force_torque_data_storage_tool);}

  while(walking_status != "Walking_Finished" && ros::ok())
  {
    ros::spinOnce();
  }

  //-----COMPUTE ADDITIONAL DATA-----
  if(withCOBFeetSub && withJointSub && withFTSub)
  {
    ROS_INFO_STREAM("Computing additional data...");
    cob_feet_data_storage_tool.computeEstimatedZMPCOBFeet(joint_data_storage_tool.getDataVectors(), force_torque_data_storage_tool.getDataVectors());
  }


  //-----WRITE DATA IN FILES-----
  ROS_INFO_STREAM("Writing data...");
  writeInfos(common_path + common_infos_filename + ".dat", current_date_and_time);
  if (withJointSub)
  {joint_data_storage_tool.writeValues();}
  if (withCOBFeetSub)
  {cob_feet_data_storage_tool.writeValues();}
  if(withFTSub)
  {force_torque_data_storage_tool.writeValues();}

  ROS_INFO_STREAM("Job done!");


  return 0;
}


