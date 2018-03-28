
#ifndef data_storage_tool_h
#define data_storage_tool_h

#include <iostream>
#include <fstream>
#include <vector>
#include <string>

#include "thormang3_walking_module_msgs/WalkingJointStatesStamped.h"
#include "thormang3_walking_module_msgs/RobotPose.h"
#include "thormang3_walking_module_msgs/ForceTorqueStates.h"


class DataStorageTool
{

  public:
    enum DataType {JOINT, COB_FEET, FORCE_TORQUE};

    DataStorageTool(std::string filepath, DataType TYPE);
    ~ DataStorageTool();
    void storeJointValues(const thormang3_walking_module_msgs::WalkingJointStatesStamped::ConstPtr& msg);
    void storeCOBFeetValues(const thormang3_walking_module_msgs::RobotPose::ConstPtr& msg);
    void storeForceTorqueValues(const thormang3_walking_module_msgs::ForceTorqueStates::ConstPtr& msg);
    void computeEstimatedZMPCOBFeet(const std::vector< std::vector<float> > joint_data_vectors, const std::vector< std::vector<float> > ft_data_vectors);
    void writeValues();

    std::vector< std::vector<float> > getDataVectors();

  private:
    std::string _filepath;
    std::ofstream _file;
    DataType _type;

    enum Status {STANDING, LEFT_FOOT_SWING, RIGHT_FOOT_SWING};

    bool _withEstimatedZMPCOBFeet;

    std::vector<ros::Time> _time_vector;
    std::vector< std::vector<float> > _data_vectors;
    std::vector< std::vector<float> > _computed_vectors;

};

#endif
