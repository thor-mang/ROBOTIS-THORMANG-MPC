/*
 * store values in table form in a file so it can be ploted
 */


#include "my_visualization_for_walking/data_storage_tool.h"

#include "ros/ros.h"


DataStorageTool::DataStorageTool(std::string filepath, DataType TYPE)
{
  _filepath = filepath + ".dat";
  _type = TYPE;
  _withEstimatedZMPCOBFeet = false;

  switch(_type)
  {
  case JOINT:
    _data_vectors.resize(36);
    break;
  case COB_FEET:
    _data_vectors.resize(43);
    break;
  case FORCE_TORQUE:
    _data_vectors.resize(24);
    break;
  }

}

DataStorageTool::~ DataStorageTool()
{ }


void DataStorageTool::storeJointValues(const thormang3_walking_module_msgs::WalkingJointStatesStamped::ConstPtr& msg)
{
  _time_vector.push_back(msg->header.stamp);

  //right
  _data_vectors[0].push_back(msg->r_goal_hip_y);
  _data_vectors[1].push_back(msg->r_mdfd_goal_hip_y);
  _data_vectors[2].push_back(msg->r_present_hip_y);

  _data_vectors[3].push_back(msg->r_goal_hip_r);
  _data_vectors[4].push_back(msg->r_mdfd_goal_hip_r);
  _data_vectors[5].push_back(msg->r_present_hip_r);

  _data_vectors[6].push_back(msg->r_goal_hip_p);
  _data_vectors[7].push_back(msg->r_mdfd_goal_hip_p);
  _data_vectors[8].push_back(msg->r_present_hip_p);

  _data_vectors[9].push_back(msg->r_goal_kn_p);
  _data_vectors[10].push_back(msg->r_mdfd_goal_kn_p);
  _data_vectors[11].push_back(msg->r_present_kn_p);

  _data_vectors[12].push_back(msg->r_goal_an_p);
  _data_vectors[13].push_back(msg->r_mdfd_goal_an_p);
  _data_vectors[14].push_back(msg->r_present_an_p);

  _data_vectors[15].push_back(msg->r_goal_an_r);
  _data_vectors[16].push_back(msg->r_mdfd_goal_an_r);
  _data_vectors[17].push_back(msg->r_present_an_r);

  //left
  _data_vectors[18].push_back(msg->l_goal_hip_y);
  _data_vectors[19].push_back(msg->l_mdfd_goal_hip_y);
  _data_vectors[20].push_back(msg->l_present_hip_y);

  _data_vectors[21].push_back(msg->l_goal_hip_r);
  _data_vectors[22].push_back(msg->l_mdfd_goal_hip_r);
  _data_vectors[23].push_back(msg->l_present_hip_r);

  _data_vectors[24].push_back(msg->l_goal_hip_p);
  _data_vectors[25].push_back(msg->l_mdfd_goal_hip_p);
  _data_vectors[26].push_back(msg->l_present_hip_p);

  _data_vectors[27].push_back(msg->l_goal_kn_p);
  _data_vectors[28].push_back(msg->l_mdfd_goal_kn_p);
  _data_vectors[29].push_back(msg->l_present_kn_p);

  _data_vectors[30].push_back(msg->l_goal_an_p);
  _data_vectors[31].push_back(msg->l_mdfd_goal_an_p);
  _data_vectors[32].push_back(msg->l_present_an_p);

  _data_vectors[33].push_back(msg->l_goal_an_r);
  _data_vectors[34].push_back(msg->l_mdfd_goal_an_r);
  _data_vectors[35].push_back(msg->l_present_an_r);
}


void DataStorageTool::storeCOBFeetValues(const thormang3_walking_module_msgs::RobotPose::ConstPtr& msg)
{
  _time_vector.push_back(ros::Time::now());

  //ref ZMP
  _data_vectors[0].push_back(msg->ref_zmp.x);
  _data_vectors[1].push_back(msg->ref_zmp.y);
  _data_vectors[2].push_back(msg->ref_zmp.z);

  //desired 3D positions:
  //body
  _data_vectors[3].push_back(msg->global_to_center_of_body.position.x);
  _data_vectors[4].push_back(msg->global_to_center_of_body.position.y);
  _data_vectors[5].push_back(msg->global_to_center_of_body.position.z);
  _data_vectors[6].push_back(msg->global_to_center_of_body.orientation.x);
  _data_vectors[7].push_back(msg->global_to_center_of_body.orientation.y);
  _data_vectors[8].push_back(msg->global_to_center_of_body.orientation.z);

  //right foot
  _data_vectors[9].push_back(msg->global_to_right_foot.position.x);
  _data_vectors[10].push_back(msg->global_to_right_foot.position.y);
  _data_vectors[11].push_back(msg->global_to_right_foot.position.z);
  _data_vectors[12].push_back(msg->global_to_right_foot.orientation.x);
  _data_vectors[13].push_back(msg->global_to_right_foot.orientation.y);
  _data_vectors[14].push_back(msg->global_to_right_foot.orientation.z);

  //left foot
  _data_vectors[15].push_back(msg->global_to_left_foot.position.x);
  _data_vectors[16].push_back(msg->global_to_left_foot.position.y);
  _data_vectors[17].push_back(msg->global_to_left_foot.position.z);
  _data_vectors[18].push_back(msg->global_to_left_foot.orientation.x);
  _data_vectors[19].push_back(msg->global_to_left_foot.orientation.y);
  _data_vectors[20].push_back(msg->global_to_left_foot.orientation.z);

  //desired 3D positions modified by Balance Control:
  //body
  _data_vectors[21].push_back(msg->mdfd_global_to_center_of_body.position.x);
  _data_vectors[22].push_back(msg->mdfd_global_to_center_of_body.position.y);
  _data_vectors[23].push_back(msg->mdfd_global_to_center_of_body.position.z);
  _data_vectors[24].push_back(msg->mdfd_global_to_center_of_body.orientation.x);
  _data_vectors[25].push_back(msg->mdfd_global_to_center_of_body.orientation.y);
  _data_vectors[26].push_back(msg->mdfd_global_to_center_of_body.orientation.z);

  //right foot
  _data_vectors[27].push_back(msg->mdfd_global_to_right_foot.position.x);
  _data_vectors[28].push_back(msg->mdfd_global_to_right_foot.position.y);
  _data_vectors[29].push_back(msg->mdfd_global_to_right_foot.position.z);
  _data_vectors[30].push_back(msg->mdfd_global_to_right_foot.orientation.x);
  _data_vectors[31].push_back(msg->mdfd_global_to_right_foot.orientation.y);
  _data_vectors[32].push_back(msg->mdfd_global_to_right_foot.orientation.z);

  //left foot
  _data_vectors[33].push_back(msg->mdfd_global_to_left_foot.position.x);
  _data_vectors[34].push_back(msg->mdfd_global_to_left_foot.position.y);
  _data_vectors[35].push_back(msg->mdfd_global_to_left_foot.position.z);
  _data_vectors[36].push_back(msg->mdfd_global_to_left_foot.orientation.x);
  _data_vectors[37].push_back(msg->mdfd_global_to_left_foot.orientation.y);
  _data_vectors[38].push_back(msg->mdfd_global_to_left_foot.orientation.z);

  //desired cob acceleration
  _data_vectors[39].push_back(msg->global_to_cob_acc.x);
  _data_vectors[40].push_back(msg->global_to_cob_acc.y);
  _data_vectors[41].push_back(msg->robot_to_cob_acc.x);
  _data_vectors[42].push_back(msg->robot_to_cob_acc.y);
}


void DataStorageTool::storeForceTorqueValues(const thormang3_walking_module_msgs::ForceTorqueStates::ConstPtr& msg)
{
  _time_vector.push_back(msg->header.stamp);

  //right
  _data_vectors[0].push_back(msg->des_fx_r);
  _data_vectors[1].push_back(msg->meas_fx_r);
  _data_vectors[2].push_back(msg->des_fy_r);
  _data_vectors[3].push_back(msg->meas_fy_r);
  _data_vectors[4].push_back(msg->des_fz_r);
  _data_vectors[5].push_back(msg->meas_fz_r);

  _data_vectors[6].push_back(msg->des_tx_r);
  _data_vectors[7].push_back(msg->meas_tx_r);
  _data_vectors[8].push_back(msg->des_ty_r);
  _data_vectors[9].push_back(msg->meas_ty_r);
  _data_vectors[10].push_back(msg->des_tz_r);
  _data_vectors[11].push_back(msg->meas_tz_r);

  _data_vectors[12].push_back(msg->des_fx_l);
  _data_vectors[13].push_back(msg->meas_fx_l);
  _data_vectors[14].push_back(msg->des_fy_l);
  _data_vectors[15].push_back(msg->meas_fy_l);
  _data_vectors[16].push_back(msg->des_fz_l);
  _data_vectors[17].push_back(msg->meas_fz_l);

  _data_vectors[18].push_back(msg->des_tx_l);
  _data_vectors[19].push_back(msg->meas_tx_l);
  _data_vectors[20].push_back(msg->des_ty_l);
  _data_vectors[21].push_back(msg->meas_ty_l);
  _data_vectors[22].push_back(msg->des_tz_r);
  _data_vectors[23].push_back(msg->meas_tz_l);
}


void DataStorageTool::computeEstimatedZMPCOBFeet(const std::vector< std::vector<float> > joint_data_vectors, const std::vector< std::vector<float> > ft_data_vectors)
{
  _withEstimatedZMPCOBFeet = true;
  _computed_vectors.resize(21);

  Status currentStatus = STANDING;

  float current_des_zmp_x = _data_vectors[0][0];
  float current_des_zmp_y = _data_vectors[1][0];
  float previous_des_zmp_x;
  float previous_des_zmp_y;


  for (unsigned int k = 0; k < _time_vector.size(); k++)
  {
    previous_des_zmp_x = current_des_zmp_x;
    previous_des_zmp_y = current_des_zmp_y;
    current_des_zmp_x = _data_vectors[0][k];
    current_des_zmp_y = _data_vectors[1][k];

    //----UPDATE STATUS----
    //TODO should be subscribed...
    if ((current_des_zmp_x != previous_des_zmp_x) || (current_des_zmp_y != previous_des_zmp_y)) //if desired px or py is different from previous desired px/py (=> beginning new step)
    {
      switch (currentStatus)
      {
      case STANDING:
        if (( current_des_zmp_y - (_data_vectors[10][k] + _data_vectors[16][k])/2 ) <0)
          currentStatus = LEFT_FOOT_SWING;
        else
          currentStatus = RIGHT_FOOT_SWING;
        break;
      case LEFT_FOOT_SWING:
        currentStatus = RIGHT_FOOT_SWING;
        break;
      case RIGHT_FOOT_SWING:
        currentStatus = LEFT_FOOT_SWING;
        break;
      }

      if( (fabs(current_des_zmp_x - (_data_vectors[9][k] + _data_vectors[15][k])/2) < 0.00001)
       && (fabs(current_des_zmp_y - (_data_vectors[10][k] + _data_vectors[16][k])/2) < 0.00001)) //last step i.e. des_zmp in the middle of the feet
      {
        currentStatus = STANDING;
      }
    }

    //----PUSHBACK DATA----
    switch(currentStatus)
    {
    case RIGHT_FOOT_SWING:
      //TODO real test
      //ZMP
      _computed_vectors[0].push_back(current_des_zmp_x - ft_data_vectors[21][k]/ft_data_vectors[17][k]); //px = -ty_l/Fz_l
      _computed_vectors[1].push_back(current_des_zmp_y + ft_data_vectors[19][k]/ft_data_vectors[17][k]); //py = tx_l/Fz_l
      _computed_vectors[2].push_back(-0.630); //pz depends on the terrain

      //TODO FK from left foot
      //body
      _computed_vectors[3].push_back(0);
      _computed_vectors[4].push_back(0);
      _computed_vectors[5].push_back(0);
      _computed_vectors[6].push_back(0);
      _computed_vectors[7].push_back(0);
      _computed_vectors[8].push_back(0);

      //TODO FK from cob
      //right foot
      _computed_vectors[9].push_back(0);
      _computed_vectors[10].push_back(0);
      _computed_vectors[11].push_back(0);
      _computed_vectors[12].push_back(0);
      _computed_vectors[13].push_back(0);
      _computed_vectors[14].push_back(0);

      //TODO last FK from cob from previous step
      //left foot
      _computed_vectors[15].push_back(0);
      _computed_vectors[16].push_back(0);
      _computed_vectors[17].push_back(0);
      _computed_vectors[18].push_back(0);
      _computed_vectors[19].push_back(0);
      _computed_vectors[20].push_back(0);
      break;
    case LEFT_FOOT_SWING:
      //TODO real test
      //ZMP
      _computed_vectors[0].push_back(current_des_zmp_x - ft_data_vectors[9][k]/ft_data_vectors[5][k]); //px = -ty_r/Fz_r
      _computed_vectors[1].push_back(current_des_zmp_y + ft_data_vectors[7][k]/ft_data_vectors[5][k]); //py = tx_r/Fz_r
      _computed_vectors[2].push_back(-0.630); //pz depends on the terrain

      //TODO FK from right foot
      //body
      _computed_vectors[3].push_back(1.0);
      _computed_vectors[4].push_back(1.0);
      _computed_vectors[5].push_back(1.0);
      _computed_vectors[6].push_back(1.0);
      _computed_vectors[7].push_back(1.0);
      _computed_vectors[8].push_back(1.0);

      //right foot
      _computed_vectors[9].push_back(1);
      _computed_vectors[10].push_back(1);
      _computed_vectors[11].push_back(1);
      _computed_vectors[12].push_back(1);
      _computed_vectors[13].push_back(1);
      _computed_vectors[14].push_back(1);

      //left foot
      _computed_vectors[15].push_back(1);
      _computed_vectors[16].push_back(1);
      _computed_vectors[17].push_back(1);
      _computed_vectors[18].push_back(1);
      _computed_vectors[19].push_back(1);
      _computed_vectors[20].push_back(1);
      break;
    case STANDING:
      //TODO real test
      //ZMP
      _computed_vectors[0].push_back(current_des_zmp_x - ft_data_vectors[21][k]/ft_data_vectors[17][k]/2 - ft_data_vectors[9][k]/ft_data_vectors[5][k]/2); //px = -(ty_l/Fz_l + ty_r/Fz_r)/2
      _computed_vectors[1].push_back(current_des_zmp_y + ft_data_vectors[19][k]/ft_data_vectors[17][k]/2 + ft_data_vectors[7][k]/ft_data_vectors[5][k]/2); //py = (tx_l/Fz_l + tx_r/Fz_r)/2
      _computed_vectors[2].push_back(-0.630); //pz depends on the terrain

      //TODO FK = mean of both legs FK
      //body
      _computed_vectors[3].push_back(5);
      _computed_vectors[4].push_back(5);
      _computed_vectors[5].push_back(5);
      _computed_vectors[6].push_back(5);
      _computed_vectors[7].push_back(5);
      _computed_vectors[8].push_back(5);

      //right foot
      _computed_vectors[9].push_back(5);
      _computed_vectors[10].push_back(5);
      _computed_vectors[11].push_back(5);
      _computed_vectors[12].push_back(5);
      _computed_vectors[13].push_back(5);
      _computed_vectors[14].push_back(5);

      //left foot
      _computed_vectors[15].push_back(5);
      _computed_vectors[16].push_back(5);
      _computed_vectors[17].push_back(5);
      _computed_vectors[18].push_back(5);
      _computed_vectors[19].push_back(5);
      _computed_vectors[20].push_back(5);
      break;
    }
  }

}


void DataStorageTool::writeValues()
{
  //----Open file-----
  std::ofstream file;
  char *filepath_char = new char[_filepath.length() + 1];
  std::strcpy(filepath_char, _filepath.c_str());
  file.open(filepath_char);
  delete[] filepath_char;


  //----Write column names into file-----
  file << "time" << " ";
  switch(_type)
  {
  case JOINT:
    //FIRST LINE
    file << "r_hip_y" << " " << " " << " ";
    file << "r_hip_r" << " " << " " << " ";
    file << "r_hip_p" << " " << " " << " ";
    file << "r_kn_p" << " " << " " << " ";
    file << "r_an_p" << " " << " " << " ";
    file << "r_an_r" << " " << " " << " ";

    file << "l_hip_y" << " " << " " << " ";
    file << "l_hip_r" << " " << " " << " ";
    file << "l_hip_p" << " " << " " << " ";
    file << "l_kn_p" << " " << " " << " ";
    file << "l_an_p" << " " << " " << " ";
    file << "l_an_r" << " " << " " << " ";

    //SECOND LINE
    file << "\n" << " ";
    for (unsigned int k = 0; k < 12; k++)
    {
        file << "goal" << " " << "mdfd_goal" << " " << "present" << " ";
    }
    break;
  case COB_FEET:
    //FIRST LINE
    file << "des_zmp" << " " << " " << " ";

    file << "des_g_to_cob" << " " << " " << " " << " " << " " << " ";
    file << "des_g_to_rf" << " " << " " << " " << " " << " " << " ";
    file << "des_g_to_lf" << " " << " " << " " << " " << " " << " ";

    file << "mdfd_des_g_to_cob" << " " << " " << " " << " " << " " << " ";
    file << "mdfd_des_g_to_rf" << " " << " " << " " << " " << " " << " ";
    file << "mdfd_des_g_to_lf" << " " << " " << " " << " " << " " << " ";

    file << "des_g_to_cob_acc" << " " << " ";
    file << "des_robot_to_cob_acc" << " " << " ";

    if(_withEstimatedZMPCOBFeet)
    {
      file << "est_zmp" << " " << " " << " ";

      file << "est_g_to_cob" << " " << " " << " " << " " << " " << " ";
      file << "est_g_to_rf" << " " << " " << " " << " " << " " << " ";
      file << "est_g_to_lf" << " " << " " << " " << " " << " " << " ";
    }

    //SECOND LINE
    file << "\n" << " ";
    file << "x y z ";
    for (unsigned int k = 0; k < 6; k++)
    {
        file << "x y z roll pitch yaw ";
    }
    file << "x y ";
    file << "x y ";

    if(_withEstimatedZMPCOBFeet)
    {
      file << "x y z ";
      for (unsigned int k = 0; k < 3; k++)
      {
          file << "x y z roll pitch yaw ";
      }
    }
    break;
  case FORCE_TORQUE:
    //FIRST LINE
    file << "fx_r" << " " << " ";
    file << "fy_r" << " " << " ";
    file << "fz_r" << " " << " ";
    file << "tx_r" << " " << " ";
    file << "ty_r" << " " << " ";
    file << "tz_r" << " " << " ";

    file << "fx_l" << " " << " ";
    file << "fy_l" << " " << " ";
    file << "fz_l" << " " << " ";
    file << "tx_l" << " " << " ";
    file << "ty_l" << " " << " ";
    file << "tz_l" << " " << " ";

    //SECOND LINE
    file << "\n" << " ";
    for (unsigned int k = 0; k < 12; k++)
    {
        file << "des" << " " << "meas" << " ";
    }
    break;
  }
  file << "\n";


  //-----Write data into file-----
  for (unsigned int j = 0; j < _data_vectors[0].size(); j++)
  {
    file << _time_vector[j] << " ";

    for (unsigned int i = 0; i < _data_vectors.size(); i++)
    {
      file << _data_vectors[i][j] << " ";
    }

    if(_withEstimatedZMPCOBFeet)
    {
      for (unsigned int i = 0; i < _computed_vectors.size(); i++)
      {
        file << _computed_vectors[i][j] << " ";
      }
    }

    file << "\n";
  }

  //----Close file-----
  file.close();
}


std::vector< std::vector<float> > DataStorageTool::getDataVectors()
{
  return _data_vectors;
}

