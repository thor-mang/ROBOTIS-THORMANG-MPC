
// include
#include "thormang3_localization/thormang3_localization.h"

//node main
int main(int argc, char **argv)
{
  //init ros
  ros::init(argc, argv, "thormang3_localization_node");

  //create ros wrapper object
  thormang3::Thormang3Localization thormang3_localization;

  //set node loop rate
  ros::Rate loop_rate(20);

  //node loop
  while ( ros::ok() )
  {
    thormang3_localization.process();

    //execute pending callbacks
    ros::spinOnce();

    //relax to fit output rate
    loop_rate.sleep();
  }

  //exit program
  return 0;
}

