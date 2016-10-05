#include <thormang3_diagnostic_module/diagnostic_sensor_module.h>



using namespace thormang3;

DiagnosticSensor::DiagnosticSensor()
  : control_cycle_msec_(8)
  , gazebo_robot_name_("robotis")
  , gazebo_mode_(false)
{
  module_name_ = "thormang3_diagnostic_sensor_module"; // set unique module name
}

DiagnosticSensor::~DiagnosticSensor()
{
  queue_thread_.join();
}

void DiagnosticSensor::initialize(const int control_cycle_msec, robotis_framework::Robot* robot)
{
  queue_thread_ = boost::thread(boost::bind(&DiagnosticSensor::msgQueueThread, this));
}

void DiagnosticSensor::msgQueueThread()
{
  ros::NodeHandle ros_node;
  ros::CallbackQueue callback_queue;

  ros_node.setCallbackQueue(&callback_queue);

  /* subscriber */
  //ros::Subscriber imu_raw_sub = ros_node.subscribe<sensor_msgs::Imu>(gazebo_mode_ ? "/gazebo/" + gazebo_robot_name_ + "/sensor/imu" : "sensor/imu/raw",	1, boost::bind(&DiagnosticSensor::DiagnosticSensorCallback, this, _1, std::string("raw")));

  /* publisher */
  //imu_status_pub_	= ros_node.advertise<robotis_controller_msgs::StatusMsg>("robotis/status", 1);

  ros::WallDuration duration(control_cycle_msec_/1000.0);
  while(ros_node.ok())
    callback_queue.callAvailable(duration);
}

void DiagnosticSensor::process(std::map<std::string, robotis_framework::Dynamixel*> /*dxls*/, std::map<std::string, robotis_framework::Sensor*> /*sensors*/)
{
  // TODO
}
