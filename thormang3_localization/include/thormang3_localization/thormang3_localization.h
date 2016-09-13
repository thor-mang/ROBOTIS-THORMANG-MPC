#ifndef THORMANG3_LOCALIZATION_H_
#define THORMANG3_LOCALIZATION_H_

//std
#include <string>

//ros dependencies
#include <ros/ros.h>

// ros msg, srv
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>

// eigen
#include <Eigen/Dense>

// boost
#include <boost/thread.hpp>

namespace thormang3
{

class Thormang3Localization
{

private:
    //ros node handle
    ros::NodeHandle ros_node_;

    //subscriber
    ros::Subscriber thormang3_pelvis_msg_sub_;

    tf::TransformBroadcaster broadcaster_;

    tf::StampedTransform pelvis_trans_;


//    geometry_msgs::TransformStamped pelvis_trans_;
    geometry_msgs::PoseStamped thormang3_pelvis_pose_;
    double transform_tolerance_;

public:
    void initialize();
    void thormang3PelvisPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

    //constructor
    Thormang3Localization();
    //destructor
    ~Thormang3Localization();

    void process();

};

}       // namespace

#endif  // THORMANG3_LOCALIZATION_H_
