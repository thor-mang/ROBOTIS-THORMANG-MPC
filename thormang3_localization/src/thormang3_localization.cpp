//#include <yaml-cpp/yaml.h>

#include "thormang3_localization/thormang3_localization.h"

namespace thormang3
{

Thormang3Localization::Thormang3Localization()
 : ros_node_(),
   transform_tolerance_(0.01)
{
  initialize();

  thormang3_pelvis_pose_.pose.position.x = 0.0;
  thormang3_pelvis_pose_.pose.position.y = 0.0;
  thormang3_pelvis_pose_.pose.position.z = 0.723;
  thormang3_pelvis_pose_.pose.orientation.x = 0.0;
  thormang3_pelvis_pose_.pose.orientation.y = 0.0;
  thormang3_pelvis_pose_.pose.orientation.z = 0.0;
  thormang3_pelvis_pose_.pose.orientation.w = 1.0;
}

Thormang3Localization::~Thormang3Localization()
{

}

void Thormang3Localization::initialize()
{
  // subscriber
  thormang3_pelvis_msg_sub_ = ros_node_.subscribe("/robotis/pelvis_pose", 5,
                                                  &Thormang3Localization::thormang3PelvisPoseCallback, this);
}

void Thormang3Localization::thormang3PelvisPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  thormang3_pelvis_pose_ = *msg;

//  process();
}

void Thormang3Localization::process()
{
  pelvis_trans_.setOrigin(tf::Vector3(thormang3_pelvis_pose_.pose.position.x,
                                      thormang3_pelvis_pose_.pose.position.y,
                                      thormang3_pelvis_pose_.pose.position.z)
                          );

  tf::Quaternion q(thormang3_pelvis_pose_.pose.orientation.x,
                   thormang3_pelvis_pose_.pose.orientation.y,
                   thormang3_pelvis_pose_.pose.orientation.z,
                   thormang3_pelvis_pose_.pose.orientation.w);

  pelvis_trans_.setRotation(q);

  ros::Duration transform_tolerance(transform_tolerance_);
  ros::Time transform_expiration = (thormang3_pelvis_pose_.header.stamp + transform_tolerance);

  tf::StampedTransform tmp_tf_stamped(pelvis_trans_, transform_expiration, "world", "pelvis_link");

  broadcaster_.sendTransform(tmp_tf_stamped);
}

} // namespace thormang3
