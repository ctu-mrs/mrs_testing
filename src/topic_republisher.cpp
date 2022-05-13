/* includes //{ */

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <nav_msgs/Odometry.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/profiler.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/transformer.h>

//}

namespace mrs_uav_testing
{

/* class TopicRepublisher //{ */

class TopicRepublisher : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  ros::NodeHandle nh_;
  bool            is_initialized_ = false;

private:
  // subscribers and publishers
  ros::Subscriber sub_odom_;
  ros::Publisher  pub_odom_;

  std::unique_ptr<mrs_lib::Transformer> transformer_;
  std::string                           target_frame_ = "uav3/rtk_origin";

private:
  void callbackOdom(const nav_msgs::OdometryConstPtr &msg);
};

//}

/* onInit() //{ */

void TopicRepublisher::onInit() {

  ros::NodeHandle nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  // | ----------------------- subscribers ---------------------- |

  sub_odom_ = nh_.subscribe("odom_in", 1, &TopicRepublisher::callbackOdom, this, ros::TransportHints().tcpNoDelay());

  // | ----------------------- publishers ----------------------- |

  pub_odom_ = nh_.advertise<nav_msgs::Odometry>("odom_out", 1);

  transformer_ = std::make_unique<mrs_lib::Transformer>(nh_, "TopicRepublisher");

  is_initialized_ = true;

  ROS_INFO("[TopicRepublisher]: initialized");
}

//}

// --------------------------------------------------------------
// |                          callbacks                         |
// --------------------------------------------------------------

/* callbackOdom() //{ */

void TopicRepublisher::callbackOdom(const nav_msgs::OdometryConstPtr &msg) {

  if (!is_initialized_)
    return;

  nav_msgs::Odometry odom_out = *msg;

  geometry_msgs::PoseStamped pose_tmp;
  pose_tmp.header = msg->header;
  pose_tmp.pose = msg->pose.pose;

  auto response = transformer_->transformSingle(pose_tmp, target_frame_);
  if (response) {
    odom_out.pose.pose = response.value().pose;
    odom_out.header = response.value().header;
  } else {
    ROS_WARN_THROTTLE(1.0, "[TopicRepublisher]: Transform from %s to %s failed.", msg->header.frame_id.c_str(), target_frame_.c_str());
    return;
  }

  try {
    pub_odom_.publish(odom_out);
  }
  catch (...) {
    ROS_ERROR("exception caught during publishing topic '%s'", pub_odom_.getTopic().c_str());
  }
}

//}

}  // namespace mrs_uav_testing

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_uav_testing::TopicRepublisher, nodelet::Nodelet)
