/* includes //{ */

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/PoseStamped.h>

#include <mrs_lib/attitude_converter.h>

#include <random>

//}

namespace mrs_uav_testing
{

/* class TfTest //{ */

class TfTest : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  bool is_initialized_ = false;

  ros::Timer main_timer_;

  tf2_ros::Buffer tf_buffer_;

  std::unique_ptr<tf2_ros::TransformListener> tf_listener_ptr_;

  void timerMain(const ros::TimerEvent& event);
};

//}

/* onInit() //{ */

void TfTest::onInit(void) {

  ros::NodeHandle nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  ros::Time::waitForValid();

  main_timer_ = nh_.createTimer(ros::Rate(1.0), &TfTest::timerMain, this);

  tf_listener_ptr_ = std::make_unique<tf2_ros::TransformListener>(tf_buffer_);

  is_initialized_ = true;

  ROS_INFO_ONCE("[TfTest]: initialized");
}

//}

// | ------------------------- timers ------------------------- |

/* timerMain() //{ */

void TfTest::timerMain([[maybe_unused]] const ros::TimerEvent& event) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[TfTest]: main timer spinning");

  geometry_msgs::TransformStamped tf;

  try {
    tf = tf_buffer_.lookupTransform("B", "A", ros::Time::now(), ros::Duration(1.0));
  }
  catch (...) {
    ROS_ERROR("[TfTest]: exception");
    return;
  }

  {
    double      x    = tf.transform.translation.x;
    double      y    = tf.transform.translation.y;
    double      z    = tf.transform.translation.z;
    std::string from = tf.header.frame_id;
    std::string to   = tf.child_frame_id;

    ROS_INFO("[TfTest]: x = %.2f, y = %.2f, z = %.2f, from = %s, to = %s", x, y, z, from.c_str(), to.c_str());
  }

  // --------------------------------------------------------------
  // |                     test the transform                     |
  // --------------------------------------------------------------

  {
    geometry_msgs::PoseStamped pose;
    geometry_msgs::PoseStamped trans_pose;

    pose.header.frame_id = "A";
    pose.header.stamp    = ros::Time::now();

    pose.pose.position.x = 2;
    pose.pose.position.y = 0;
    pose.pose.position.z = 0;

    pose.pose.orientation = mrs_lib::AttitudeConverter(0, 0, 0);

    tf2::doTransform(pose, trans_pose, tf);

    double x = trans_pose.pose.position.x;
    double y = trans_pose.pose.position.y;
    double z = trans_pose.pose.position.z;

    ROS_INFO("[TfTest]: transformed pose x = %.2f, y = %.2f, z = %.2f", x, y, z);
  }

  main_timer_.stop();
}

//}

}  // namespace mrs_uav_testing

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_uav_testing::TfTest, nodelet::Nodelet)
