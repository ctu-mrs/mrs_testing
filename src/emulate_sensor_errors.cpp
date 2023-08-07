/* includes //{ */

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <geometry_msgs/Point.h>

#include <mrs_msgs/RtkGps.h>

#include <std_srvs/Trigger.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/publisher_handler.h>

#include <random>

namespace mrs_uav_testing
{

/* class EmulateSensorErrors //{ */

class EmulateSensorErrors : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  ros::NodeHandle nh_;
  bool            is_initialized_ = false;

private:
  // subscribers and publishers
  mrs_lib::SubscribeHandler<mrs_msgs::RtkGps> sh_rtk_;

  mrs_lib::PublisherHandler<mrs_msgs::RtkGps> ph_rtk_;

  // simulation of RTK signal degradation
  geometry_msgs::Point jump_;
  geometry_msgs::Point min_jump_;
  geometry_msgs::Point max_jump_;

  int min_jump_time_;
  int max_jump_time_;

  double coeff_;

  double trigger_jump_time_;

  bool emulating_jump_;

  std::unique_ptr<std::mt19937>                           random_engine_;
  std::unique_ptr<std::uniform_real_distribution<double>> rand_x_;
  std::unique_ptr<std::uniform_real_distribution<double>> rand_y_;
  std::unique_ptr<std::uniform_real_distribution<double>> rand_z_;
  std::unique_ptr<std::uniform_real_distribution<double>> rand_time_;

  mrs_msgs::RtkFixType fix_type_;

private:
  void callbackRtk(mrs_lib::SubscribeHandler<mrs_msgs::RtkGps> &wrp);
};

//}

/* onInit() //{ */

void EmulateSensorErrors::onInit() {

  ros::NodeHandle nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  mrs_lib::ParamLoader param_loader(nh_, "EmulateSensorErrors");

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[EmulateSensorErrors]: could not load all parameters!");
    ros::shutdown();
  }

  // | ----------------------- subscribers ---------------------- |

  mrs_lib::SubscribeHandlerOptions shopts;
  shopts.nh                 = nh_;
  shopts.node_name          = "EmulateSensorErrors";
  shopts.no_message_timeout = mrs_lib::no_timeout;
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  shopts.queue_size         = 10;
  shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

  sh_rtk_ = mrs_lib::SubscribeHandler<mrs_msgs::RtkGps>(shopts, "rtk_in", &EmulateSensorErrors::callbackRtk, this);

  // | ----------------------- publishers ----------------------- |

  ph_rtk_ = mrs_lib::PublisherHandler<mrs_msgs::RtkGps>(nh_, "rtk_out", 1);

  // | ----------------------- finish init ---------------------- |

  jump_.x = 0.0;
  jump_.y = 0.0;
  jump_.z = 0.0;

  min_jump_.x = -5;
  min_jump_.y = -5;
  min_jump_.z = -5;

  max_jump_.x = 5;
  max_jump_.y = 5;
  max_jump_.z = 5;

  min_jump_time_ = 10;
  max_jump_time_ = 20;

  trigger_jump_time_ = ros::Time::now().toSec() + 30.0;

  coeff_ = 100000.0;

  std::random_device rd;
  random_engine_ = std::make_unique<std::mt19937>(rd());
  rand_x_        = std::make_unique<std::uniform_real_distribution<double>>(min_jump_.x / coeff_, max_jump_.x / coeff_);
  rand_y_        = std::make_unique<std::uniform_real_distribution<double>>(min_jump_.y / coeff_, max_jump_.y / coeff_);
  rand_z_        = std::make_unique<std::uniform_real_distribution<double>>(min_jump_.z / coeff_, max_jump_.z / coeff_);
  rand_time_     = std::make_unique<std::uniform_real_distribution<double>>(min_jump_time_, min_jump_time_);

  is_initialized_ = true;

  ROS_INFO("[EmulateSensorErrors]: initialized");
}

//}

// --------------------------------------------------------------
// |                          callbacks                         |
// --------------------------------------------------------------

/* callbackTersus() //{ */

void EmulateSensorErrors::callbackRtk(mrs_lib::SubscribeHandler<mrs_msgs::RtkGps> &wrp) {

  if (!is_initialized_)
    return;

  mrs_msgs::RtkGps rtk_msg_out;

  rtk_msg_out = *wrp.getMsg();

  if (rtk_msg_out.header.stamp.toSec() > trigger_jump_time_) {
    if (emulating_jump_) {
      jump_.x            = 0.0;
      jump_.y            = 0.0;
      jump_.z            = 0.0;
      fix_type_.fix_type = mrs_msgs::RtkFixType::RTK_FIX;
      const double jump_length = (*rand_time_)(*random_engine_);
      trigger_jump_time_ = rtk_msg_out.header.stamp.toSec() + jump_length;
      emulating_jump_    = false;
      ROS_INFO("[EmulateSensorErrors]: emulating RTK_FIX for %.2f s", jump_length);

    } else {
      jump_.x            = (*rand_x_)(*random_engine_);
      jump_.y            = (*rand_y_)(*random_engine_);
      jump_.z            = (*rand_z_)(*random_engine_);
      fix_type_.fix_type = mrs_msgs::RtkFixType::SPS;
      const double jump_length = (*rand_time_)(*random_engine_);
      trigger_jump_time_ = rtk_msg_out.header.stamp.toSec() + jump_length;
      emulating_jump_    = true;
      ROS_INFO("[EmulateSensorErrors]: emulating SINGLE with jump: [%.2f, %.2f, %.2f] for %.2f s", jump_.x*coeff_, jump_.y*coeff_, jump_.z*coeff_, jump_length);
    }
  }

  rtk_msg_out.gps.latitude += jump_.x;
  rtk_msg_out.gps.longitude += jump_.y;
  rtk_msg_out.gps.altitude += jump_.z;

  rtk_msg_out.fix_type = fix_type_;

  ph_rtk_.publish(rtk_msg_out);
}

//}

// --------------------------------------------------------------
// |                           timers                           |
// --------------------------------------------------------------

}  // namespace mrs_uav_testing

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_uav_testing::EmulateSensorErrors, nodelet::Nodelet)
