/* includes //{ */

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <stdio.h>
#include <stdlib.h>

#include <mrs_msgs/PositionCommand.h>
#include <mrs_msgs/ReferenceStampedSrv.h>
#include <std_srvs/Trigger.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/msg_extractor.h>

#include <random>

//}

namespace mrs_uav_testing
{

/* class RandomFlier //{ */

class RandomFlier : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  bool is_initialized_ = false;

  bool   callbackActivate([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  void   timerMain(const ros::TimerEvent& event);
  double randd(double from, double to);

  mrs_lib::SubscribeHandler<mrs_msgs::PositionCommand> sh_position_cmd_;

  ros::ServiceServer service_server_activate_;

  ros::ServiceClient service_client_reference_;

  ros::Timer main_timer_;

  double _main_timer_rate_;

  // parameters loaded from config file
  double _height_;
  bool   _randomize_distance_ = false;
  bool   _randomize_heading_  = false;
  double _max_distance_;

  bool active_ = true;

  ros::Time last_successfull_command_;
};

//}

/* onInit() //{ */

void RandomFlier::onInit(void) {

  ros::NodeHandle nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  ros::Time::waitForValid();

  mrs_lib::ParamLoader param_loader(nh_, "RandomFlier");

  // load parameters from config file
  param_loader.loadParam("main_timer_rate", _main_timer_rate_);
  param_loader.loadParam("height", _height_);
  param_loader.loadParam("active", active_);
  param_loader.loadParam("randomize_distance", _randomize_distance_);
  param_loader.loadParam("randomize_heading", _randomize_heading_);
  param_loader.loadParam("max_distance", _max_distance_);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[RandomFlier]: Could not load all parameters!");
    ros::shutdown();
  }

  mrs_lib::SubscribeHandlerOptions shopts;
  shopts.nh                 = nh_;
  shopts.node_name          = "RandomFlier";
  shopts.no_message_timeout = mrs_lib::no_timeout;
  shopts.threadsafe         = true;
  shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

  sh_position_cmd_ = mrs_lib::SubscribeHandler<mrs_msgs::PositionCommand>(shopts, "position_command_in");

  service_server_activate_  = nh_.advertiseService("activate_in", &RandomFlier::callbackActivate, this);
  service_client_reference_ = nh_.serviceClient<mrs_msgs::ReferenceStampedSrv>("reference_out");

  // initialize the random number generator
  /* srand(static_cast<unsigned int>(time(0))); */
  srand(time(NULL));

  last_successfull_command_ = ros::Time(0);

  main_timer_ = nh_.createTimer(ros::Rate(_main_timer_rate_), &RandomFlier::timerMain, this);

  // | ----------------------- finish init ---------------------- |

  is_initialized_ = true;

  ROS_INFO_ONCE("[RandomFlier]: initialized");
}

//}

// | ------------------------ callbacks ----------------------- |

/* callbackActivate() //{ */

bool RandomFlier::callbackActivate([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {

  if (!is_initialized_) {
    return false;
  }

  active_ = true;

  res.success = true;
  res.message = "home reseted";

  return true;
}

//}

// | ------------------------- timers ------------------------- |

/* timerMain() //{ */

void RandomFlier::timerMain([[maybe_unused]] const ros::TimerEvent& event) {

  if (!is_initialized_) {
    return;
  }

  if (!active_) {

    ROS_INFO_THROTTLE(1.0, "[RandomFlier]: waiting for activation");
    return;
  }

  if (!sh_position_cmd_.hasMsg()) {

    ROS_INFO_THROTTLE(1.0, "[RandomFlier]: waiting for PositionCommand");
    return;
  }

  auto [cmd_speed_x, cmd_speed_y, cmd_speed_z] = mrs_lib::getVelocity(sh_position_cmd_.getMsg());
  auto [cmd_x, cmd_y, cmd_z]                   = mrs_lib::getPosition(sh_position_cmd_.getMsg());

  // if the uav reach the previousy set destination
  if ((ros::Time::now() - last_successfull_command_).toSec() > 1.0 && fabs(cmd_speed_x) < 0.01 && fabs(cmd_speed_y) < 0.01) {

    // create new point to fly to
    mrs_msgs::ReferenceStampedSrv new_point;
    new_point.request.header.frame_id = "slam_mapping_origin";

    double dist, direction, heading, height;

    if (_randomize_distance_) {
      dist = randd(0, _max_distance_);
    } else {
      dist = _max_distance_;
    }

    direction = randd(-M_PI, M_PI);

    if (_randomize_heading_) {
      heading = randd(-M_PI, M_PI);
    } else {
      heading = 0;
    }

    height = randd(1.0, 20.0);

    /* if (cmd_x > 5.0) { */
    /*   if (cmd_z < 10) { */
    /*     height = randd(10.0, 20.0); */
    /*   } else { */
    /*     height = randd(1.0, 10.0); */
    /*   } */
    /* } */

    double transfer_p = randd(0, 100);
    double random_y   = randd(-10, 10);

    while (true) {

      if (transfer_p > 50) {
        new_point.request.reference.position.x = cmd_x + cos(direction) * dist;
        new_point.request.reference.position.y = cmd_y + sin(direction) * dist;
      } else {
        new_point.request.reference.position.x = 10;
        new_point.request.reference.position.y = random_y;
      }
      new_point.request.reference.position.z = height;
      new_point.request.reference.heading    = heading;

      if (service_client_reference_.call(new_point)) {

        if (new_point.response.success) {

          ROS_INFO("New goal: %.2f %.2f %.2f", new_point.request.reference.position.x, new_point.request.reference.position.y,
                   new_point.request.reference.position.z);

          last_successfull_command_ = ros::Time::now();
          break;

        } else {

          ROS_WARN_THROTTLE(1.0, "New goal: %.2f %.2f service response success: false. Check control tab.", new_point.request.reference.position.x,
                            new_point.request.reference.position.y);

          dist -= 0.1;

          if (dist < 1.0) {

            if (_randomize_distance_) {
              dist = randd(0, _max_distance_);
            } else {
              dist = _max_distance_;
            }

            direction = randd(-M_PI, M_PI);
          }
        }
      } else {
        ROS_WARN_THROTTLE(1.0, "New goal: %.2f %.2f service failed. Check control tab.", new_point.request.reference.position.x,
                          new_point.request.reference.position.y);
      }
    }
  }
}

//}

// | ------------------------ routines ------------------------ |

/* randd() //{ */

double RandomFlier::randd(double from, double to) {

  double zero_to_one = double((float)rand()) / double(RAND_MAX);

  return floor(to - from) * zero_to_one + from;
}

//}

}  // namespace mrs_uav_testing

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_uav_testing::RandomFlier, nodelet::Nodelet)
