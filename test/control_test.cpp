/* includes //{ */

#include <ros/ros.h>

#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>

#include <std_msgs/Float64.h>

#include <mrs_msgs/ReferenceStamped.h>
#include <mrs_msgs/TrajectoryReference.h>
#include <mrs_msgs/TrajectoryReferenceSrv.h>
#include <mrs_msgs/ReferenceStampedSrv.h>
#include <mrs_msgs/Vec4.h>
#include <mrs_msgs/Vec1.h>
#include <mrs_msgs/String.h>
#include <mrs_msgs/PositionCommand.h>
#include <mrs_msgs/ControlManagerDiagnostics.h>
#include <mrs_msgs/SpawnerDiagnostics.h>

#include <nav_msgs/Odometry.h>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>

#include <mutex>

#include <mrs_lib/mutex.h>
#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/msg_extractor.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/geometry/cyclic.h>
#include <mrs_lib/geometry/misc.h>

#include <tf/transform_datatypes.h>

#ifdef ROSTEST
#include <gtest/gtest.h>
#include <ros/console.h>
#include <log4cxx/logger.h>
#endif

//}

/* using //{ */

using radians  = mrs_lib::geometry::radians;
using sradians = mrs_lib::geometry::sradians;

using vec2_t = mrs_lib::geometry::vec_t<2>;
using vec3_t = mrs_lib::geometry::vec_t<3>;

//}

/* //{ state machine states */

#define TRACKER_LINE 0
#define TRACKER_MPC 1

// state machine
typedef enum
{
  IDLE_STATE,
  SPAWN_STATE,
  WAIT_FOR_SPAWN,
  TAKEOFF_STATE,
  CHANGE_ESTIMATOR_STATE,
  CHANGE_TRACKER_STATE,
  SET_REFERENCE_TOPIC_STATE,
  SET_REFERENCE_SERVICE_STATE,
  GOTO_SERVICE_STATE,
  GOTO_RELATIVE_SERVICE_STATE,
  GOTO_ALTITUDE_SERVICE_STATE,
  SET_HEADING_SERVICE_STATE,
  SET_heading_RELATIVE_SERVICE_STATE,
  TRAJECTORY_LOAD_STATIC_TOPIC_STATE,
  TRAJECTORY_GOTO_START_STATE,
  TRAJECTORY_START_TRACKING_STATE,
  TRAJECTORY_LOAD_STATIC_SERVICE_STATE,
  TRAJECTORY_GOTO_START_SERVICE_STATE,
  TRAJECTORY_START_TRACKING_SERVICE_STATE,
  TRAJECTORY_LOAD_DYNAMIC_TOPIC_STATE,
  TRAJECTORY_LOAD_DYNAMIC_SERVICE_STATE,
  TRAJECTORY_NOT_USE_heading_STATE,
  TRAJECTORY_CIRCLE_LOOP,
  TRAJECTORY_CIRCLE_PRE_PAUSE,
  TRAJECTORY_CIRCLE_PAUSE,
  TRAJECTORY_CIRCLE_POST_PAUSE,
  TRAJECTORY_HEADLESS_LOAD_SERVICE_STATE,
  TRAJECTORY_HEADLESS_FLY_TO_START_SERVICE_STATE,
  TRAJECTORY_HEADLESS_START_TRACKING_SERVICE_STATE,
  LAND_HOME_STATE,
  GOTO_ORIGIN_STATE,
  LAND_STATE,
  FINISHED_STATE,
  ERROR_STATE,
} ControlState_t;

const char* state_names[34] = {
    "IDLE_STATE",
    "SPAWN_STATE",
    "WAIT_FOR_SPAWN",
    "TAKEOFF_STATE",
    "CHANGE_ESTIMATOR_STATE",
    "CHANGE_TRACKER_STATE",
    "SET_REFERENCE_TOPIC_STATE",
    "SET_REFERENCE_SERVICE_STATE",
    "GOTO_SERVICE_STATE",
    "GOTO_RELATIVE_SERVICE_STATE",
    "GOTO_ALTITUDE_SERVICE_STATE",
    "SET_HEADING_SERVICE_STATE",
    "SET_heading_RELATIVE_SERVICE_STATE",
    "TRAJECTORY_LOAD_STATIC_TOPIC_STATE",
    "TRAJECTORY_GOTO_START_STATE",
    "TRAJECTORY_START_TRACKING_STATE",
    "TRAJECTORY_LOAD_STATIC_SERVICE_STATE",
    "TRAJECTORY_GOTO_START_SERVICE_STATE",
    "TRAJECTORY_START_TRACKING_SERVICE_STATE",
    "TRAJECTORY_LOAD_DYNAMIC_TOPIC_STATE",
    "TRAJECTORY_LOAD_DYNAMIC_SERVICE_STATE",
    "TRAJECTORY_NOT_USE_heading_STATE",
    "TRAJECTORY_CIRCLE_LOOP",
    "TRAJECTORY_CIRCLE_PRE_PAUSE",
    "TRAJECTORY_CIRCLE_PAUSE",
    "TRAJECTORY_CIRCLE_POST_PAUSE",
    "TRAJECTORY_HEADLESS_LOAD_SERVICE_STATE",
    "TRAJECTORY_HEADLESS_FLY_TO_START_SERVICE_STATE",
    "TRAJECTORY_HEADLESS_START_TRACKING_SERVICE_STATE",
    "LAND_HOME_STATE",
    "GOTO_ORIGIN_STATE",
    "LAND_STATE",
    "FINISHED_STATE",
    "ERROR_STATE",
};

//}

/* //{ class ControlTest */

class ControlTest {

public:
  ControlTest();

  bool finished(void);
  bool result(void);

private:
  ros::NodeHandle nh_;
  bool            is_initialized_ = false;

  // | ----------------------- subscribers ---------------------- |

  mrs_lib::SubscribeHandler<mrs_msgs::SpawnerDiagnostics>        sh_spawner_diag_;
  mrs_lib::SubscribeHandler<nav_msgs::Odometry>                  sh_odometry_;
  mrs_lib::SubscribeHandler<mrs_msgs::PositionCommand>           sh_position_cmd_;
  mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics> sh_control_manager_diag_;

  // | ----------------------- publishers ----------------------- |

  ros::Publisher publisher_reference_;
  ros::Publisher publisher_trajectory_reference_;

  // | --------------------- service clients -------------------- |

  ros::ServiceClient service_client_switch_tracker_;
  ros::ServiceClient service_client_arm_;
  ros::ServiceClient service_client_offboard_;
  ros::ServiceClient service_client_takeoff_;
  ros::ServiceClient service_client_land_;
  ros::ServiceClient service_client_land_home_;
  ros::ServiceClient service_client_motors_;
  ros::ServiceClient service_client_set_reference_;
  ros::ServiceClient service_client_goto_;
  ros::ServiceClient service_client_goto_relative_;
  ros::ServiceClient service_client_goto_altitude_;
  ros::ServiceClient service_client_set_heading_;
  ros::ServiceClient service_client_set_heading_relative_;
  ros::ServiceClient service_client_switch_estimator_;
  ros::ServiceClient service_client_spawn_uav_;

  // trajectory tracking
  ros::ServiceClient service_client_trajectory_reference_;
  ros::ServiceClient service_client_goto_trajectory_start_;
  ros::ServiceClient service_client_start_trajectory_tracking_;
  ros::ServiceClient service_client_resume_trajectory_tracking_;
  ros::ServiceClient service_client_stop_trajectory_tracking_;

  bool callbackStart(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

  // | --------------------- service servers -------------------- |

  ros::ServiceServer service_server_start_;

  // | ----------------------- main timer ----------------------- |

  ros::Timer timer_main_;
  void       timerMain(const ros::TimerEvent& event);

  // | ---------------------- state machine --------------------- |

  ControlState_t current_state_  = IDLE_STATE;
  ControlState_t previous_state_ = IDLE_STATE;
  void           changeState(const ControlState_t new_state);

  // | ------------------------- params ------------------------- |

  bool _start_with_takeoff_ = false;

  std::string _spawn_args_;

  double _max_xy_;
  double _min_xy_;
  double _max_z_;
  double _min_z_;
  double _max_heading_;
  double _min_heading_;

  double _trajectory_dt_;

  double _line_trajectory_p1_;
  double _line_trajectory_p2_;
  double _line_trajectory_speed_;
  double _line_trajectory_heading_rate_;

  double _looping_circle_speed_;
  double _looping_circle_radius_;
  double _looping_circle_duration_;

  double _goto_relative_altitude_down_;
  double _goto_relative_altitude_up_;

  bool _test_line_tracker_ = false;
  bool _test_mpc_gotos_    = true;

  bool        _switch_estimator_after_takeoff_ = false;
  std::string _target_estimator_;

  // | ------------------------ routines ------------------------ |

  double randd(const double from, const double to);
  double genheading(void);
  double genXY(void);
  double genZ(void);
  bool   inDesiredState(void);
  bool   isStationary(void);
  bool   trackerReady(void);
  void   switchTracker(const std::string tracker_name);
  void   spawnUAV(const std::string args);
  void   switchEstimator(const std::string estimator_name);
  void   startTrajectoryTracking(void);
  void   stopTrajectoryTracking(void);
  void   resumeTrajectoryTracking(void);
  void   gotoTrajectoryStart(void);
  void   setTrajectorySrv(const mrs_msgs::TrajectoryReference trajectory);

  mrs_msgs::TrajectoryReference createLoopingCircleTrajectory();
  mrs_msgs::TrajectoryReference createLineTrajectory(const bool ascending, const bool fly_now, const bool use_heading);
  mrs_msgs::TrajectoryReference createHeadlessTrajectory();

  // | ----------------------- global vars ---------------------- |

  int         active_tracker_   = -1;
  int         takeoff_num_      = 0;
  std::string active_estimator_ = "UNKNOWN";

  double des_x_;
  double des_y_;
  double des_z_;
  double des_heading_;

  double home_x_;
  double home_y_;

  ros::Time timeout_;

  ros::Time looping_start_time_;

  mrs_msgs::TrajectoryReference goal_trajectory_;
};

//}

/* //{ ControlTest() */

ControlTest::ControlTest() {

  ros::NodeHandle nh_ = ros::NodeHandle("~");

  ros::Time::waitForValid();

  timeout_ = ros::Time::now();

  srand(static_cast<unsigned int>(ros::Time::now().toSec()));

  // --------------------------------------------------------------
  // |                           params                           |
  // --------------------------------------------------------------

  mrs_lib::ParamLoader param_loader(nh_, "ControlTest");

  param_loader.loadParam("start_with_takeoff", _start_with_takeoff_);

  param_loader.loadParam("spawn_args", _spawn_args_);

  param_loader.loadParam("max_xy", _max_xy_);
  param_loader.loadParam("min_xy", _min_xy_);
  param_loader.loadParam("max_z", _max_z_);
  param_loader.loadParam("min_z", _min_z_);
  param_loader.loadParam("max_heading", _max_heading_);
  param_loader.loadParam("min_heading", _min_heading_);
  param_loader.loadParam("goto_relative_altitude_down", _goto_relative_altitude_down_);
  param_loader.loadParam("goto_relative_altitude_up", _goto_relative_altitude_up_);

  param_loader.loadParam("trajectory_dt", _trajectory_dt_);

  param_loader.loadParam("line_trajectory/p1", _line_trajectory_p1_);
  param_loader.loadParam("line_trajectory/p2", _line_trajectory_p2_);
  param_loader.loadParam("line_trajectory/speed", _line_trajectory_speed_);
  param_loader.loadParam("line_trajectory/heading_rate", _line_trajectory_heading_rate_);

  param_loader.loadParam("looping_circle_trajectory/radius", _looping_circle_radius_);
  param_loader.loadParam("looping_circle_trajectory/speed", _looping_circle_speed_);
  param_loader.loadParam("looping_circle_trajectory/duration", _looping_circle_duration_);

  param_loader.loadParam("test_line_tracker", _test_line_tracker_);
  param_loader.loadParam("test_mpc_gotos", _test_mpc_gotos_);

  param_loader.loadParam("switch_estimator_after_takeoff/active", _switch_estimator_after_takeoff_);
  param_loader.loadParam("switch_estimator_after_takeoff/target_estimator", _target_estimator_);

  if (!param_loader.loadedSuccessfully()) {
    ros::shutdown();
  }

  // | ------------- calculate stuff from the params ------------ |

  _line_trajectory_speed_ = _line_trajectory_speed_ / 1.414;

  // | ----------------------- subscribers ---------------------- |

  mrs_lib::SubscribeHandlerOptions shopts;
  shopts.nh                 = nh_;
  shopts.node_name          = "ControlTest";
  shopts.no_message_timeout = mrs_lib::no_timeout;
  shopts.threadsafe         = true;
  shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

  sh_odometry_ = mrs_lib::SubscribeHandler<nav_msgs::Odometry>(shopts, "odometry_in");

  sh_spawner_diag_ = mrs_lib::SubscribeHandler<mrs_msgs::SpawnerDiagnostics>(shopts, "spawner_diagnostics_in");

  sh_position_cmd_ = mrs_lib::SubscribeHandler<mrs_msgs::PositionCommand>(shopts, "position_command_in");

  sh_control_manager_diag_ = mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics>(shopts, "control_manager_diagnostics_in");

  // | ------------------- std tracker topics ------------------- |

  publisher_reference_ = nh_.advertise<mrs_msgs::ReferenceStamped>("reference_out", 1);

  // | --------------- additional tracker topics ---------------- |

  publisher_trajectory_reference_ = nh_.advertise<mrs_msgs::TrajectoryReference>("trajectory_reference_out", 1);

  // | ---------------- takeoff/landing services ---------------- |

  service_client_switch_tracker_ = nh_.serviceClient<mrs_msgs::String>("switch_tracker_out");
  service_client_motors_         = nh_.serviceClient<std_srvs::SetBool>("motors_out");
  service_client_arm_            = nh_.serviceClient<mavros_msgs::CommandBool>("arm_out");
  service_client_offboard_       = nh_.serviceClient<mavros_msgs::SetMode>("offboard_out");
  service_client_takeoff_        = nh_.serviceClient<std_srvs::Trigger>("takeoff_out");
  service_client_land_           = nh_.serviceClient<std_srvs::Trigger>("land_out");
  service_client_land_home_      = nh_.serviceClient<std_srvs::Trigger>("land_home_out");

  // | ----------- control manager reference interface ---------- |

  service_client_set_reference_              = nh_.serviceClient<mrs_msgs::ReferenceStampedSrv>("reference_out");
  service_client_goto_                       = nh_.serviceClient<mrs_msgs::Vec4>("goto_out");
  service_client_goto_relative_              = nh_.serviceClient<mrs_msgs::Vec4>("goto_relative_out");
  service_client_goto_altitude_              = nh_.serviceClient<mrs_msgs::Vec1>("goto_altitude_out");
  service_client_set_heading_                = nh_.serviceClient<mrs_msgs::Vec1>("set_heading_out");
  service_client_set_heading_relative_       = nh_.serviceClient<mrs_msgs::Vec1>("set_heading_relative_out");
  service_client_trajectory_reference_       = nh_.serviceClient<mrs_msgs::TrajectoryReferenceSrv>("trajectory_reference_out");
  service_client_goto_trajectory_start_      = nh_.serviceClient<std_srvs::Trigger>("goto_trajectory_start_out");
  service_client_start_trajectory_tracking_  = nh_.serviceClient<std_srvs::Trigger>("start_trajectory_tracking_out");
  service_client_stop_trajectory_tracking_   = nh_.serviceClient<std_srvs::Trigger>("stop_trajectory_tracking_out");
  service_client_resume_trajectory_tracking_ = nh_.serviceClient<std_srvs::Trigger>("resume_trajectory_tracking_out");

  // | -------------------- odometry services ------------------- |

  service_client_switch_estimator_ = nh_.serviceClient<mrs_msgs::String>("switch_estimator_out");

  // | ------------------ drone spawner service ----------------- |

  service_client_spawn_uav_ = nh_.serviceClient<mrs_msgs::String>("spawn_out");

  // | --------------------- service servers -------------------- |

  service_server_start_ = nh_.advertiseService("start_in", &ControlTest::callbackStart, this);

  // | ------------------------- timers ------------------------- |

  timer_main_ = nh_.createTimer(ros::Rate(10), &ControlTest::timerMain, this);

  // | ---------------------- finish inint ---------------------- |

  is_initialized_ = true;

  ROS_INFO("[ControlTest]: initialized");
}

//}

// | ------------------------- timers ------------------------- |

/* //{ timerMain() */

void ControlTest::timerMain([[maybe_unused]] const ros::TimerEvent& event) {

  if (!is_initialized_)
    return;

  double odom_x, odom_y, odom_z;

  if (sh_odometry_.hasMsg()) {

    std::tie(odom_x, odom_y, odom_z) = mrs_lib::getPosition(sh_odometry_.getMsg());
  }

  std::string active_tracker_name;

  if (sh_control_manager_diag_.hasMsg()) {

    active_tracker_name = sh_control_manager_diag_.getMsg()->active_tracker;
  }

  if ((ros::Time::now() - timeout_).toSec() > 180.0) {
    ROS_ERROR("[ControlTest]: TIMEOUT, TEST FAILED!!");
    changeState(ERROR_STATE);
  }

  switch (current_state_) {

    case IDLE_STATE: {

      ROS_INFO_THROTTLE(1.0, "[ControlTest]: idling");

      if (sh_spawner_diag_.hasMsg()) {
        changeState(SPAWN_STATE);
      } else {
        ROS_INFO_THROTTLE(1.0, "[ControlTest]: waiting for spawner diagnostics");
      }

      break;
    }

    case SPAWN_STATE: {

      break;
    }

    case WAIT_FOR_SPAWN: {

      ROS_INFO_THROTTLE(1.0, "[ControlTest]: waiting for UAV");

      if (_start_with_takeoff_) {
        if (sh_spawner_diag_.hasMsg()) {
          mrs_msgs::SpawnerDiagnostics diag = *sh_spawner_diag_.getMsg();
          if (diag.spawn_called && !diag.processing) {
            if (sh_odometry_.hasMsg() && sh_control_manager_diag_.hasMsg()) {
              changeState(TAKEOFF_STATE);
            }
          }
        }
      }

      break;
    }

    case TAKEOFF_STATE: {

      if (active_tracker_name == "MpcTracker" && trackerReady()) {

        ROS_INFO("[ControlTest]: takeoff_num=%d", takeoff_num_);

        takeoff_num_++;

        if (_switch_estimator_after_takeoff_) {

          changeState(CHANGE_ESTIMATOR_STATE);

        } else {

          changeState(CHANGE_TRACKER_STATE);
        }

        break;
      }
    }

    case CHANGE_ESTIMATOR_STATE: {

      if (active_estimator_ == _target_estimator_) {

        changeState(CHANGE_TRACKER_STATE);
      }

      break;
    }

    case CHANGE_TRACKER_STATE: {

      if (trackerReady()) {

        // after the first takeoff
        if (takeoff_num_ == 1) {

          if (active_tracker_ == TRACKER_MPC) {
            if (_test_mpc_gotos_) {
              changeState(SET_REFERENCE_TOPIC_STATE);
            } else {
              changeState(TRAJECTORY_LOAD_STATIC_TOPIC_STATE);
            }
          } else {
            changeState(SET_REFERENCE_TOPIC_STATE);
          }

        } else if (takeoff_num_ == 2) {
          changeState(GOTO_ORIGIN_STATE);  // after testing land_home
        }
      }

      break;
    }

    case SET_REFERENCE_TOPIC_STATE: {

      if (inDesiredState() && trackerReady()) {
        changeState(ControlState_t(int(current_state_) + 1));
      }

      break;
    }

    case SET_REFERENCE_SERVICE_STATE: {

      if (inDesiredState() && trackerReady()) {
        changeState(ControlState_t(int(current_state_) + 1));
      }

      break;
    }

    case GOTO_SERVICE_STATE: {

      if (inDesiredState() && trackerReady()) {
        changeState(ControlState_t(int(current_state_) + 1));
      }

      break;
    }

    case GOTO_RELATIVE_SERVICE_STATE: {

      if (inDesiredState() && trackerReady()) {
        changeState(ControlState_t(int(current_state_) + 1));
      }

      break;
    }

    case GOTO_ALTITUDE_SERVICE_STATE: {

      if (inDesiredState() && trackerReady()) {
        changeState(ControlState_t(int(current_state_) + 1));
      }

      break;
    }

    case SET_HEADING_SERVICE_STATE: {

      if (inDesiredState() && trackerReady()) {
        changeState(ControlState_t(int(current_state_) + 1));
      }

      break;
    }

    case SET_heading_RELATIVE_SERVICE_STATE: {

      if (inDesiredState() && trackerReady()) {

        if (active_tracker_ == TRACKER_LINE) {
          changeState(CHANGE_TRACKER_STATE);
        } else if (active_tracker_ == TRACKER_MPC) {
          changeState(TRAJECTORY_LOAD_STATIC_TOPIC_STATE);
        }
      }

      break;
    }

    case TRAJECTORY_LOAD_STATIC_TOPIC_STATE: {

      changeState(ControlState_t(int(current_state_) + 1));

      break;
    }

    case TRAJECTORY_GOTO_START_STATE: {

      if (inDesiredState() && trackerReady()) {
        changeState(ControlState_t(int(current_state_) + 1));
      }

      break;
    }

    case TRAJECTORY_START_TRACKING_STATE: {

      if (inDesiredState() && trackerReady()) {
        changeState(ControlState_t(int(current_state_) + 1));
      }

      break;
    }

    case TRAJECTORY_LOAD_STATIC_SERVICE_STATE: {

      changeState(ControlState_t(int(current_state_) + 1));

      break;
    }

    case TRAJECTORY_GOTO_START_SERVICE_STATE: {

      if (inDesiredState() && trackerReady()) {
        changeState(ControlState_t(int(current_state_) + 1));
      }

      break;
    }

    case TRAJECTORY_START_TRACKING_SERVICE_STATE: {

      if (inDesiredState() && trackerReady()) {
        changeState(ControlState_t(int(current_state_) + 1));
      }

      break;
    }

    case TRAJECTORY_LOAD_DYNAMIC_TOPIC_STATE: {

      if (inDesiredState() && trackerReady()) {
        changeState(ControlState_t(int(current_state_) + 1));
      }

      break;
    }

    case TRAJECTORY_LOAD_DYNAMIC_SERVICE_STATE: {

      if (inDesiredState() && trackerReady()) {
        changeState(ControlState_t(int(current_state_) + 1));
      }

      break;
    }

    case TRAJECTORY_NOT_USE_heading_STATE: {

      if (inDesiredState() && trackerReady()) {
        changeState(ControlState_t(int(current_state_) + 1));
      }

      break;
    }

    case TRAJECTORY_CIRCLE_LOOP: {

      auto [cmd_x, cmd_y, cmd_z] = mrs_lib::getPosition(sh_position_cmd_.getMsg());

      if ((ros::Time::now() - looping_start_time_).toSec() > _looping_circle_duration_) {
        if (mrs_lib::geometry::dist(vec3_t(odom_x, odom_y, odom_z), vec3_t(cmd_x, cmd_y, cmd_z)) < 2.0) {
          changeState(ControlState_t(int(current_state_) + 1));
        }
      }

      break;
    }

    case TRAJECTORY_CIRCLE_PRE_PAUSE: {

      if ((ros::Time::now() - looping_start_time_).toSec() > 10.0) {
        changeState(ControlState_t(int(current_state_) + 1));
      }

      break;
    }

    case TRAJECTORY_CIRCLE_PAUSE: {

      if (isStationary() && trackerReady()) {
        changeState(ControlState_t(int(current_state_) + 1));
      }

      break;
    }

    case TRAJECTORY_CIRCLE_POST_PAUSE: {

      auto [cmd_x, cmd_y, cmd_z] = mrs_lib::getPosition(sh_position_cmd_.getMsg());

      if ((ros::Time::now() - looping_start_time_).toSec() > 20.0) {
        if (mrs_lib::geometry::dist(vec3_t(odom_x, odom_y, odom_z), vec3_t(cmd_x, cmd_y, cmd_z)) < 2.0) {
          changeState(ControlState_t(int(current_state_) + 1));
        }
      }

      break;
    }

    case TRAJECTORY_HEADLESS_LOAD_SERVICE_STATE: {

      changeState(ControlState_t(int(current_state_) + 1));

      break;
    }

    case TRAJECTORY_HEADLESS_FLY_TO_START_SERVICE_STATE: {

      if (inDesiredState() && trackerReady()) {
        changeState(ControlState_t(int(current_state_) + 1));
      }

      break;
    }

    case TRAJECTORY_HEADLESS_START_TRACKING_SERVICE_STATE: {

      if (inDesiredState() && trackerReady()) {
        changeState(GOTO_ORIGIN_STATE);
      }

      break;
    }

    case LAND_HOME_STATE: {

      if (active_tracker_name == "NullTracker") {
        ROS_INFO("[ControlTest]: %s", active_tracker_name.c_str());
        changeState(TAKEOFF_STATE);
      }

      break;
    }

    case GOTO_ORIGIN_STATE: {

      if (inDesiredState() && trackerReady()) {
        if (takeoff_num_ == 1) {
          changeState(LAND_HOME_STATE);
        } else if (takeoff_num_ == 2) {
          changeState(LAND_STATE);
        }
      }

      break;
    }

    case LAND_STATE: {

      if (active_tracker_name == "NullTracker" && mrs_lib::geometry::dist(vec2_t(des_x_, des_y_), vec2_t(odom_x, odom_y)) < 1.0) {
        changeState(ControlState_t(int(current_state_) + 1));
      }

      break;
    }

    case FINISHED_STATE: {

      break;
    }

    case ERROR_STATE: {

      break;
    }
  }

  if (current_state_ >= SET_REFERENCE_TOPIC_STATE) {

    auto [cmd_x, cmd_y, cmd_z] = mrs_lib::getPosition(sh_position_cmd_.getMsg());

    double cmd_heading = 0;
    try {
      cmd_heading = mrs_lib::getHeading(sh_position_cmd_.getMsg());
    }
    catch (mrs_lib::AttitudeConverter::GetHeadingException e) {
      ROS_ERROR_THROTTLE(1.0, "[ControlTest]: exception caught: '%s'", e.what());
    }

    double odom_heading = 0;
    try {
      odom_heading = mrs_lib::getHeading(sh_odometry_.getMsg());
    }
    catch (mrs_lib::AttitudeConverter::GetHeadingException e) {
      ROS_ERROR_THROTTLE(1.0, "[ControlTest]: exception caught: '%s'", e.what());
    }

    ROS_INFO_THROTTLE(5.0, " ");
    ROS_INFO_THROTTLE(5.0, "[ControlTest]: desired: %.2f %.2f %.2f %.2f", des_x_, des_y_, des_z_, des_heading_);
    ROS_INFO_THROTTLE(5.0, "[ControlTest]: cmd: %.2f %.2f %.2f %.2f", cmd_x, cmd_y, cmd_z, cmd_heading);
    ROS_INFO_THROTTLE(5.0, "[ControlTest]: odom: %.2f %.2f %.2f %.2f", odom_x, odom_y, odom_z, odom_heading);
    ROS_INFO_THROTTLE(5.0, " ");
  }
}

//}

// | ------------------------ callbacks ----------------------- |

/* //{ callbackStart() */

bool ControlTest::callbackStart([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {

  if (!is_initialized_)
    return false;

  ROS_INFO("[ControlTest]: start triggered by service");

  takeoff_num_++;

  if (_switch_estimator_after_takeoff_) {

    changeState(CHANGE_ESTIMATOR_STATE);

  } else {

    changeState(CHANGE_TRACKER_STATE);
  }

  res.success = true;
  res.message = "started";

  return true;
}

//}

// | ---------------------- other methods --------------------- |

/* //{ changeState() */

void ControlTest::changeState(const ControlState_t new_state) {

  double odom_x, odom_y, odom_z;

  if (sh_odometry_.hasMsg()) {
    std::tie(odom_x, odom_y, odom_z) = mrs_lib::getPosition(sh_odometry_.getMsg());
  }

  ROS_INFO("[ControlTest]: chaging state %s -> %s", state_names[current_state_], state_names[new_state]);

  previous_state_ = current_state_;
  current_state_  = new_state;

  mrs_msgs::ReferenceStamped    goal_reference_stamped_topic;
  mrs_msgs::ReferenceStampedSrv goal_reference_stamped_srv;
  std_msgs::Float64             goal_float64;
  mrs_msgs::Vec4                goal_vec4;
  mrs_msgs::Vec1                goal_vec1;
  std_srvs::SetBool             goal_bool;
  mavros_msgs::CommandBool      goal_mavros_commandbool;
  mavros_msgs::SetMode          goal_mavros_set_mode;
  std_srvs::Trigger             goal_trigger;
  mrs_msgs::Reference           trajectory_point;

  ros::Duration wait(1.0);

  double cmd_x, cmd_y, cmd_z, cmd_heading;

  if (current_state_ > TAKEOFF_STATE) {

    std::tie(cmd_x, cmd_y, cmd_z) = mrs_lib::getPosition(sh_position_cmd_.getMsg());

    try {
      cmd_heading = mrs_lib::getHeading(sh_position_cmd_.getMsg());
    }
    catch (mrs_lib::AttitudeConverter::GetHeadingException e) {
      ROS_ERROR_THROTTLE(1.0, "[ControlTest]: exception caught: '%s'", e.what());
      cmd_heading = 0;
    }
  }

  switch (new_state) {

    case IDLE_STATE: {
      break;
    }

    case SPAWN_STATE: {

      /* //{ spawn */

      spawnUAV(_spawn_args_);

      changeState(WAIT_FOR_SPAWN);

      break;

      //}
    }

    case WAIT_FOR_SPAWN: {

      /* waiting for spawn //{ */

      ros::Duration(10.0).sleep();
      ROS_INFO("[ControlTest]: waiting for spawn");

      break;

      //}
    }

    case TAKEOFF_STATE: {

      /* //{ test the full takeoff sequence */

      // | ------------------------- motors ------------------------- |
      goal_bool.request.data = 1;
      service_client_motors_.call(goal_bool);

      wait.sleep();

      // | ------------------------- arming ------------------------- |
      goal_mavros_commandbool.request.value = 1;
      service_client_arm_.call(goal_mavros_commandbool);

      // | ------------------------ offboard ------------------------ |
      goal_mavros_set_mode.request.base_mode   = 0;
      goal_mavros_set_mode.request.custom_mode = "offboard";
      service_client_offboard_.call(goal_mavros_set_mode);

      wait.sleep();

      // | ------------------------- takeoff ------------------------ |
      service_client_takeoff_.call(goal_trigger);

      home_x_ = odom_x;
      home_y_ = odom_y;

      break;

      //}
    }

    case CHANGE_ESTIMATOR_STATE: {

      /* change the estimator //{ */

      if (active_estimator_ != _target_estimator_) {

        switchEstimator(_target_estimator_);
        active_estimator_ = _target_estimator_;
      }

      break;

      //}
    }

    case CHANGE_TRACKER_STATE: {

      /* change the tracker //{ */

      if (active_tracker_ == TRACKER_LINE) {

        switchTracker("MpcTracker");
        active_tracker_ = TRACKER_MPC;

      } else {

        if (_test_line_tracker_) {
          switchTracker("LineTracker");
          active_tracker_ = TRACKER_LINE;
        } else {
          switchTracker("MpcTracker");
          active_tracker_ = TRACKER_MPC;
        }
      }

      break;

      //}
    }

    case SET_REFERENCE_TOPIC_STATE: {

      /* //{ test set reference topic */

      ROS_INFO("[ControlTest]: calling set reference");

      goal_reference_stamped_topic.reference.position.x = genXY();
      goal_reference_stamped_topic.reference.position.y = genXY();
      goal_reference_stamped_topic.reference.position.z = genZ();
      goal_reference_stamped_topic.reference.heading    = genheading();

      des_x_       = goal_reference_stamped_topic.reference.position.x;
      des_y_       = goal_reference_stamped_topic.reference.position.y;
      des_z_       = goal_reference_stamped_topic.reference.position.z;
      des_heading_ = goal_reference_stamped_topic.reference.heading;

      try {
        publisher_reference_.publish(goal_reference_stamped_topic);
      }
      catch (...) {
        ROS_ERROR("Exception caught during publishing topic %s.", publisher_reference_.getTopic().c_str());
      }

      break;

      //}
    }

    case SET_REFERENCE_SERVICE_STATE: {

      /* //{ test set reference service */

      goal_reference_stamped_srv.request.reference.position.x = genXY();
      goal_reference_stamped_srv.request.reference.position.y = genXY();
      goal_reference_stamped_srv.request.reference.position.z = genZ();
      goal_reference_stamped_srv.request.reference.heading    = genheading();

      des_x_       = goal_reference_stamped_srv.request.reference.position.x;
      des_y_       = goal_reference_stamped_srv.request.reference.position.y;
      des_z_       = goal_reference_stamped_srv.request.reference.position.z;
      des_heading_ = goal_reference_stamped_srv.request.reference.heading;

      service_client_set_reference_.call(goal_reference_stamped_srv);

      break;

      //}
    }

    case GOTO_SERVICE_STATE: {

      /* //{ test goto service */

      goal_vec4.request.goal[0] = genXY();
      goal_vec4.request.goal[1] = genXY();
      goal_vec4.request.goal[2] = genZ();
      goal_vec4.request.goal[3] = genheading();

      des_x_       = goal_vec4.request.goal[0];
      des_y_       = goal_vec4.request.goal[1];
      des_z_       = goal_vec4.request.goal[2];
      des_heading_ = goal_vec4.request.goal[3];

      service_client_goto_.call(goal_vec4);

      break;

      //}
    }

    case GOTO_RELATIVE_SERVICE_STATE: {

      /* //{ test goto_relative service */

      goal_vec4.request.goal[0] = genXY();
      goal_vec4.request.goal[1] = genXY();
      goal_vec4.request.goal[2] = randd(_goto_relative_altitude_down_, _goto_relative_altitude_up_);
      goal_vec4.request.goal[3] = genheading();

      des_x_       = cmd_x + goal_vec4.request.goal[0];
      des_y_       = cmd_y + goal_vec4.request.goal[1];
      des_z_       = cmd_z + goal_vec4.request.goal[2];
      des_heading_ = cmd_heading + goal_vec4.request.goal[3];

      service_client_goto_relative_.call(goal_vec4);

      break;

      //}
    }

    case GOTO_ALTITUDE_SERVICE_STATE: {

      /* //{ test goto_altitude service */

      goal_vec1.request.goal = genZ();

      des_z_ = goal_vec1.request.goal;

      service_client_goto_altitude_.call(goal_vec1);

      break;

      //}
    }

    case SET_HEADING_SERVICE_STATE: {

      /* //{ test set_heading service */

      goal_vec1.request.goal = genheading();

      des_x_       = cmd_x;
      des_y_       = cmd_y;
      des_z_       = cmd_z;
      des_heading_ = goal_vec1.request.goal;

      service_client_set_heading_.call(goal_vec1);

      break;

      //}
    }

    case SET_heading_RELATIVE_SERVICE_STATE: {

      /* //{ test set_heading_relative service */

      goal_vec1.request.goal = genheading();

      des_x_       = cmd_x;
      des_y_       = cmd_y;
      des_z_       = cmd_z;
      des_heading_ = cmd_heading + goal_vec1.request.goal;

      service_client_set_heading_relative_.call(goal_vec1);

      break;

      //}
    }

    case TRAJECTORY_LOAD_STATIC_TOPIC_STATE: {

      /* //{ test trajectory_reference topic */

      switchTracker("MpcTracker");

      goal_trajectory_ = createLineTrajectory(true, false, true);

      try {
        publisher_trajectory_reference_.publish(goal_trajectory_);
      }
      catch (...) {
        ROS_ERROR("Exception caught during publishing topic %s.", publisher_trajectory_reference_.getTopic().c_str());
      }

      break;

      //}
    }

    case TRAJECTORY_GOTO_START_STATE: {

      /* //{ test goto_trajectory_start service */

      gotoTrajectoryStart();

      break;

      //}
    }

    case TRAJECTORY_START_TRACKING_STATE: {

      /* //{ test start_trajectory_tracking service */

      startTrajectoryTracking();

      break;

      //}
    }

    case TRAJECTORY_LOAD_STATIC_SERVICE_STATE: {

      /* //{ test trajectory loading using service */

      switchTracker("MpcTracker");

      goal_trajectory_ = createLineTrajectory(false, false, true);

      setTrajectorySrv(goal_trajectory_);

      break;

      //}
    }

    case TRAJECTORY_GOTO_START_SERVICE_STATE: {

      /* //{ test goto_trajectory_start service */

      gotoTrajectoryStart();

      break;

      //}
    }

    case TRAJECTORY_START_TRACKING_SERVICE_STATE: {

      /* //{ test start_trajectory_tracking service */

      startTrajectoryTracking();

      break;

      //}
    }

    case TRAJECTORY_LOAD_DYNAMIC_TOPIC_STATE: {

      /* //{ test setting trajectory via topic with fly_now */

      switchTracker("MpcTracker");

      goal_trajectory_ = createLineTrajectory(true, true, true);

      des_x_       = goal_trajectory_.points.back().position.x;
      des_y_       = goal_trajectory_.points.back().position.y;
      des_z_       = goal_trajectory_.points.back().position.z;
      des_heading_ = goal_trajectory_.points.back().heading;

      try {
        publisher_trajectory_reference_.publish(goal_trajectory_);
      }
      catch (...) {
        ROS_ERROR("Exception caught during publishing topic %s.", publisher_trajectory_reference_.getTopic().c_str());
      }

      break;

      //}
    }

    case TRAJECTORY_LOAD_DYNAMIC_SERVICE_STATE: {

      /* //{ test setting trajectory via service with fly_now */

      switchTracker("MpcTracker");

      goal_trajectory_ = createLineTrajectory(false, true, true);

      des_x_       = goal_trajectory_.points.back().position.x;
      des_y_       = goal_trajectory_.points.back().position.y;
      des_z_       = goal_trajectory_.points.back().position.z;
      des_heading_ = goal_trajectory_.points.back().heading;

      setTrajectorySrv(goal_trajectory_);

      break;

      //}
    }

    case TRAJECTORY_NOT_USE_heading_STATE: {

      /* //{ test setting trajectory with use_heading=false */

      switchTracker("MpcTracker");

      goal_trajectory_ = createLineTrajectory(true, true, false);

      des_x_       = goal_trajectory_.points.back().position.x;
      des_y_       = goal_trajectory_.points.back().position.y;
      des_z_       = goal_trajectory_.points.back().position.z;
      des_heading_ = cmd_heading;

      setTrajectorySrv(goal_trajectory_);

      break;

      //}
    }

    case TRAJECTORY_CIRCLE_LOOP: {

      /* //{ load trajectory for testing the trajectory looping */

      switchTracker("MpcTracker");

      goal_trajectory_ = createLoopingCircleTrajectory();

      looping_start_time_ = ros::Time::now();

      setTrajectorySrv(goal_trajectory_);

      break;

      //}
    }

    case TRAJECTORY_CIRCLE_PRE_PAUSE: {

      /* //{ load trajectory for testing the trajectory pausing */

      switchTracker("MpcTracker");

      goal_trajectory_ = createLoopingCircleTrajectory();

      looping_start_time_ = ros::Time::now();

      setTrajectorySrv(goal_trajectory_);

      break;

      //}
    }

    case TRAJECTORY_CIRCLE_PAUSE: {

      /* pause the trajectory tracking //{ */

      stopTrajectoryTracking();

      break;

      //}
    }

    case TRAJECTORY_CIRCLE_POST_PAUSE: {

      /* resume the trajectory tracking //{ */

      resumeTrajectoryTracking();

      des_x_       = goal_trajectory_.points.back().position.x;
      des_y_       = goal_trajectory_.points.back().position.y;
      des_z_       = goal_trajectory_.points.back().position.z;
      des_heading_ = goal_trajectory_.points.back().heading;

      break;

      //}
    }

    case TRAJECTORY_HEADLESS_LOAD_SERVICE_STATE: {

      /* //{ load trajectory for testing the headless tracking */

      switchTracker("MpcTracker");

      goal_trajectory_ = createHeadlessTrajectory();

      setTrajectorySrv(goal_trajectory_);

      break;

      //}
    }

    case TRAJECTORY_HEADLESS_FLY_TO_START_SERVICE_STATE: {

      /* //{ goto_trajectory_start */

      gotoTrajectoryStart();

      break;

      //}
    }

    case TRAJECTORY_HEADLESS_START_TRACKING_SERVICE_STATE: {

      /* start trajectory tracking //{ */

      des_x_       = goal_trajectory_.points.back().position.x;
      des_y_       = goal_trajectory_.points.back().position.y;
      des_z_       = goal_trajectory_.points.back().position.z;
      des_heading_ = goal_trajectory_.points.back().heading;

      startTrajectoryTracking();

      break;

      //}
    }

    case LAND_HOME_STATE: {

      /* //{ test land_home service */

      service_client_land_home_.call(goal_trigger);

      break;

      //}
    }

    case GOTO_ORIGIN_STATE: {

      /* //{ go to origin */

      switchTracker("MpcTracker");

      goal_reference_stamped_topic.reference.position.x = 0;
      goal_reference_stamped_topic.reference.position.y = 0;
      goal_reference_stamped_topic.reference.position.z = _min_z_;
      goal_reference_stamped_topic.reference.heading    = 0;

      des_x_       = goal_reference_stamped_topic.reference.position.x;
      des_y_       = goal_reference_stamped_topic.reference.position.y;
      des_z_       = goal_reference_stamped_topic.reference.position.z;
      des_heading_ = goal_reference_stamped_topic.reference.heading;

      try {
        publisher_reference_.publish(goal_reference_stamped_topic);
      }
      catch (...) {
        ROS_ERROR("Exception caught during publishing topic %s.", publisher_reference_.getTopic().c_str());
      }

      break;

      //}
    }

    case LAND_STATE: {

      /* //{ test land service */

      service_client_land_.call(goal_trigger);

      break;

      //}
    }

    case FINISHED_STATE: {

      /* //{ finish and end the node */

      ROS_INFO(" ");
      ROS_INFO("[ControlTest]: TEST FINISHED");

      break;

      //}
    }

    case ERROR_STATE: {

      /* //{ finish and end the node */

      ROS_INFO(" ");
      ROS_ERROR("[ControlTest]: TEST FAILED");

      break;

      //}
    }
  }

  timeout_ = ros::Time::now();
}

//}

/* //{ randd() */

double ControlTest::randd(const double from, const double to) {

  double zero_to_one = double((float)rand()) / double(RAND_MAX);

  return floor(to - from) * zero_to_one + from;
}

//}

/* //{ genheading() */

double ControlTest::genheading(void) {

  return randd(_min_heading_, _max_heading_);
}

//}

/* //{ genXY() */

double ControlTest::genXY(void) {

  return randd(_min_xy_, _max_xy_);
}

//}

/* //{ genZ() */

double ControlTest::genZ(void) {

  return randd(_min_z_, _max_z_);
}

//}

/* //{ inDesiredState() */

bool ControlTest::inDesiredState(void) {

  auto [odom_x, odom_y, odom_z] = mrs_lib::getPosition(sh_odometry_.getMsg());

  double odom_heading = 0;
  try {
    odom_heading = mrs_lib::getHeading(sh_odometry_.getMsg());
  }
  catch (mrs_lib::AttitudeConverter::GetHeadingException e) {
    ROS_ERROR_THROTTLE(1.0, "[ControlTest]: exception caught: '%s'", e.what());
    return false;
  }

  if (isStationary() && mrs_lib::geometry::dist(vec3_t(odom_x, odom_y, odom_z), vec3_t(des_x_, des_y_, des_z_)) < 0.20 &&
      fabs(radians::diff(odom_heading, des_heading_)) < 0.20) {

    ROS_WARN_THROTTLE(1.0, "[ControlTest]: the goal has been reached.");
    return true;
  }

  return false;
}

//}

/* //{ isStationary() */

bool ControlTest::isStationary(void) {

  nav_msgs::OdometryConstPtr odometry = sh_odometry_.getMsg();

  if (abs(odometry->twist.twist.linear.x) < 0.2 && abs(odometry->twist.twist.linear.y) < 0.2 && abs(odometry->twist.twist.linear.z) < 0.2 &&
      abs(odometry->twist.twist.angular.z) < 0.2) {

    ROS_WARN_THROTTLE(1.0, "[ControlTest]: the UAV is stationary");
    return true;
  }

  return false;
}

//}

/* //{ trackerReady() */

bool ControlTest::trackerReady(void) {

  mrs_msgs::TrackerStatus status = sh_control_manager_diag_.getMsg()->tracker_status;

  return status.active && status.callbacks_enabled && !status.have_goal;
}

//}

/* createLineTrajectory() //{ */

mrs_msgs::TrajectoryReference ControlTest::createLineTrajectory(const bool ascending, const bool fly_now, const bool use_heading) {

  mrs_msgs::TrajectoryReference trajectory;

  trajectory.fly_now         = fly_now;
  trajectory.header.frame_id = "";
  trajectory.header.stamp    = ros::Time::now();
  trajectory.loop            = false;
  trajectory.use_heading     = use_heading;
  trajectory.dt              = _trajectory_dt_;

  mrs_msgs::Reference trajectory_point;

  double trajectory_length = int(1.414 * ((1 / _trajectory_dt_) * fabs(_line_trajectory_p2_ - _line_trajectory_p1_)) / (_line_trajectory_speed_));

  double start_z = ascending ? _min_z_ : _max_z_;
  double end_z   = ascending ? _max_z_ : _min_z_;
  double z_step  = (end_z - start_z) / trajectory_length;

  trajectory_point.position.x = _line_trajectory_p1_;
  trajectory_point.position.y = _line_trajectory_p1_;
  trajectory_point.position.z = start_z;
  trajectory_point.heading    = 1.57;
  trajectory.points.push_back(trajectory_point);

  for (int i = 0; i < trajectory_length; i++) {

    trajectory_point.position.x += _line_trajectory_speed_ * _trajectory_dt_;
    trajectory_point.position.y += _line_trajectory_speed_ * _trajectory_dt_;
    trajectory_point.position.z += z_step;
    trajectory_point.heading = trajectory_point.heading + _line_trajectory_heading_rate_ * _trajectory_dt_;

    trajectory.points.push_back(trajectory_point);
  }

  return trajectory;
}

//}

/* createLoopingCircleTrajectory() //{ */

mrs_msgs::TrajectoryReference ControlTest::createLoopingCircleTrajectory() {

  mrs_msgs::TrajectoryReference trajectory;

  trajectory.fly_now         = true;
  trajectory.header.frame_id = "";
  trajectory.header.stamp    = ros::Time::now();
  trajectory.loop            = true;
  trajectory.use_heading     = true;
  trajectory.dt              = _trajectory_dt_;

  double angle = 0;

  mrs_msgs::Reference trajectory_point;

  trajectory_point.position.x = _looping_circle_radius_;
  trajectory_point.position.y = 0;
  trajectory_point.position.z = _min_z_;
  trajectory_point.heading    = 1.57;
  trajectory.points.push_back(trajectory_point);

  double trajectory_time   = (_looping_circle_radius_ * 2 * M_PI) / _looping_circle_speed_;
  int    trajectory_length = floor(trajectory_time * (1 / _trajectory_dt_));
  double angular_step      = (2 * M_PI) / trajectory_length;

  double last_x = trajectory_point.position.x;
  double last_y = trajectory_point.position.y;

  for (int i = 0; i < trajectory_length; i++) {

    angle += angular_step;

    trajectory_point.position.x = _looping_circle_radius_ * cos(angle);
    trajectory_point.position.y = _looping_circle_radius_ * sin(angle);
    trajectory_point.position.z = _min_z_;
    trajectory_point.heading    = atan2(trajectory_point.position.y - last_y, trajectory_point.position.x - last_x);

    last_x = trajectory_point.position.x;
    last_y = trajectory_point.position.y;

    trajectory.points.push_back(trajectory_point);
  }

  return trajectory;
}

//}

/* createHeadlessTrajectory() //{ */

mrs_msgs::TrajectoryReference ControlTest::createHeadlessTrajectory() {

  mrs_msgs::TrajectoryReference trajectory;

  trajectory.fly_now         = false;
  trajectory.header.frame_id = "";
  trajectory.header.stamp    = ros::Time::now();
  trajectory.loop            = false;
  trajectory.use_heading     = true;
  trajectory.dt              = _trajectory_dt_;

  mrs_msgs::Reference trajectory_point;

  double radius            = 5.0;
  double trajectory_length = 10 * (1.0 / _trajectory_dt_);

  trajectory_point.position.x = radius;
  trajectory_point.position.y = 0;
  trajectory_point.position.z = _min_z_;
  trajectory_point.heading    = 0;
  trajectory.points.push_back(trajectory_point);

  double angle = 0;

  for (int i = 0; i < trajectory_length; i++) {

    angle += (2 * M_PI) / trajectory_length;

    trajectory_point.position.x = radius * cos(angle);
    trajectory_point.position.y = radius * sin(angle);
    trajectory_point.position.z = _min_z_;
    trajectory_point.heading    = 0;
    trajectory.points.push_back(trajectory_point);
  }

  trajectory_length = 5 * (1.0 / _trajectory_dt_);

  for (int i = 0; i < trajectory_length; i++) {

    trajectory_point.position.x = radius;
    trajectory_point.position.y = -radius + i * (2 * radius / trajectory_length);
    trajectory_point.position.z = _min_z_;
    trajectory_point.heading    = 0;
    trajectory.points.push_back(trajectory_point);
  }

  for (int i = 0; i < trajectory_length; i++) {

    trajectory_point.position.x = radius - i * (2 * radius / trajectory_length);
    trajectory_point.position.y = radius;
    trajectory_point.position.z = _min_z_;
    trajectory_point.heading    = 0;
    trajectory.points.push_back(trajectory_point);
  }

  for (int i = 0; i < trajectory_length; i++) {

    trajectory_point.position.x = -radius;
    trajectory_point.position.y = radius - i * (2 * radius / trajectory_length);
    trajectory_point.position.z = _min_z_;
    trajectory_point.heading    = 0;
    trajectory.points.push_back(trajectory_point);
  }

  for (int i = 0; i < trajectory_length; i++) {

    trajectory_point.position.x = -radius + i * (2 * radius / trajectory_length);
    trajectory_point.position.y = -radius;
    trajectory_point.position.z = _min_z_;
    trajectory_point.heading    = 0;
    trajectory.points.push_back(trajectory_point);
  }

  return trajectory;
}

//}

// | ----------------- service client wrappers ---------------- |

/* //{ startTrajectoryTracking() */

void ControlTest::startTrajectoryTracking(void) {

  des_x_       = goal_trajectory_.points.back().position.x;
  des_y_       = goal_trajectory_.points.back().position.y;
  des_z_       = goal_trajectory_.points.back().position.z;
  des_heading_ = goal_trajectory_.points.back().heading;

  std_srvs::Trigger srv;

  bool success = service_client_start_trajectory_tracking_.call(srv);

  if (success) {

    if (!srv.response.success) {
      ROS_ERROR_STREAM("[ControlTest]: service call for starting trajectory tracking failed: " << srv.response.message);
      ros::shutdown();
    }

  } else {
    ROS_ERROR("[ControlTest]: service call for starting trajectory tracking failed");
    ros::shutdown();
  }
}

//}

/* //{ stopTrajectoryTracking() */

void ControlTest::stopTrajectoryTracking(void) {

  std_srvs::Trigger srv;

  bool success = service_client_stop_trajectory_tracking_.call(srv);

  if (success) {

    if (!srv.response.success) {
      ROS_ERROR_STREAM("[ControlTest]: service call for stopping trajectory tracking failed: " << srv.response.message);
      ros::shutdown();
    }

  } else {
    ROS_ERROR("[ControlTest]: service call for stopping trajectory tracking failed");
    ros::shutdown();
  }
}

//}

/* //{ resumeTrajectoryTracking() */

void ControlTest::resumeTrajectoryTracking(void) {

  std_srvs::Trigger srv;

  bool success = service_client_resume_trajectory_tracking_.call(srv);

  if (success) {

    if (!srv.response.success) {
      ROS_ERROR_STREAM("[ControlTest]: service call for resume trajectory tracking failed: " << srv.response.message);
      ros::shutdown();
    }

  } else {
    ROS_ERROR("[ControlTest]: service call for resume trajectory tracking failed");
    ros::shutdown();
  }
}

//}

/* //{ gotoTrajectoryStart() */

void ControlTest::gotoTrajectoryStart(void) {

  des_x_       = goal_trajectory_.points.front().position.x;
  des_y_       = goal_trajectory_.points.front().position.y;
  des_z_       = goal_trajectory_.points.front().position.z;
  des_heading_ = goal_trajectory_.points.front().heading;

  std_srvs::Trigger srv;

  bool success = service_client_goto_trajectory_start_.call(srv);

  if (success) {

    if (!srv.response.success) {
      ROS_ERROR_STREAM("[ControlTest]: service call for flying to trajectory start failed: " << srv.response.message);
      ros::shutdown();
    }

  } else {
    ROS_ERROR("[ControlTest]: service call for flying to trajectory start failed");
    ros::shutdown();
  }
}

//}

/* setTrajectorySrv() //{ */

void ControlTest::setTrajectorySrv(const mrs_msgs::TrajectoryReference trajectory) {

  mrs_msgs::TrajectoryReferenceSrv srv;
  srv.request.trajectory = trajectory;

  bool success = service_client_trajectory_reference_.call(srv);

  if (success) {

    if (!srv.response.success) {
      ROS_ERROR_STREAM("[ControlTest]: service call for setting trajectory failed: " << srv.response.message);
      ros::shutdown();
    }

  } else {
    ROS_ERROR("[ControlTest]: service call for setting trajectory failed");
    ros::shutdown();
  }
}

//}

/* //{ switchTracker() */

void ControlTest::switchTracker(const std::string tracker_name) {

  mrs_msgs::String srv;
  srv.request.value = tracker_name;

  ROS_INFO("[ControlTest]: switching to %s", srv.request.value.c_str());

  bool success = service_client_switch_tracker_.call(srv);

  if (success) {

    if (!srv.response.success) {
      ROS_ERROR_STREAM("[ControlTest]: service call for switching the tracker failed: " << srv.response.message);
      ros::shutdown();
    }

  } else {
    ROS_ERROR("[ControlTest]: service call for switching the tracker failed");
    ros::shutdown();
  }
}

//}

/* //{ spawnUAV() */

void ControlTest::spawnUAV(const std::string args) {

  mrs_msgs::String srv;
  srv.request.value = args;

  ROS_INFO("[ControlTest]: spawning a UAV with args \"%s\"", srv.request.value.c_str());

  bool success = service_client_spawn_uav_.call(srv);

  if (success) {

    if (!srv.response.success) {
      ROS_ERROR_STREAM("[ControlTest]: service call for spawning UAV failed: " << srv.response.message);
      ros::shutdown();
    }

  } else {
    ROS_ERROR("[ControlTest]: service call for spawning UAV failed");
    ros::shutdown();
  }
}

//}

/* //{ switchEstimator() */

void ControlTest::switchEstimator(const std::string estimator_name) {

  mrs_msgs::String srv;
  srv.request.value = estimator_name;

  ROS_INFO("[ControlTest]: switching estimator to %s", srv.request.value.c_str());

  bool success = service_client_switch_estimator_.call(srv);

  if (success) {

    if (!srv.response.success) {
      ROS_ERROR_STREAM("[ControlTest]: service call for switching the estimator failed: " << srv.response.message);
      ros::shutdown();
    }

  } else {
    ROS_ERROR("[ControlTest]: service call for switching the estimator failed");
    ros::shutdown();
  }
}

//}

/* finished() //{ */

bool ControlTest::finished(void) {

#ifdef ROSTEST
  if (current_state_ == FINISHED_STATE || current_state_ == ERROR_STATE) {
#else
  if (current_state_ == FINISHED_STATE || current_state_ == ERROR_STATE) {
#endif
    return true;
  } else {
    return false;
  }
}

//}

/* result() //{ */

bool ControlTest::result(void) {

#ifdef ROSTEST
  if (current_state_ == FINISHED_STATE) {
#else
  if (current_state_ == FINISHED_STATE) {
#endif
    return true;
  } else {
    return false;
  }
}

//}

std::shared_ptr<ControlTest> control_test_;

#ifdef ROSTEST
TEST(TESTSuite, controlTest) {
#else
bool test(void) {
#endif

  bool result = false;

  while (ros::ok()) {

    ros::spinOnce();

    ros::Duration(0.01).sleep();

    bool finished = control_test_->finished();

    if (finished) {

      result = control_test_->result();

      break;
    }
  }

#ifdef ROSTEST
  EXPECT_TRUE(result);
#else
  return true;
#endif
}

int main(int argc, char** argv) {

  ros::init(argc, argv, "ControlTest");

  control_test_.reset(new ControlTest);

#ifdef ROSTEST
  testing::InitGoogleTest(&argc, argv);
#endif

#ifdef ROSTEST
  return RUN_ALL_TESTS();
#else
  return test();
#endif
}
