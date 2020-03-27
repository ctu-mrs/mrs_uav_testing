/* includes //{ */

#include <ros/ros.h>
#include <nodelet/nodelet.h>

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

#include <nav_msgs/Odometry.h>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>

#include <mutex>

#include <mrs_lib/mutex.h>

#include <tf/transform_datatypes.h>

#include <mrs_lib/ParamLoader.h>

//}

namespace mrs_testing
{

/* //{ state machine states */

#define TRACKER_LINE 0
#define TRACKER_MPC 1

// state machine
typedef enum
{
  IDLE_STATE,
  TAKEOFF_STATE,
  CHANGE_TRACKER_STATE,
  SET_REFERENCE_TOPIC_STATE,
  SET_REFERENCE_SERVICE_STATE,
  GOTO_SERVICE_STATE,
  GOTO_RELATIVE_SERVICE_STATE,
  GOTO_ALTITUDE_SERVICE_STATE,
  SET_YAW_SERVICE_STATE,
  SET_YAW_RELATIVE_SERVICE_STATE,
  TRAJECTORY_LOAD_STATIC_TOPIC_STATE,
  TRAJECTORY_GOTO_START_STATE,
  TRAJECTORY_START_TRACKING_STATE,
  TRAJECTORY_LOAD_STATIC_SERVICE_STATE,
  TRAJECTORY_GOTO_START_SERVICE_STATE,
  TRAJECTORY_START_TRACKING_SERVICE_STATE,
  TRAJECTORY_LOAD_DYNAMIC_TOPIC_STATE,
  TRAJECTORY_LOAD_DYNAMIC_SERVICE_STATE,
  TRAJECTORY_NOT_USE_YAW_STATE,
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
} ControlState_t;

const char *state_names[30] = {
    "IDLE_STATE",
    "TAKEOFF_STATE",
    "CHANGE_TRACKER_STATE",
    "SET_REFERENCE_TOPIC_STATE",
    "SET_REFERENCE_SERVICE_STATE",
    "GOTO_SERVICE_STATE",
    "GOTO_RELATIVE_SERVICE_STATE",
    "GOTO_ALTITUDE_SERVICE_STATE",
    "SET_YAW_SERVICE_STATE",
    "SET_YAW_RELATIVE_SERVICE_STATE",
    "TRAJECTORY_LOAD_STATIC_TOPIC_STATE",
    "TRAJECTORY_GOTO_START_STATE",
    "TRAJECTORY_START_TRACKING_STATE",
    "TRAJECTORY_LOAD_STATIC_SERVICE_STATE",
    "TRAJECTORY_GOTO_START_SERVICE_STATE",
    "TRAJECTORY_START_TRACKING_SERVICE_STATE",
    "TRAJECTORY_LOAD_DYNAMIC_TOPIC_STATE",
    "TRAJECTORY_LOAD_DYNAMIC_SERVICE_STATE",
    "TRAJECTORY_NOT_USE_YAW_STATE",
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
};

//}

/* //{ class ControlTest */

class ControlTest : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  ros::NodeHandle nh_;
  bool            is_initialized_ = false;

  // | ------------------------ odometry ------------------------ |

  ros::Subscriber subscriber_odometry_;
  bool            got_odometry_ = false;
  std::mutex      mutex_odometry_;

  nav_msgs::Odometry odometry_;

  double odometry_yaw_;
  double odometry_pitch_;
  double odometry_roll_;
  double odometry_x_;
  double odometry_y_;
  double odometry_z_;

  void callbackOdometry(const nav_msgs::OdometryConstPtr &msg);

  // | -------------------- position command -------------------- |

  ros::Subscriber subscriber_position_command_;
  bool            got_position_command_ = false;
  std::mutex      mutex_position_command_;

  mrs_msgs::PositionCommand position_command_;

  double cmd_x_;
  double cmd_y_;
  double cmd_z_;
  double cmd_yaw_;

  void callbackPositionCommand(const mrs_msgs::PositionCommandConstPtr &msg);

  // | --------------- control manager diagnostics -------------- |

  ros::Subscriber subscriber_control_manager_diagnostics_;
  bool            got_control_manager_diagnostics_ = false;
  std::mutex      mutex_control_manager_diagnostics_;

  mrs_msgs::ControlManagerDiagnostics control_manager_diagnostics_;

  void callbackControlManagerDiagnostics(const mrs_msgs::ControlManagerDiagnosticsConstPtr &msg);

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
  ros::ServiceClient service_client_set_yaw_;
  ros::ServiceClient service_client_set_yaw_relative_;

  // trajectory tracking
  ros::ServiceClient service_client_trajectory_reference_;
  ros::ServiceClient service_client_goto_trajectory_start_;
  ros::ServiceClient service_client_start_trajectory_tracking_;
  ros::ServiceClient service_client_resume_trajectory_tracking_;
  ros::ServiceClient service_client_stop_trajectory_tracking_;

  // | ----------------------- main timer ----------------------- |

  ros::Timer timer_main_;
  void       timerMain(const ros::TimerEvent &event);

  // | ---------------------- state machine --------------------- |

  ControlState_t current_state_  = IDLE_STATE;
  ControlState_t previous_state_ = IDLE_STATE;
  void           changeState(const ControlState_t new_state);

  // | ------------------------- params ------------------------- |

  double _max_xy_;
  double _min_xy_;
  double _max_z_;
  double _min_z_;
  double _max_yaw_;
  double _min_yaw_;

  double _trajectory_dt_;

  double _line_trajectory_p1_;
  double _line_trajectory_p2_;
  double _line_trajectory_speed_;
  double _line_trajectory_yaw_rate_;

  double _looping_circle_speed_;
  double _looping_circle_radius_;
  double _looping_circle_duration_;

  double _goto_relative_altitude_down_;
  double _goto_relative_altitude_up_;

  bool _test_line_tracker_ = false;
  bool _test_mpc_gotos_    = true;

  // | ------------------------ routines ------------------------ |

  double dist3d(const double x1, const double x2, const double y1, const double y2, const double z1, const double z2);
  double dist2d(const double x1, const double x2, const double y1, const double y2);
  double randd(const double from, const double to);
  double genYaw(void);
  double genXY(void);
  double genZ(void);
  double sanitizeYaw(const double yaw_in);
  double angleDist(const double in1, const double in2);
  bool   inDesiredState(void);
  bool   isStationary(void);
  bool   trackerReady(void);
  void   switchTracker(const std::string tracker_name);
  void   startTrajectoryTracking(void);
  void   stopTrajectoryTracking(void);
  void   resumeTrajectoryTracking(void);
  void   gotoTrajectoryStart(void);
  void   setTrajectorySrv(const mrs_msgs::TrajectoryReference trajectory);

  mrs_msgs::TrajectoryReference createLoopingCircleTrajectory();
  mrs_msgs::TrajectoryReference createLineTrajectory(const bool ascending, const bool fly_now, const bool use_yaw);
  mrs_msgs::TrajectoryReference createHeadlessTrajectory();

  // | ----------------------- global vars ---------------------- |

  int active_tracker_ = -1;
  int takeoff_num_    = 0;

  double des_x_;
  double des_y_;
  double des_z_;
  double des_yaw_;

  double home_x_;
  double home_y_;

  ros::Time timeout_;

  ros::Time looping_start_time_;

  mrs_msgs::TrajectoryReference goal_trajectory_;
};

//}

/* //{ onInit() */

void ControlTest::onInit() {

  ros::NodeHandle nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  ros::Time::waitForValid();

  timeout_ = ros::Time::now();

  srand(static_cast<unsigned int>(ros::Time::now().toSec()));

  // --------------------------------------------------------------
  // |                           params                           |
  // --------------------------------------------------------------

  mrs_lib::ParamLoader param_loader(nh_, "ControlTest");

  param_loader.load_param("max_xy", _max_xy_);
  param_loader.load_param("min_xy", _min_xy_);
  param_loader.load_param("max_z", _max_z_);
  param_loader.load_param("min_z", _min_z_);
  param_loader.load_param("max_yaw", _max_yaw_);
  param_loader.load_param("min_yaw", _min_yaw_);
  param_loader.load_param("goto_relative_altitude_down", _goto_relative_altitude_down_);
  param_loader.load_param("goto_relative_altitude_up", _goto_relative_altitude_up_);

  param_loader.load_param("trajectory_dt", _trajectory_dt_);

  param_loader.load_param("line_trajectory/p1", _line_trajectory_p1_);
  param_loader.load_param("line_trajectory/p2", _line_trajectory_p2_);
  param_loader.load_param("line_trajectory/speed", _line_trajectory_speed_);
  param_loader.load_param("line_trajectory/yaw_rate", _line_trajectory_yaw_rate_);

  param_loader.load_param("looping_circle_trajectory/radius", _looping_circle_radius_);
  param_loader.load_param("looping_circle_trajectory/speed", _looping_circle_speed_);
  param_loader.load_param("looping_circle_trajectory/duration", _looping_circle_duration_);

  param_loader.load_param("test_line_tracker", _test_line_tracker_);
  param_loader.load_param("test_mpc_gotos", _test_mpc_gotos_);

  if (!param_loader.loaded_successfully()) {
    ros::shutdown();
  }

  // | ------------- calculate stuff from the params ------------ |

  _line_trajectory_speed_ = _line_trajectory_speed_ / 1.414;

  // | ----------------------- subscribers ---------------------- |

  subscriber_odometry_         = nh_.subscribe("odometry_in", 1, &ControlTest::callbackOdometry, this, ros::TransportHints().tcpNoDelay());
  subscriber_position_command_ = nh_.subscribe("position_command_in", 1, &ControlTest::callbackPositionCommand, this, ros::TransportHints().tcpNoDelay());
  subscriber_control_manager_diagnostics_ =
      nh_.subscribe("control_manager_diagnostics_in", 1, &ControlTest::callbackControlManagerDiagnostics, this, ros::TransportHints().tcpNoDelay());

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
  service_client_set_yaw_                    = nh_.serviceClient<mrs_msgs::Vec1>("set_yaw_out");
  service_client_set_yaw_relative_           = nh_.serviceClient<mrs_msgs::Vec1>("set_yaw_relative_out");
  service_client_trajectory_reference_       = nh_.serviceClient<mrs_msgs::TrajectoryReferenceSrv>("trajectory_reference_out");
  service_client_goto_trajectory_start_      = nh_.serviceClient<std_srvs::Trigger>("goto_trajectory_start_out");
  service_client_start_trajectory_tracking_  = nh_.serviceClient<std_srvs::Trigger>("start_trajectory_tracking_out");
  service_client_stop_trajectory_tracking_   = nh_.serviceClient<std_srvs::Trigger>("stop_trajectory_tracking_out");
  service_client_resume_trajectory_tracking_ = nh_.serviceClient<std_srvs::Trigger>("resume_trajectory_tracking_out");

  // | ------------------------- timers ------------------------- |

  timer_main_ = nh_.createTimer(ros::Rate(10), &ControlTest::timerMain, this);

  // | ---------------------- finish inint ---------------------- |

  is_initialized_ = true;

  ROS_INFO("[ControlTest]: initialized");
}

//}

// --------------------------------------------------------------
// |                          callbacks                         |
// --------------------------------------------------------------

/* //{ callbackOdometry() */

void ControlTest::callbackOdometry(const nav_msgs::OdometryConstPtr &msg) {

  if (!is_initialized_)
    return;

  got_odometry_ = true;

  {
    std::scoped_lock lock(mutex_odometry_);

    odometry_   = *msg;
    odometry_x_ = msg->pose.pose.position.x;
    odometry_y_ = msg->pose.pose.position.y;
    odometry_z_ = msg->pose.pose.position.z;

    // calculate the euler angles
    tf::Quaternion quaternion_odometry;
    quaternionMsgToTF(msg->pose.pose.orientation, quaternion_odometry);
    tf::Matrix3x3 m(quaternion_odometry);
    m.getRPY(odometry_roll_, odometry_pitch_, odometry_yaw_);
  }
}

//}

/* //{ callbackPositionCommand() */

void ControlTest::callbackPositionCommand(const mrs_msgs::PositionCommandConstPtr &msg) {

  if (!is_initialized_)
    return;

  got_position_command_ = true;

  {
    std::scoped_lock lock(mutex_position_command_);

    position_command_ = *msg;

    cmd_x_   = msg->position.x;
    cmd_y_   = msg->position.y;
    cmd_z_   = msg->position.z;
    cmd_yaw_ = msg->yaw;
  }
}

//}

/* //{ callbackControlManagerDiagnostics() */

void ControlTest::callbackControlManagerDiagnostics(const mrs_msgs::ControlManagerDiagnosticsConstPtr &msg) {

  if (!is_initialized_)
    return;

  got_control_manager_diagnostics_ = true;

  {
    std::scoped_lock lock(mutex_control_manager_diagnostics_);
    control_manager_diagnostics_ = *msg;
  }
}

//}

// --------------------------------------------------------------
// |                           timers                           |
// --------------------------------------------------------------

/* //{ timerMain() */

void ControlTest::timerMain([[maybe_unused]] const ros::TimerEvent &event) {

  if (!is_initialized_)
    return;

  if (!got_odometry_) {
    ROS_INFO_THROTTLE(1.0, "[ControlTest]: waiting for data, got_odometry: %s", got_odometry_ ? "YES" : "NO");
    return;
  }

  auto [cmd_x, cmd_y, cmd_z, cmd_yaw]                     = mrs_lib::get_mutexed(mutex_position_command_, cmd_x_, cmd_y_, cmd_z_, cmd_yaw_);
  auto [odometry_x, odometry_y, odometry_z, odometry_yaw] = mrs_lib::get_mutexed(mutex_odometry_, odometry_x_, odometry_y_, odometry_z_, odometry_yaw_);

  ROS_INFO_THROTTLE(5.0, " ");
  ROS_INFO_THROTTLE(5.0, "[ControlTest]: desired: %.2f %.2f %.2f %.2f", des_x_, des_y_, des_z_, des_yaw_);
  ROS_INFO_THROTTLE(5.0, "[ControlTest]: cmd: %.2f %.2f %.2f %.2f", cmd_x, cmd_y, cmd_z, sanitizeYaw(cmd_yaw));
  ROS_INFO_THROTTLE(5.0, "[ControlTest]: odom: %.2f %.2f %.2f %.2f", odometry_x, odometry_y, odometry_z, sanitizeYaw(odometry_yaw));
  ROS_INFO_THROTTLE(5.0, " ");

  if ((ros::Time::now() - timeout_).toSec() > 180.0) {
    ROS_ERROR("[ControlTest]: TIMEOUT, TEST FAILED!!");
    ros::shutdown();
  }

  switch (current_state_) {

    case IDLE_STATE: {

      changeState(ControlState_t(int(current_state_) + 1));
      break;
    }

    case TAKEOFF_STATE: {

      std::scoped_lock lock(mutex_control_manager_diagnostics_);

      if (control_manager_diagnostics_.active_tracker == "MpcTracker" && trackerReady()) {

        ROS_INFO("[ControlTest]: takeoff_num=%d", takeoff_num_);

        takeoff_num_++;

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

    case SET_YAW_SERVICE_STATE: {

      if (inDesiredState() && trackerReady()) {
        changeState(ControlState_t(int(current_state_) + 1));
      }

      break;
    }

    case SET_YAW_RELATIVE_SERVICE_STATE: {

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

    case TRAJECTORY_NOT_USE_YAW_STATE: {

      if (inDesiredState() && trackerReady()) {
        changeState(ControlState_t(int(current_state_) + 1));
      }

      break;
    }

    case TRAJECTORY_CIRCLE_LOOP: {

      if ((ros::Time::now() - looping_start_time_).toSec() > _looping_circle_duration_) {
        if (dist3d(odometry_x, cmd_x, odometry_y, cmd_y, odometry_z, cmd_z) < 2.0) {
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

      if ((ros::Time::now() - looping_start_time_).toSec() > 20.0) {
        if (dist3d(odometry_x, cmd_x, odometry_y, cmd_y, odometry_z, cmd_z) < 2.0) {
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

      std::scoped_lock lock(mutex_control_manager_diagnostics_);

      if (control_manager_diagnostics_.active_tracker == "NullTracker" && dist2d(home_x_, odometry_x, home_y_, odometry_y) < 1.0) {
        ROS_INFO("[ControlTest]: %s", control_manager_diagnostics_.active_tracker.c_str());
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

      std::scoped_lock lock(mutex_control_manager_diagnostics_);

      if (control_manager_diagnostics_.active_tracker == "NullTracker" && dist2d(des_x_, odometry_x, des_y_, odometry_y) < 1.0) {
        changeState(ControlState_t(int(current_state_) + 1));
      }

      break;
    }

    case FINISHED_STATE: {

      break;
    }
  }
}

//}

// --------------------------------------------------------------
// |                        other methods                       |
// --------------------------------------------------------------

/* //{ changeState() */

void ControlTest::changeState(const ControlState_t new_state) {

  auto [odometry_x, odometry_y]       = mrs_lib::get_mutexed(mutex_odometry_, odometry_x_, odometry_y_);
  auto [cmd_x, cmd_y, cmd_z, cmd_yaw] = mrs_lib::get_mutexed(mutex_position_command_, cmd_x_, cmd_y_, cmd_z_, cmd_yaw_);

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

  switch (new_state) {

    case IDLE_STATE: {
      break;
    }

    case TAKEOFF_STATE: {

      /* //{ test the full takeoff sequence */

      // | ------------------------- motors ------------------------- |
      goal_bool.request.data = 1;
      service_client_motors_.call(goal_bool);

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

      home_x_ = odometry_x;
      home_y_ = odometry_y;

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
      goal_reference_stamped_topic.reference.yaw        = sanitizeYaw(genYaw());

      des_x_   = goal_reference_stamped_topic.reference.position.x;
      des_y_   = goal_reference_stamped_topic.reference.position.y;
      des_z_   = goal_reference_stamped_topic.reference.position.z;
      des_yaw_ = goal_reference_stamped_topic.reference.yaw;

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
      goal_reference_stamped_srv.request.reference.yaw        = sanitizeYaw(genYaw());

      des_x_   = goal_reference_stamped_srv.request.reference.position.x;
      des_y_   = goal_reference_stamped_srv.request.reference.position.y;
      des_z_   = goal_reference_stamped_srv.request.reference.position.z;
      des_yaw_ = goal_reference_stamped_srv.request.reference.yaw;

      service_client_set_reference_.call(goal_reference_stamped_srv);

      break;

      //}
    }

    case GOTO_SERVICE_STATE: {

      /* //{ test goto service */

      goal_vec4.request.goal[0] = genXY();
      goal_vec4.request.goal[1] = genXY();
      goal_vec4.request.goal[2] = genZ();
      goal_vec4.request.goal[3] = sanitizeYaw(genYaw());

      des_x_   = goal_vec4.request.goal[0];
      des_y_   = goal_vec4.request.goal[1];
      des_z_   = goal_vec4.request.goal[2];
      des_yaw_ = goal_vec4.request.goal[3];

      service_client_goto_.call(goal_vec4);

      break;

      //}
    }

    case GOTO_RELATIVE_SERVICE_STATE: {

      /* //{ test goto_relative service */

      goal_vec4.request.goal[0] = genXY();
      goal_vec4.request.goal[1] = genXY();
      goal_vec4.request.goal[2] = randd(_goto_relative_altitude_down_, _goto_relative_altitude_up_);
      goal_vec4.request.goal[3] = sanitizeYaw(genYaw());

      {
        std::scoped_lock lock(mutex_position_command_);

        des_x_   = cmd_x + goal_vec4.request.goal[0];
        des_y_   = cmd_y + goal_vec4.request.goal[1];
        des_z_   = cmd_z + goal_vec4.request.goal[2];
        des_yaw_ = sanitizeYaw(cmd_yaw + goal_vec4.request.goal[3]);
      }

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

    case SET_YAW_SERVICE_STATE: {

      /* //{ test set_yaw service */

      goal_vec1.request.goal = sanitizeYaw(genYaw());

      des_x_   = cmd_x_;
      des_y_   = cmd_y_;
      des_z_   = cmd_z_;
      des_yaw_ = goal_vec1.request.goal;

      service_client_set_yaw_.call(goal_vec1);

      break;

      //}
    }

    case SET_YAW_RELATIVE_SERVICE_STATE: {

      /* //{ test set_yaw_relative service */

      goal_vec1.request.goal = sanitizeYaw(genYaw());

      {
        std::scoped_lock lock(mutex_position_command_);

        des_x_   = cmd_x_;
        des_y_   = cmd_y_;
        des_z_   = cmd_z_;
        des_yaw_ = sanitizeYaw(cmd_yaw + goal_vec1.request.goal);
      }

      service_client_set_yaw_relative_.call(goal_vec1);

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

      des_x_   = goal_trajectory_.points.back().position.x;
      des_y_   = goal_trajectory_.points.back().position.y;
      des_z_   = goal_trajectory_.points.back().position.z;
      des_yaw_ = goal_trajectory_.points.back().yaw;

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

      des_x_   = goal_trajectory_.points.back().position.x;
      des_y_   = goal_trajectory_.points.back().position.y;
      des_z_   = goal_trajectory_.points.back().position.z;
      des_yaw_ = goal_trajectory_.points.back().yaw;

      setTrajectorySrv(goal_trajectory_);

      break;

      //}
    }

    case TRAJECTORY_NOT_USE_YAW_STATE: {

      /* //{ test setting trajectory with use_yaw=false */

      switchTracker("MpcTracker");

      goal_trajectory_ = createLineTrajectory(true, true, false);

      des_x_   = goal_trajectory_.points.back().position.x;
      des_y_   = goal_trajectory_.points.back().position.y;
      des_z_   = goal_trajectory_.points.back().position.z;
      des_yaw_ = cmd_yaw_;

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

      des_x_   = goal_trajectory_.points.back().position.x;
      des_y_   = goal_trajectory_.points.back().position.y;
      des_z_   = goal_trajectory_.points.back().position.z;
      des_yaw_ = goal_trajectory_.points.back().yaw;

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

      des_x_   = goal_trajectory_.points.back().position.x;
      des_y_   = goal_trajectory_.points.back().position.y;
      des_z_   = goal_trajectory_.points.back().position.z;
      des_yaw_ = goal_trajectory_.points.back().yaw;

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
      goal_reference_stamped_topic.reference.position.z = 3;
      goal_reference_stamped_topic.reference.yaw        = 0;

      des_x_   = goal_reference_stamped_topic.reference.position.x;
      des_y_   = goal_reference_stamped_topic.reference.position.y;
      des_z_   = goal_reference_stamped_topic.reference.position.z;
      des_yaw_ = goal_reference_stamped_topic.reference.yaw;

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
      ros::shutdown();

      break;

      //}
    }
  }

  wait.sleep();

  timeout_ = ros::Time::now();
}

//}

/* //{ randd() */

double ControlTest::randd(const double from, const double to) {

  double zero_to_one = double((float)rand()) / double(RAND_MAX);

  return floor(to - from) * zero_to_one + from;
}

//}

/* //{ genYaw() */

double ControlTest::genYaw(void) {

  return randd(_min_yaw_, _max_yaw_);
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

/* //{ dist3d() */

double ControlTest::dist3d(const double x1, const double x2, const double y1, const double y2, const double z1, const double z2) {

  return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2) + pow(z1 - z2, 2));
}

//}

/* //{ dist2d() */

double ControlTest::dist2d(const double x1, const double x2, const double y1, const double y2) {

  return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
}

//}

/* //{ sanitizeYaw() */

double ControlTest::sanitizeYaw(const double yaw_in) {

  double yaw_out = yaw_in;

  // if desired yaw_out is grater then 2*M_PI mod it
  if (fabs(yaw_out) > 2 * M_PI) {
    yaw_out = fmod(yaw_out, 2 * M_PI);
  }

  // move it to its place
  if (yaw_out > M_PI) {
    yaw_out -= 2 * M_PI;
  } else if (yaw_out < -M_PI) {
    yaw_out += 2 * M_PI;
  }

  return yaw_out;
}

//}

/* angleDist() //{ */

double ControlTest::angleDist(const double in1, const double in2) {

  double sanitized_difference = fabs(sanitizeYaw(in1) - sanitizeYaw(in2));

  if (sanitized_difference > M_PI) {
    sanitized_difference = 2 * M_PI - sanitized_difference;
  }

  return fabs(sanitized_difference);
}

//}

/* //{ inDesiredState() */

bool ControlTest::inDesiredState(void) {

  auto [odometry_x, odometry_y, odometry_z, odometry_yaw] = mrs_lib::get_mutexed(mutex_odometry_, odometry_x_, odometry_y_, odometry_z_, odometry_yaw_);

  if (isStationary() && dist3d(odometry_x, des_x_, odometry_y, des_y_, odometry_z, des_z_) < 0.10 && angleDist(odometry_yaw, sanitizeYaw(des_yaw_)) < 0.10) {

    ROS_WARN("[ControlTest]: the goal has been reached.");
    return true;
  }

  return false;
}

//}

/* //{ isStationary() */

bool ControlTest::isStationary(void) {

  auto odometry = mrs_lib::get_mutexed(mutex_odometry_, odometry_);

  if (abs(odometry.twist.twist.linear.x) < 0.1 && abs(odometry.twist.twist.linear.y) < 0.1 && abs(odometry.twist.twist.linear.z) < 0.1 &&
      abs(odometry.twist.twist.angular.z) < 0.1) {

    ROS_WARN_THROTTLE(1.0, "[ControlTest]: the UAV is statinary");
    return true;
  }

  return false;
}

//}

/* //{ trackerReady() */

bool ControlTest::trackerReady(void) {

  mrs_msgs::TrackerStatus status = control_manager_diagnostics_.tracker_status;

  return status.active && status.callbacks_enabled && !status.moving_reference;
}

//}

/* createLineTrajectory() //{ */

mrs_msgs::TrajectoryReference ControlTest::createLineTrajectory(const bool ascending, const bool fly_now, const bool use_yaw) {

  mrs_msgs::TrajectoryReference trajectory;

  trajectory.fly_now         = fly_now;
  trajectory.header.frame_id = "";
  trajectory.header.stamp    = ros::Time::now();
  trajectory.loop            = false;
  trajectory.use_yaw         = use_yaw;
  trajectory.dt              = _trajectory_dt_;

  mrs_msgs::Reference trajectory_point;

  double trajectory_length = int(1.414 * ((1 / _trajectory_dt_) * fabs(_line_trajectory_p2_ - _line_trajectory_p1_)) / (_line_trajectory_speed_));

  double start_z = ascending ? _min_z_ : _max_z_;
  double end_z   = ascending ? _max_z_ : _min_z_;
  double z_step  = (end_z - start_z) / trajectory_length;

  trajectory_point.position.x = _line_trajectory_p1_;
  trajectory_point.position.y = _line_trajectory_p1_;
  trajectory_point.position.z = start_z;
  trajectory_point.yaw        = 1.57;
  trajectory.points.push_back(trajectory_point);

  for (int i = 0; i < trajectory_length; i++) {

    trajectory_point.position.x += _line_trajectory_speed_ * _trajectory_dt_;
    trajectory_point.position.y += _line_trajectory_speed_ * _trajectory_dt_;
    trajectory_point.position.z += z_step;
    trajectory_point.yaw = sanitizeYaw(trajectory_point.yaw + _line_trajectory_yaw_rate_ * _trajectory_dt_);

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
  trajectory.use_yaw         = true;
  trajectory.dt              = _trajectory_dt_;

  double angle = 0;

  mrs_msgs::Reference trajectory_point;

  trajectory_point.position.x = _looping_circle_radius_;
  trajectory_point.position.y = 0;
  trajectory_point.position.z = _min_z_;
  trajectory_point.yaw        = 1.57;
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
    trajectory_point.yaw        = atan2(trajectory_point.position.y - last_y, trajectory_point.position.x - last_x);

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
  trajectory.use_yaw         = true;
  trajectory.dt              = _trajectory_dt_;

  mrs_msgs::Reference trajectory_point;

  double radius            = 5.0;
  double trajectory_length = 10 * (1.0 / _trajectory_dt_);

  trajectory_point.position.x = radius;
  trajectory_point.position.y = 0;
  trajectory_point.position.z = _min_z_;
  trajectory_point.yaw        = 0;
  trajectory.points.push_back(trajectory_point);

  double angle = 0;

  for (int i = 0; i < trajectory_length; i++) {

    angle += (2 * M_PI) / trajectory_length;

    trajectory_point.position.x = radius * cos(angle);
    trajectory_point.position.y = radius * sin(angle);
    trajectory_point.position.z = _min_z_;
    trajectory_point.yaw        = 0;
    trajectory.points.push_back(trajectory_point);
  }

  trajectory_length = 5 * (1.0 / _trajectory_dt_);

  for (int i = 0; i < trajectory_length; i++) {

    trajectory_point.position.x = radius;
    trajectory_point.position.y = -radius + i * (2 * radius / trajectory_length);
    trajectory_point.position.z = _min_z_;
    trajectory_point.yaw        = 0;
    trajectory.points.push_back(trajectory_point);
  }

  for (int i = 0; i < trajectory_length; i++) {

    trajectory_point.position.x = radius - i * (2 * radius / trajectory_length);
    trajectory_point.position.y = radius;
    trajectory_point.position.z = _min_z_;
    trajectory_point.yaw        = 0;
    trajectory.points.push_back(trajectory_point);
  }

  for (int i = 0; i < trajectory_length; i++) {

    trajectory_point.position.x = -radius;
    trajectory_point.position.y = radius - i * (2 * radius / trajectory_length);
    trajectory_point.position.z = _min_z_;
    trajectory_point.yaw        = 0;
    trajectory.points.push_back(trajectory_point);
  }

  for (int i = 0; i < trajectory_length; i++) {

    trajectory_point.position.x = -radius + i * (2 * radius / trajectory_length);
    trajectory_point.position.y = -radius;
    trajectory_point.position.z = _min_z_;
    trajectory_point.yaw        = 0;
    trajectory.points.push_back(trajectory_point);
  }

  return trajectory;
}

//}

// | ----------------- service client wrappers ---------------- |

/* //{ startTrajectoryTracking() */

void ControlTest::startTrajectoryTracking(void) {

  des_x_   = goal_trajectory_.points.back().position.x;
  des_y_   = goal_trajectory_.points.back().position.y;
  des_z_   = goal_trajectory_.points.back().position.z;
  des_yaw_ = goal_trajectory_.points.back().yaw;

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

  des_x_   = goal_trajectory_.points.front().position.x;
  des_y_   = goal_trajectory_.points.front().position.y;
  des_z_   = goal_trajectory_.points.front().position.z;
  des_yaw_ = goal_trajectory_.points.front().yaw;

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

}  // namespace mrs_testing

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_testing::ControlTest, nodelet::Nodelet)
