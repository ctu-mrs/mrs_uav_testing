/* includes //{ */

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>

#include <std_msgs/Float64.h>

#include <mrs_msgs/ReferenceStamped.h>
#include <mrs_msgs/TrackerPoint.h>
#include <mrs_msgs/TrackerTrajectory.h>
#include <mrs_msgs/TrackerTrajectorySrv.h>
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
  TRAJECTORY_FLY_TO_START_TOPIC_STATE,
  TRAJECTORY_START_FOLLOWING_TOPIC_STATE,
  TRAJECTORY_LOAD_STATIC_SERVICE_STATE,
  TRAJECTORY_FLY_TO_START_SERVICE_STATE,
  TRAJECTORY_START_FOLLOWING_SERVICE_STATE,
  TRAJECTORY_LOAD_DYNAMIC_TOPIC_STATE,
  TRAJECTORY_LOAD_DYNAMIC_SERVICE_STATE,
  TRAJECTORY_CIRCLE_LOOP,
  TRAJECTORY_HEADLESS_LOAD_SERVICE_STATE,
  TRAJECTORY_HEADLESS_FLY_TO_START_SERVICE_STATE,
  TRAJECTORY_HEADLESS_START_FOLLOWING_SERVICE_STATE,
  LAND_HOME_STATE,
  GOTO_ORIGIN_STATE,
  LAND_STATE,
  FINISHED_STATE,
} ControlState_t;

const char *state_names[26] = {
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
    "TRAJECTORY_FLY_TO_START_TOPIC_STATE",
    "TRAJECTORY_START_FOLLOWING_TOPIC_STATE",
    "TRAJECTORY_LOAD_STATIC_SERVICE_STATE",
    "TRAJECTORY_FLY_TO_START_SERVICE_STATE",
    "TRAJECTORY_START_FOLLOWING_SERVICE_STATE",
    "TRAJECTORY_LOAD_DYNAMIC_TOPIC_STATE",
    "TRAJECTORY_LOAD_DYNAMIC_SERVICE_STATE",
    "TRAJECTORY_CIRCLE_LOOP",
    "TRAJECTORY_HEADLESS_LOAD_SERVICE_STATE",
    "TRAJECTORY_HEADLESS_FLY_TO_START_SERVICE_STATE",
    "TRAJECTORY_HEADLESS_START_FOLLOWING_SERVICE_STATE",
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

private:
  ros::Subscriber    subscriber_odometry_;
  bool               got_odometry_ = false;
  nav_msgs::Odometry odometry_;
  std::mutex         mutex_odometry_;
  double             odometry_yaw_, odometry_pitch_, odometry_roll_;
  double             odometry_x_, odometry_y_, odometry_z_;

private:
  ros::Subscriber           subscriber_position_command_;
  bool                      got_position_command_ = false;
  mrs_msgs::PositionCommand position_command_;
  double                    cmd_x_, cmd_y_, cmd_z_, cmd_yaw_;
  std::mutex                mutex_position_command_;

private:
  ros::Subscriber                     subscriber_control_manager_diagnostics_;
  bool                                got_control_manager_diagnostics_ = false;
  mrs_msgs::ControlManagerDiagnostics control_manager_diagnostics_;
  std::mutex                          mutex_control_manager_diagnostics_;

private:
  ros::Publisher publisher_reference_;
  ros::Publisher publisher_set_trajectory_;

private:
  ros::ServiceClient service_client_switch_tracker_;
  ros::ServiceClient service_client_arm_;
  ros::ServiceClient service_client_offboard_;
  ros::ServiceClient service_client_takeoff_;
  ros::ServiceClient service_client_land_;
  ros::ServiceClient service_client_land_home_;
  ros::ServiceClient service_client_motors_;

private:
  ros::ServiceClient service_client_set_reference_;
  ros::ServiceClient service_client_goto_;
  ros::ServiceClient service_client_goto_relative_;
  ros::ServiceClient service_client_goto_altitude_;
  ros::ServiceClient service_client_set_yaw_;
  ros::ServiceClient service_client_set_yaw_relative_;
  ros::ServiceClient service_client_trajectory_;
  ros::ServiceClient service_client_fly_to_start_;
  ros::ServiceClient service_client_start_following_;

private:
  void callbackOdometry(const nav_msgs::OdometryConstPtr &msg);
  void callbackPositionCommand(const mrs_msgs::PositionCommandConstPtr &msg);
  void callbackControlManagerDiagnostics(const mrs_msgs::ControlManagerDiagnosticsConstPtr &msg);

private:
  ros::Timer timer_main_;
  void       timerMain(const ros::TimerEvent &event);

private:
  ControlState_t current_state_  = IDLE_STATE;
  ControlState_t previous_state_ = IDLE_STATE;
  void           changeState(ControlState_t new_state);

private:
  double _max_xy_;
  double _min_xy_;
  double _max_z_;
  double _min_z_;
  double _max_yaw_;
  double _min_yaw_;

  double dist3d(double x1, double x2, double y1, double y2, double z1, double z2);
  double dist2d(double x1, double x2, double y1, double y2);
  double randd(double from, double to);
  double genYaw(void);
  double genXY(void);
  double genZ(void);
  double sanitizeYaw(const double yaw_in);
  double angleDist(const double in1, const double in2);
  bool   inDesiredState(void);
  bool   trackerReady(void);
  void   activateTracker(std::string tracker_name);

  int active_tracker_ = 0;
  int takeoff_num_    = 0;

  mrs_msgs::TrackerPoint goal_tracker_point_;

  // | -------------------- for goto testing -------------------- |
private:
  double des_x_;
  double des_y_;
  double des_z_;
  double des_yaw_;
  double home_x_;
  double home_y_;

  // | ------------------ goto relative testing ----------------- |

private:
  double goto_relative_altitude_down_;
  double goto_relative_altitude_up_;

private:
  double _trajectory_p1_;
  double _trajectory_p2_;
  double _trajectory_speed_;
  double _trajectory_yaw_rate_;
  double _trajectory_dt_;

private:
  ros::Time timeout_;

  // | ---------------- looped trajectory testing --------------- |

  ros::Time looping_start_time_;
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
  param_loader.load_param("goto_relative_altitude_down", goto_relative_altitude_down_);
  param_loader.load_param("goto_relative_altitude_up", goto_relative_altitude_up_);

  param_loader.load_param("trajectory/p1", _trajectory_p1_);
  param_loader.load_param("trajectory/p2", _trajectory_p2_);
  param_loader.load_param("trajectory/speed", _trajectory_speed_);
  param_loader.load_param("trajectory/yaw_rate", _trajectory_yaw_rate_);
  param_loader.load_param("trajectory/dt", _trajectory_dt_);

  _trajectory_speed_ = _trajectory_speed_ / 1.414;

  // --------------------------------------------------------------
  // |                         subscribers                        |
  // --------------------------------------------------------------

  subscriber_odometry_         = nh_.subscribe("odometry_in", 1, &ControlTest::callbackOdometry, this, ros::TransportHints().tcpNoDelay());
  subscriber_position_command_ = nh_.subscribe("position_command_in", 1, &ControlTest::callbackPositionCommand, this, ros::TransportHints().tcpNoDelay());
  subscriber_control_manager_diagnostics_ =
      nh_.subscribe("control_manager_diagnostics_in", 1, &ControlTest::callbackControlManagerDiagnostics, this, ros::TransportHints().tcpNoDelay());

  // --------------------------------------------------------------
  // |                         publishers                         |
  // --------------------------------------------------------------

  // | ------------------- std tracker topics ------------------- |

  publisher_reference_ = nh_.advertise<mrs_msgs::ReferenceStamped>("reference_out", 1);

  // | --------------- additional tracker topics ---------------- |

  publisher_set_trajectory_ = nh_.advertise<mrs_msgs::TrackerTrajectory>("set_trajectory_out", 1);

  // --------------------------------------------------------------
  // |                          services                          |
  // --------------------------------------------------------------

  // | -------------------- takeoff and land -------------------- |

  service_client_switch_tracker_ = nh_.serviceClient<mrs_msgs::String>("switch_tracker_out");
  service_client_motors_         = nh_.serviceClient<std_srvs::SetBool>("motors_out");
  service_client_arm_            = nh_.serviceClient<mavros_msgs::CommandBool>("arm_out");
  service_client_offboard_       = nh_.serviceClient<mavros_msgs::SetMode>("offboard_out");
  service_client_takeoff_        = nh_.serviceClient<std_srvs::Trigger>("takeoff_out");
  service_client_land_           = nh_.serviceClient<std_srvs::Trigger>("land_out");
  service_client_land_home_      = nh_.serviceClient<std_srvs::Trigger>("land_home_out");

  // | ------------------ std tracker services ------------------ |

  service_client_set_reference_ = nh_.serviceClient<mrs_msgs::ReferenceStampedSrv>("set_reference_out");

  service_client_goto_             = nh_.serviceClient<mrs_msgs::Vec4>("goto_out");
  service_client_goto_relative_    = nh_.serviceClient<mrs_msgs::Vec4>("goto_relative_out");
  service_client_goto_altitude_    = nh_.serviceClient<mrs_msgs::Vec1>("goto_altitude_out");
  service_client_set_yaw_          = nh_.serviceClient<mrs_msgs::Vec1>("set_yaw_out");
  service_client_set_yaw_relative_ = nh_.serviceClient<mrs_msgs::Vec1>("set_yaw_relative");

  // | -------------- additional trackers services -------------- |

  service_client_trajectory_      = nh_.serviceClient<mrs_msgs::TrackerTrajectorySrv>("set_trajectory_out");
  service_client_fly_to_start_    = nh_.serviceClient<std_srvs::Trigger>("fly_to_start_out");
  service_client_start_following_ = nh_.serviceClient<std_srvs::Trigger>("start_following_out");

  // --------------------------------------------------------------
  // |                           timers                           |
  // --------------------------------------------------------------

  timer_main_ = nh_.createTimer(ros::Rate(10), &ControlTest::timerMain, this);

  // | ---------------------- finish inint ---------------------- |

  if (!param_loader.loaded_successfully()) {
    ros::shutdown();
  }

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
    ROS_ERROR("[ControlTest]: TIMEOUT_, TEST FAILED!!");
    ros::shutdown();
  }

  switch (current_state_) {

    case IDLE_STATE:

      changeState(ControlState_t(int(current_state_) + 1));
      break;

    case TAKEOFF_STATE: {

      std::scoped_lock lock(mutex_control_manager_diagnostics_);

      if (control_manager_diagnostics_.tracker_status.tracker == "MpcTracker" && trackerReady()) {

        ROS_INFO("[ControlTest]: takeoff_num_ %d", takeoff_num_);

        takeoff_num_++;

        changeState(CHANGE_TRACKER_STATE);

        ros::Duration wait(1.0);
        wait.sleep();
      }
      break;
    }

    case CHANGE_TRACKER_STATE:

      if (trackerReady()) {

        ros::Duration wait(1.0);
        wait.sleep();

        if (takeoff_num_ == 1) {

          changeState(SET_REFERENCE_TOPIC_STATE);  // after the first takeoff
          /* changeState(TRAJECTORY_LOAD_STATIC_TOPIC_STATE); */
          /* changeState(TRAJECTORY_LOAD_DYNAMIC_TOPIC_STATE); */
          /* changeState(TRAJECTORY_CIRCLE_LOOP); */

        } else if (takeoff_num_ == 2) {
          changeState(GOTO_ORIGIN_STATE);  // after testing land_home
        }

        wait.sleep();
      }
      break;

    case SET_REFERENCE_TOPIC_STATE:
      if (inDesiredState() && trackerReady()) {
        changeState(ControlState_t(int(current_state_) + 1));
      }
      break;

    case SET_REFERENCE_SERVICE_STATE:
      if (inDesiredState() && trackerReady()) {
        changeState(ControlState_t(int(current_state_) + 1));
      }
      break;

    case GOTO_SERVICE_STATE:
      if (inDesiredState() && trackerReady()) {
        changeState(ControlState_t(int(current_state_) + 1));
      }
      break;

    case GOTO_RELATIVE_SERVICE_STATE:
      if (inDesiredState() && trackerReady()) {
        changeState(ControlState_t(int(current_state_) + 1));
      }
      break;

    case GOTO_ALTITUDE_SERVICE_STATE:
      if (inDesiredState() && trackerReady()) {
        changeState(ControlState_t(int(current_state_) + 1));
      }
      break;

    case SET_YAW_SERVICE_STATE:
      if (inDesiredState() && trackerReady()) {
        changeState(ControlState_t(int(current_state_) + 1));
      }
      break;

    case SET_YAW_RELATIVE_SERVICE_STATE:
      if (inDesiredState() && trackerReady()) {
        if (active_tracker_ == 1) {
          changeState(CHANGE_TRACKER_STATE);
        } else if (active_tracker_ == 2) {
          changeState(TRAJECTORY_LOAD_STATIC_TOPIC_STATE);
        }
      }
      break;

    case TRAJECTORY_LOAD_STATIC_TOPIC_STATE:

      changeState(ControlState_t(int(current_state_) + 1));
      break;

    case TRAJECTORY_FLY_TO_START_TOPIC_STATE:

      if (inDesiredState() && trackerReady()) {
        changeState(ControlState_t(int(current_state_) + 1));
      }
      break;

    case TRAJECTORY_START_FOLLOWING_TOPIC_STATE:

      if (inDesiredState() && trackerReady()) {
        changeState(ControlState_t(int(current_state_) + 1));
      }
      break;

    case TRAJECTORY_LOAD_STATIC_SERVICE_STATE:

      changeState(ControlState_t(int(current_state_) + 1));
      break;

    case TRAJECTORY_FLY_TO_START_SERVICE_STATE:

      if (inDesiredState() && trackerReady()) {
        changeState(ControlState_t(int(current_state_) + 1));
      }
      break;

    case TRAJECTORY_START_FOLLOWING_SERVICE_STATE:

      if (inDesiredState() && trackerReady()) {
        changeState(ControlState_t(int(current_state_) + 1));
      }
      break;

    case TRAJECTORY_LOAD_DYNAMIC_TOPIC_STATE:

      if (inDesiredState() && trackerReady()) {
        changeState(ControlState_t(int(current_state_) + 1));
      }
      break;

    case TRAJECTORY_LOAD_DYNAMIC_SERVICE_STATE:

      if (inDesiredState() && trackerReady()) {
        changeState(ControlState_t(int(current_state_) + 1));
      }
      break;

    case TRAJECTORY_CIRCLE_LOOP:

      if ((ros::Time::now() - looping_start_time_).toSec() > 30.0) {

        if (dist3d(odometry_x, cmd_x, odometry_y, cmd_y, odometry_z, cmd_z) < 2.0) {

          changeState(ControlState_t(int(current_state_) + 1));
        }
      }
      break;

    case TRAJECTORY_HEADLESS_LOAD_SERVICE_STATE:

      changeState(ControlState_t(int(current_state_) + 1));
      break;

    case TRAJECTORY_HEADLESS_FLY_TO_START_SERVICE_STATE:

      if (inDesiredState() && trackerReady()) {
        changeState(ControlState_t(int(current_state_) + 1));
      }
      break;

    case TRAJECTORY_HEADLESS_START_FOLLOWING_SERVICE_STATE:

      if (inDesiredState() && trackerReady()) {
        changeState(GOTO_ORIGIN_STATE);
      }
      break;

    case LAND_HOME_STATE: {
      std::scoped_lock lock(mutex_control_manager_diagnostics_);

      if (control_manager_diagnostics_.tracker_status.tracker == "NullTracker" && dist2d(home_x_, odometry_x, home_y_, odometry_y) < 1.0) {
        ROS_INFO("[ControlTest]: %s", control_manager_diagnostics_.tracker_status.tracker.c_str());
        changeState(TAKEOFF_STATE);
      }
    } break;

    case GOTO_ORIGIN_STATE:

      if (inDesiredState() && trackerReady()) {
        if (takeoff_num_ == 1) {
          changeState(LAND_HOME_STATE);
        } else if (takeoff_num_ == 2) {
          changeState(LAND_STATE);
        }
      }
      break;

    case LAND_STATE: {
      std::scoped_lock lock(mutex_control_manager_diagnostics_);

      if (control_manager_diagnostics_.tracker_status.tracker == "NullTracker" && dist2d(des_x_, odometry_x, des_y_, odometry_y) < 1.0) {
        changeState(ControlState_t(int(current_state_) + 1));
      }
    } break;

    case FINISHED_STATE:

      break;
  }
}

//}

// --------------------------------------------------------------
// |                       other methodsd                       |
// --------------------------------------------------------------

/* //{ changeState() */

void ControlTest::changeState(ControlState_t new_state) {

  auto [odometry_x, odometry_y]       = mrs_lib::get_mutexed(mutex_odometry_, odometry_x_, odometry_y_);
  auto [cmd_x, cmd_y, cmd_z, cmd_yaw] = mrs_lib::get_mutexed(mutex_position_command_, cmd_x_, cmd_y_, cmd_z_, cmd_yaw_);

  ROS_INFO("[ControlTest]: chaging state %s -> %s", state_names[current_state_], state_names[new_state]);

  previous_state_ = current_state_;
  current_state_  = new_state;

  mrs_msgs::ReferenceStamped     goal_reference_stamped_topic;
  mrs_msgs::ReferenceStampedSrv  goal_reference_stamped_srv;
  std_msgs::Float64              goal_float64;
  mrs_msgs::Vec4                 goal_vec4;
  mrs_msgs::Vec1                 goal_vec1;
  std_srvs::SetBool              goal_bool;
  mavros_msgs::CommandBool       goal_mavros_commandbool;
  mavros_msgs::SetMode           goal_mavros_set_mode;
  std_srvs::Trigger              goal_trigger;
  mrs_msgs::TrackerTrajectory    goal_trajectory_topic;
  mrs_msgs::TrackerTrajectorySrv goal_trajectory_srv;
  double                         trajectory_length;

  ros::Duration wait(1.0);

  switch (new_state) {

    case IDLE_STATE:
      break;

    case TAKEOFF_STATE:

      /* //{ testing takeoff */
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

      wait.sleep();

      //}

      break;

    case CHANGE_TRACKER_STATE: {

      if (active_tracker_ == 0) {
        activateTracker("LineTracker");
        active_tracker_++;
      } else if (active_tracker_ == 1) {
        activateTracker("MpcTracker");
        active_tracker_++;
      }

      break;
    }

    case SET_REFERENCE_TOPIC_STATE:

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

      //}

      break;

    case SET_REFERENCE_SERVICE_STATE:

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

      //}


    case GOTO_SERVICE_STATE:

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

      //}

      break;

    case GOTO_RELATIVE_SERVICE_STATE:

      /* //{ test goto_relative service */

      goal_vec4.request.goal[0] = genXY();
      goal_vec4.request.goal[1] = genXY();
      goal_vec4.request.goal[2] = randd(goto_relative_altitude_down_, goto_relative_altitude_up_);
      goal_vec4.request.goal[3] = sanitizeYaw(genYaw());

      {
        std::scoped_lock lock(mutex_position_command_);

        des_x_   = cmd_x + goal_vec4.request.goal[0];
        des_y_   = cmd_y + goal_vec4.request.goal[1];
        des_z_   = cmd_z + goal_vec4.request.goal[2];
        des_yaw_ = sanitizeYaw(cmd_yaw + goal_vec4.request.goal[3]);
      }

      service_client_goto_relative_.call(goal_vec4);

      //}

      break;

    case GOTO_ALTITUDE_SERVICE_STATE:

      /* //{ test goto_altitude service */

      goal_vec1.request.goal = genZ();

      des_z_ = goal_vec1.request.goal;

      service_client_goto_altitude_.call(goal_vec1);

      //}

      break;

    case SET_YAW_SERVICE_STATE:

      /* //{ test set_yaw service */

      goal_vec1.request.goal = sanitizeYaw(genYaw());

      des_yaw_ = goal_vec1.request.goal;

      service_client_set_yaw_.call(goal_vec1);

      //}

      break;

    case SET_YAW_RELATIVE_SERVICE_STATE:

      /* //{ test set_yaw_relative service */

      goal_vec1.request.goal = sanitizeYaw(genYaw());

      {
        std::scoped_lock lock(mutex_position_command_);

        des_yaw_ = sanitizeYaw(cmd_yaw + goal_vec1.request.goal);
      }

      service_client_set_yaw_relative_.call(goal_vec1);

      //}

      break;

    case TRAJECTORY_LOAD_STATIC_TOPIC_STATE:

      /* //{ test set_trajectory topic */

      activateTracker("MpcTracker");

      goal_trajectory_topic.fly_now         = false;
      goal_trajectory_topic.header.frame_id = "";
      goal_trajectory_topic.header.stamp    = ros::Time::now();
      goal_trajectory_topic.loop            = false;
      goal_trajectory_topic.use_yaw         = true;
      goal_trajectory_topic.dt              = _trajectory_dt_;

      goal_tracker_point_.x   = _trajectory_p1_;
      goal_tracker_point_.y   = _trajectory_p1_;
      goal_tracker_point_.z   = _min_z_;
      goal_tracker_point_.yaw = 1.57;
      goal_trajectory_topic.points.push_back(goal_tracker_point_);

      des_x_   = goal_tracker_point_.x;
      des_y_   = goal_tracker_point_.y;
      des_z_   = goal_tracker_point_.z;
      des_yaw_ = sanitizeYaw(goal_tracker_point_.yaw);

      trajectory_length = int(1.414 * ((1 / _trajectory_dt_) * fabs(_trajectory_p2_ - _trajectory_p1_)) / (_trajectory_speed_));

      for (int i = 0; i < trajectory_length; i++) {

        goal_tracker_point_.x += _trajectory_speed_ * _trajectory_dt_;
        goal_tracker_point_.y += _trajectory_speed_ * _trajectory_dt_;
        goal_tracker_point_.z += (_max_z_ - _min_z_) / trajectory_length;
        goal_tracker_point_.yaw = sanitizeYaw(goal_tracker_point_.yaw + _trajectory_yaw_rate_ * _trajectory_dt_);
        /* ROS_INFO("[ControlTest]: x %f y %f z %f yaw %f", goal_tracker_point_.x, goal_tracker_point_.y, goal_tracker_point_.z, goal_tracker_point_.yaw); */
        goal_trajectory_topic.points.push_back(goal_tracker_point_);
      }

      try {
        publisher_set_trajectory_.publish(goal_trajectory_topic);
      }
      catch (...) {
        ROS_ERROR("Exception caught during publishing topic %s.", publisher_set_trajectory_.getTopic().c_str());
      }

      wait.sleep();

      //}

      break;

    case TRAJECTORY_FLY_TO_START_TOPIC_STATE:

      /* //{ test fly_to_start_out service */

      service_client_fly_to_start_.call(goal_trigger);

      //}

      break;

    case TRAJECTORY_START_FOLLOWING_TOPIC_STATE:

      /* //{ test start_following_out service */

      des_x_   = goal_tracker_point_.x;
      des_y_   = goal_tracker_point_.y;
      des_z_   = goal_tracker_point_.z;
      des_yaw_ = sanitizeYaw(goal_tracker_point_.yaw);

      service_client_start_following_.call(goal_trigger);

      //}

      break;

    case TRAJECTORY_LOAD_STATIC_SERVICE_STATE:

      /* //{ test trajectory loading using service */

      activateTracker("MpcTracker");

      goal_trajectory_topic.fly_now         = false;
      goal_trajectory_topic.header.frame_id = "";
      goal_trajectory_topic.header.stamp    = ros::Time::now();
      goal_trajectory_topic.loop            = false;
      goal_trajectory_topic.use_yaw         = true;
      goal_trajectory_topic.dt              = _trajectory_dt_;

      goal_tracker_point_.x   = _trajectory_p1_;
      goal_tracker_point_.y   = _trajectory_p1_;
      goal_tracker_point_.z   = _max_z_;
      goal_tracker_point_.yaw = 1.57;
      goal_trajectory_topic.points.push_back(goal_tracker_point_);

      des_x_   = goal_tracker_point_.x;
      des_y_   = goal_tracker_point_.y;
      des_z_   = goal_tracker_point_.z;
      des_yaw_ = sanitizeYaw(goal_tracker_point_.yaw);

      trajectory_length = int(1.414 * ((1 / _trajectory_dt_) * fabs(_trajectory_p2_ - _trajectory_p1_)) / (_trajectory_speed_));

      for (int i = 0; i < trajectory_length; i++) {

        goal_tracker_point_.x += _trajectory_speed_ * _trajectory_dt_;
        goal_tracker_point_.y += _trajectory_speed_ * _trajectory_dt_;
        goal_tracker_point_.z -= (_max_z_ - _min_z_) / trajectory_length;
        goal_tracker_point_.yaw = sanitizeYaw(goal_tracker_point_.yaw - _trajectory_yaw_rate_ * _trajectory_dt_);
        /* ROS_INFO("[ControlTest]: x %f y %f z %f yaw %f", goal_tracker_point_.x, goal_tracker_point_.y, goal_tracker_point_.z, goal_tracker_point_.yaw); */
        goal_trajectory_topic.points.push_back(goal_tracker_point_);
      }

      goal_trajectory_srv.request.trajectory_msg = goal_trajectory_topic;

      service_client_trajectory_.call(goal_trajectory_srv);

      wait.sleep();

      //}

      break;

    case TRAJECTORY_FLY_TO_START_SERVICE_STATE:

      /* //{ test fly_to_start_out service */

      service_client_fly_to_start_.call(goal_trigger);

      //}

      break;

    case TRAJECTORY_START_FOLLOWING_SERVICE_STATE:

      /* //{ test start_following_out service */

      des_x_   = goal_tracker_point_.x;
      des_y_   = goal_tracker_point_.y;
      des_z_   = goal_tracker_point_.z;
      des_yaw_ = sanitizeYaw(goal_tracker_point_.yaw);

      service_client_start_following_.call(goal_trigger);

      //}

      break;

    case TRAJECTORY_LOAD_DYNAMIC_TOPIC_STATE:

      /* //{ test set_trajectory topic with fly_now */

      activateTracker("MpcTracker");

      goal_trajectory_topic.fly_now         = true;
      goal_trajectory_topic.header.frame_id = "";
      goal_trajectory_topic.header.stamp    = ros::Time::now();
      goal_trajectory_topic.loop            = false;
      goal_trajectory_topic.use_yaw         = true;
      goal_trajectory_topic.dt              = _trajectory_dt_;

      goal_tracker_point_.x   = _trajectory_p2_;
      goal_tracker_point_.y   = _trajectory_p2_;
      goal_tracker_point_.z   = _min_z_;
      goal_tracker_point_.yaw = 0;
      goal_trajectory_topic.points.push_back(goal_tracker_point_);

      trajectory_length = int(1.414 * ((1 / _trajectory_dt_) * fabs(_trajectory_p2_ - _trajectory_p1_)) / (_trajectory_speed_));

      for (int i = 0; i < trajectory_length; i++) {

        goal_tracker_point_.x -= _trajectory_speed_ * _trajectory_dt_;
        goal_tracker_point_.y -= _trajectory_speed_ * _trajectory_dt_;
        goal_tracker_point_.z += (_max_z_ - _min_z_) / trajectory_length;
        goal_tracker_point_.yaw = sanitizeYaw(goal_tracker_point_.yaw + _trajectory_yaw_rate_ * _trajectory_dt_);
        /* ROS_INFO("[ControlTest]: x %f y %f z %f yaw %f", goal_tracker_point_.x, goal_tracker_point_.y, goal_tracker_point_.z, goal_tracker_point_.yaw); */
        goal_trajectory_topic.points.push_back(goal_tracker_point_);
      }

      des_x_   = goal_tracker_point_.x;
      des_y_   = goal_tracker_point_.y;
      des_z_   = goal_tracker_point_.z;
      des_yaw_ = sanitizeYaw(goal_tracker_point_.yaw);

      try {
        publisher_set_trajectory_.publish(goal_trajectory_topic);
      }
      catch (...) {
        ROS_ERROR("Exception caught during publishing topic %s.", publisher_set_trajectory_.getTopic().c_str());
      }

      /* wait.sleep(); */

      //}

      break;

    case TRAJECTORY_LOAD_DYNAMIC_SERVICE_STATE:

      /* //{ test trajectory loading using service */

      activateTracker("MpcTracker");

      goal_trajectory_topic.fly_now         = true;
      goal_trajectory_topic.header.frame_id = "";
      goal_trajectory_topic.header.stamp    = ros::Time::now();
      goal_trajectory_topic.loop            = false;
      goal_trajectory_topic.use_yaw         = true;
      goal_trajectory_topic.dt              = _trajectory_dt_;

      goal_tracker_point_.x   = _trajectory_p1_;
      goal_tracker_point_.y   = _trajectory_p1_;
      goal_tracker_point_.z   = _max_z_;
      goal_tracker_point_.yaw = 1.57;
      goal_trajectory_topic.points.push_back(goal_tracker_point_);

      trajectory_length = int(1.414 * ((1 / _trajectory_dt_) * fabs(_trajectory_p2_ - _trajectory_p1_)) / (_trajectory_speed_));

      for (int i = 0; i < trajectory_length; i++) {

        goal_tracker_point_.x += _trajectory_speed_ * _trajectory_dt_;
        goal_tracker_point_.y += _trajectory_speed_ * _trajectory_dt_;
        goal_tracker_point_.z -= (_max_z_ - _min_z_) / trajectory_length;
        goal_tracker_point_.yaw = sanitizeYaw(goal_tracker_point_.yaw - _trajectory_yaw_rate_ * _trajectory_dt_);
        /* ROS_INFO("[ControlTest]: x %f y %f z %f yaw %f", goal_tracker_point_.x, goal_tracker_point_.y, goal_tracker_point_.z, goal_tracker_point_.yaw); */
        goal_trajectory_topic.points.push_back(goal_tracker_point_);
      }

      des_x_   = goal_tracker_point_.x;
      des_y_   = goal_tracker_point_.y;
      des_z_   = goal_tracker_point_.z;
      des_yaw_ = sanitizeYaw(goal_tracker_point_.yaw);

      goal_trajectory_srv.request.trajectory_msg = goal_trajectory_topic;

      service_client_trajectory_.call(goal_trajectory_srv);

      //}

      break;

    case TRAJECTORY_CIRCLE_LOOP: {

      /* //{ load trajectory for testing the headless following */

      activateTracker("MpcTracker");

      goal_trajectory_topic.fly_now         = true;
      goal_trajectory_topic.header.frame_id = "";
      goal_trajectory_topic.header.stamp    = ros::Time::now();
      goal_trajectory_topic.loop            = true;
      goal_trajectory_topic.use_yaw         = true;
      goal_trajectory_topic.dt              = _trajectory_dt_;

      double radius = 15.0;
      double speed  = 8.0;
      double angle  = 0;

      goal_tracker_point_.x   = radius;
      goal_tracker_point_.y   = 0;
      goal_tracker_point_.z   = _min_z_;
      goal_tracker_point_.yaw = 1.57;
      goal_trajectory_topic.points.push_back(goal_tracker_point_);

      double trajectory_time   = (radius * 2 * M_PI) / speed;
      int    trajectory_length = floor(trajectory_time * (1 / _trajectory_dt_));
      double angular_step      = (2 * M_PI) / trajectory_length;

      double last_x = goal_tracker_point_.x;
      double last_y = goal_tracker_point_.y;

      for (int i = 0; i < trajectory_length; i++) {

        angle += angular_step;

        goal_tracker_point_.x   = radius * cos(angle);
        goal_tracker_point_.y   = radius * sin(angle);
        goal_tracker_point_.z   = _min_z_;
        goal_tracker_point_.yaw = atan2(goal_tracker_point_.y - last_y, goal_tracker_point_.x - last_x);

        last_x = goal_tracker_point_.x;
        last_y = goal_tracker_point_.y;

        goal_trajectory_topic.points.push_back(goal_tracker_point_);
      }

      looping_start_time_ = ros::Time::now();

      try {
        publisher_set_trajectory_.publish(goal_trajectory_topic);
      }
      catch (...) {
        ROS_ERROR("Exception caught during publishing topic %s.", publisher_set_trajectory_.getTopic().c_str());
      }

      wait.sleep();

    }

    //}

    break;

    case TRAJECTORY_HEADLESS_LOAD_SERVICE_STATE: {

      /* //{ load trajectory for testing the headless following */

      activateTracker("MpcTracker");

      goal_trajectory_topic.fly_now         = false;
      goal_trajectory_topic.header.frame_id = "";
      goal_trajectory_topic.header.stamp    = ros::Time::now();
      goal_trajectory_topic.loop            = false;
      goal_trajectory_topic.use_yaw         = true;
      goal_trajectory_topic.dt              = _trajectory_dt_;

      double radius = 5;

      goal_tracker_point_.x   = radius;
      goal_tracker_point_.y   = 0;
      goal_tracker_point_.z   = _min_z_;
      goal_tracker_point_.yaw = 0;
      goal_trajectory_topic.points.push_back(goal_tracker_point_);

      des_x_   = goal_tracker_point_.x;
      des_y_   = goal_tracker_point_.y;
      des_z_   = goal_tracker_point_.z;
      des_yaw_ = sanitizeYaw(goal_tracker_point_.yaw);

      trajectory_length = 10 * (1 / _trajectory_dt_);
      ;

      double angle = 0;

      for (int i = 0; i < trajectory_length; i++) {

        angle += (2 * M_PI) / trajectory_length;

        goal_tracker_point_.x   = radius * cos(angle);
        goal_tracker_point_.y   = radius * sin(angle);
        goal_tracker_point_.z   = _min_z_;
        goal_tracker_point_.yaw = 0;
        goal_trajectory_topic.points.push_back(goal_tracker_point_);
      }

      trajectory_length = 5 * (1 / _trajectory_dt_);

      for (int i = 0; i < trajectory_length; i++) {

        goal_tracker_point_.x   = radius;
        goal_tracker_point_.y   = -radius + i * (2 * radius / trajectory_length);
        goal_tracker_point_.z   = _min_z_;
        goal_tracker_point_.yaw = 0;
        goal_trajectory_topic.points.push_back(goal_tracker_point_);
      }

      for (int i = 0; i < trajectory_length; i++) {

        goal_tracker_point_.x   = radius - i * (2 * radius / trajectory_length);
        goal_tracker_point_.y   = radius;
        goal_tracker_point_.z   = _min_z_;
        goal_tracker_point_.yaw = 0;
        goal_trajectory_topic.points.push_back(goal_tracker_point_);
      }

      for (int i = 0; i < trajectory_length; i++) {

        goal_tracker_point_.x   = -radius;
        goal_tracker_point_.y   = radius - i * (2 * radius / trajectory_length);
        goal_tracker_point_.z   = _min_z_;
        goal_tracker_point_.yaw = 0;
        goal_trajectory_topic.points.push_back(goal_tracker_point_);
      }

      for (int i = 0; i < trajectory_length; i++) {

        goal_tracker_point_.x   = -radius + i * (2 * radius / trajectory_length);
        goal_tracker_point_.y   = -radius;
        goal_tracker_point_.z   = _min_z_;
        goal_tracker_point_.yaw = 0;
        goal_trajectory_topic.points.push_back(goal_tracker_point_);
      }

      try {
        publisher_set_trajectory_.publish(goal_trajectory_topic);
      }
      catch (...) {
        ROS_ERROR("Exception caught during publishing topic %s.", publisher_set_trajectory_.getTopic().c_str());
      }

      wait.sleep();

    }

    //}

    break;

    case TRAJECTORY_HEADLESS_FLY_TO_START_SERVICE_STATE:

      /* //{ test fly_to_start_out service */

      service_client_fly_to_start_.call(goal_trigger);

      //}

      break;

    case TRAJECTORY_HEADLESS_START_FOLLOWING_SERVICE_STATE:

      /* //{ test start_following_out service */

      des_x_   = goal_tracker_point_.x;
      des_y_   = goal_tracker_point_.y;
      des_z_   = goal_tracker_point_.z;
      des_yaw_ = sanitizeYaw(goal_tracker_point_.yaw);

      service_client_start_following_.call(goal_trigger);

      //}

      break;
    case LAND_HOME_STATE:

      /* //{ test land_home service */

      service_client_land_home_.call(goal_trigger);

      //}

      break;

    case GOTO_ORIGIN_STATE:

      /* //{ go to origin */

      activateTracker("MpcTracker");

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

      //}

      break;

    case LAND_STATE:

      /* //{ test land service */

      service_client_land_.call(goal_trigger);

      //}

      break;

    case FINISHED_STATE:

      /* //{ finish and end the node */

      ROS_INFO(" ");
      ROS_INFO("[ControlTest]: TEST FINISHED");
      ros::shutdown();

      //}

      break;
  }

  timeout_ = ros::Time::now();
}

//}

/* //{ randd() */

double ControlTest::randd(double from, double to) {

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

double ControlTest::dist3d(double x1, double x2, double y1, double y2, double z1, double z2) {

  return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2) + pow(z1 - z2, 2));
}

//}

/* //{ dist2d() */

double ControlTest::dist2d(double x1, double x2, double y1, double y2) {

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

  if (dist3d(odometry_x, des_x_, odometry_y, des_y_, odometry_z, des_z_) < 0.15 && angleDist(odometry_yaw, sanitizeYaw(des_yaw_)) < 0.15) {

    ROS_WARN("[ControlTest]: The goal has been reached.");
    ros::Duration(1.0).sleep();
    return true;
  }

  return false;
}

//}

/* //{ trackerReady() */

bool ControlTest::trackerReady(void) {

  return control_manager_diagnostics_.tracker_status.active && control_manager_diagnostics_.tracker_status.callbacks_enabled;
}

//}

/* //{ activateTracker() */

void ControlTest::activateTracker(std::string tracker_name) {

  mrs_msgs::String goal_switch_tracker;
  goal_switch_tracker.request.value = tracker_name;

  ROS_INFO("[ControlTest]: switching to %s", goal_switch_tracker.request.value.c_str());
  service_client_switch_tracker_.call(goal_switch_tracker);
  ros::Duration wait(1.0);
  wait.sleep();
}

//}

}  // namespace mrs_testing

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_testing::ControlTest, nodelet::Nodelet)
