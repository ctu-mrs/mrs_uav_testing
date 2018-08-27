#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>

#include <std_msgs/Float64.h>

#include <mrs_msgs/TrackerPointStamped.h>
#include <mrs_msgs/TrackerPoint.h>
#include <mrs_msgs/TrackerTrajectory.h>
#include <mrs_msgs/TrackerTrajectorySrv.h>
#include <mrs_msgs/Vec4.h>
#include <mrs_msgs/Vec1.h>
#include <mrs_msgs/String.h>
#include <mrs_msgs/PositionCommand.h>
#include <mrs_msgs/TrackerStatus.h>

#include <nav_msgs/Odometry.h>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>

#include <mutex>

#include <tf/transform_datatypes.h>

#include <mrs_lib/ParamLoader.h>

#define PI 3.141592653

namespace mrs_testing
{

//{ state machine states

// state machine
typedef enum
{
  IDLE_STATE,
  TAKEOFF_STATE,
  GOTO_TOPIC_STATE,
  GOTO_RELATIVE_TOPIC_STATE,
  GOTO_ALTITUDE_TOPIC_STATE,
  SET_YAW_TOPIC_STATE,
  SET_YAW_RELATIVE_TOPIC_STATE,
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
  LAND_HOME_STATE,
  GOTO_ORIGIN_STATE,
  LAND_STATE,
  FINISHED_STATE,
} ControlState_t;

const char *state_names[24] = {
    "IDLE_STATE",
    "TAKEOFF_STATE",
    "GOTO_TOPIC_STATE",
    "GOTO_RELATIVE_TOPIC_STATE",
    "GOTO_ALTITUDE_TOPIC_STATE",
    "SET_YAW_TOPIC_STATE",
    "SET_YAW_RELATIVE_TOPIC_STATE",
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
    "LAND_HOME_STATE",
    "GOTO_ORIGIN_STATE",
    "LAND_STATE",
    "FINISHED_STATE",
};

//}

//{ class ControlTest

class ControlTest : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  ros::NodeHandle nh_;
  bool            is_initialized = false;

private:
  ros::Subscriber    subscriber_odometry;
  bool               got_odometry = false;
  nav_msgs::Odometry odometry;
  std::mutex         mutex_odometry;
  double             odometry_yaw, odometry_pitch, odometry_roll;
  double             odometry_x, odometry_y, odometry_z;

private:
  ros::Subscriber           subscriber_position_command;
  bool                      got_position_command = false;
  mrs_msgs::PositionCommand position_command;
  double                    cmd_x, cmd_y, cmd_z, cmd_yaw;
  std::mutex                mutex_cmd;

private:
  ros::Subscriber         subscriber_tracker_status;
  bool                    got_tracker_status = false;
  mrs_msgs::TrackerStatus tracker_status;
  std::mutex              mutex_tracker_status;

private:
  ros::Publisher publisher_goto;
  ros::Publisher publisher_goto_relative;
  ros::Publisher publisher_goto_altitude;
  ros::Publisher publisher_set_yaw;
  ros::Publisher publisher_set_yaw_relative;
  ros::Publisher publisher_set_trajectory;

private:
  ros::ServiceClient service_client_switch_tracker;
  ros::ServiceClient service_client_arm;
  ros::ServiceClient service_client_offboard;
  ros::ServiceClient service_client_takeoff;
  ros::ServiceClient service_client_land;
  ros::ServiceClient service_client_land_home;
  ros::ServiceClient service_client_motors;

private:
  ros::ServiceClient service_client_goto;
  ros::ServiceClient service_client_goto_relative;
  ros::ServiceClient service_client_goto_altitude;
  ros::ServiceClient service_client_set_yaw;
  ros::ServiceClient service_client_set_yaw_relative;
  ros::ServiceClient service_client_trajectory;
  ros::ServiceClient service_client_fly_to_start;
  ros::ServiceClient service_client_start_following;

private:
  void callbackOdometry(const nav_msgs::OdometryConstPtr &msg);
  void callbackPositionCommand(const mrs_msgs::PositionCommandConstPtr &msg);
  void callbackTrackerStatus(const mrs_msgs::TrackerStatusConstPtr &msg);

private:
  ros::Timer main_timer;
  void       mainTimer(const ros::TimerEvent &event);

private:
  ControlState_t current_state  = IDLE_STATE;
  ControlState_t previous_state = IDLE_STATE;
  void           changeState(ControlState_t new_state);

private:
  double max_xy_;
  double min_xy_;
  double max_z_;
  double min_z_;
  double max_yaw_;
  double min_yaw_;

  double dist3d(double x1, double x2, double y1, double y2, double z1, double z2);
  double dist2d(double x1, double x2, double y1, double y2);
  double randd(double from, double to);
  double genYaw(void);
  double genXY(void);
  double genZ(void);
  double sanitizeYaw(const double yaw_in);
  bool   inDesiredState(void);
  void   activateTracker(std::string tracker_name);

  int active_tracker = 0;
  int takeoff_num    = 0;

  mrs_msgs::TrackerPoint goal_tracker_point;

  // | -------------------- for goto testing -------------------- |
private:
  double des_x, des_y, des_z, des_yaw;
  double home_x, home_y;

  // | ------------------ goto relative testing ----------------- |

private:
  double goto_relative_altitude_down_;
  double goto_relative_altitude_up_;

private:
  double trajectory_p1_, trajectory_p2_, trajectory_speed_;

private:
  ros::Time timeout;
};

//}

//{ onInit()

void ControlTest::onInit() {

  ros::NodeHandle nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  ros::Time::waitForValid();

  timeout = ros::Time::now();

  srand(static_cast<unsigned int>(ros::Time::now().toSec()));

  // --------------------------------------------------------------
  // |                           params                           |
  // --------------------------------------------------------------

  mrs_lib::ParamLoader param_loader(nh_, "ControlTest");

  param_loader.load_param("max_xy", max_xy_);
  param_loader.load_param("min_xy", max_xy_);
  param_loader.load_param("max_z", max_z_);
  param_loader.load_param("min_z", min_z_);
  param_loader.load_param("max_yaw", max_yaw_);
  param_loader.load_param("min_yaw", min_yaw_);
  param_loader.load_param("goto_relative_altitude_down", goto_relative_altitude_down_);
  param_loader.load_param("goto_relative_altitude_up", goto_relative_altitude_up_);

  param_loader.load_param("trajectory/p1", trajectory_p1_);
  param_loader.load_param("trajectory/p2", trajectory_p2_);
  param_loader.load_param("trajectory/speed", trajectory_speed_);

  trajectory_speed_ = trajectory_speed_/1.414;

  // --------------------------------------------------------------
  // |                         subscribers                        |
  // --------------------------------------------------------------

  subscriber_odometry         = nh_.subscribe("odometry_in", 1, &ControlTest::callbackOdometry, this, ros::TransportHints().tcpNoDelay());
  subscriber_position_command = nh_.subscribe("position_command_in", 1, &ControlTest::callbackPositionCommand, this, ros::TransportHints().tcpNoDelay());
  subscriber_tracker_status   = nh_.subscribe("tracker_status_in", 1, &ControlTest::callbackTrackerStatus, this, ros::TransportHints().tcpNoDelay());

  // --------------------------------------------------------------
  // |                         publishers                         |
  // --------------------------------------------------------------

  // | ------------------- std tracker topics ------------------- |

  publisher_goto             = nh_.advertise<mrs_msgs::TrackerPointStamped>("goto_out", 1);
  publisher_goto_relative    = nh_.advertise<mrs_msgs::TrackerPointStamped>("goto_relative_out", 1);
  publisher_goto_altitude    = nh_.advertise<std_msgs::Float64>("goto_altitude_out", 1);
  publisher_set_yaw          = nh_.advertise<std_msgs::Float64>("set_yaw_out", 1);
  publisher_set_yaw_relative = nh_.advertise<std_msgs::Float64>("set_yaw_relative", 1);

  // | --------------- additional tracker topics ---------------- |

  publisher_set_trajectory = nh_.advertise<mrs_msgs::TrackerTrajectory>("set_trajectory_out", 1);

  // --------------------------------------------------------------
  // |                          services                          |
  // --------------------------------------------------------------

  // | -------------------- takeoff and land -------------------- |

  service_client_switch_tracker = nh_.serviceClient<mrs_msgs::String>("switch_tracker_out");
  service_client_motors         = nh_.serviceClient<std_srvs::SetBool>("motors_out");
  service_client_arm            = nh_.serviceClient<mavros_msgs::CommandBool>("arm_out");
  service_client_offboard       = nh_.serviceClient<mavros_msgs::SetMode>("offboard_out");
  service_client_takeoff        = nh_.serviceClient<std_srvs::Trigger>("takeoff_out");
  service_client_land           = nh_.serviceClient<std_srvs::Trigger>("land_out");
  service_client_land_home      = nh_.serviceClient<std_srvs::Trigger>("land_home_out");

  // | ------------------ std tracker services ------------------ |

  service_client_goto             = nh_.serviceClient<mrs_msgs::Vec4>("goto_out");
  service_client_goto_relative    = nh_.serviceClient<mrs_msgs::Vec4>("goto_relative_out");
  service_client_goto_altitude    = nh_.serviceClient<mrs_msgs::Vec1>("goto_altitude_out");
  service_client_set_yaw          = nh_.serviceClient<mrs_msgs::Vec1>("set_yaw_out");
  service_client_set_yaw_relative = nh_.serviceClient<mrs_msgs::Vec1>("set_yaw_relative");

  // | -------------- additional trackers services -------------- |

  service_client_trajectory      = nh_.serviceClient<mrs_msgs::TrackerTrajectorySrv>("set_trajectory_out");
  service_client_fly_to_start    = nh_.serviceClient<std_srvs::Trigger>("fly_to_start_out");
  service_client_start_following = nh_.serviceClient<std_srvs::Trigger>("start_following_out");

  // --------------------------------------------------------------
  // |                           timers                           |
  // --------------------------------------------------------------

  main_timer = nh_.createTimer(ros::Rate(10), &ControlTest::mainTimer, this);

  // | ---------------------- finish inint ---------------------- |

  if (!param_loader.loaded_successfully()) {
    ros::shutdown();
  }

  is_initialized = true;

  ROS_INFO("[ControlTest]: initialized");
}

//}

// --------------------------------------------------------------
// |                          callbacks                         |
// --------------------------------------------------------------

//{ callbackOdometry()

void ControlTest::callbackOdometry(const nav_msgs::OdometryConstPtr &msg) {

  if (!is_initialized)
    return;

  got_odometry = true;

  mutex_odometry.lock();
  {
    odometry   = *msg;
    odometry_x = odometry.pose.pose.position.x;
    odometry_y = odometry.pose.pose.position.y;
    odometry_z = odometry.pose.pose.position.z;
  }

  // calculate the euler angles
  tf::Quaternion quaternion_odometry;
  quaternionMsgToTF(odometry.pose.pose.orientation, quaternion_odometry);
  tf::Matrix3x3 m(quaternion_odometry);
  m.getRPY(odometry_roll, odometry_pitch, odometry_yaw);

  mutex_odometry.unlock();
}

//}

//{ callbackPositionCommand()

void ControlTest::callbackPositionCommand(const mrs_msgs::PositionCommandConstPtr &msg) {

  if (!is_initialized)
    return;

  got_position_command = true;

  mutex_cmd.lock();
  {
    position_command = *msg;

    cmd_x   = position_command.position.x;
    cmd_y   = position_command.position.y;
    cmd_z   = position_command.position.z;
    cmd_yaw = position_command.yaw;
  }
  mutex_cmd.unlock();
}

//}

//{ callbackTrackerStatus()

void ControlTest::callbackTrackerStatus(const mrs_msgs::TrackerStatusConstPtr &msg) {

  if (!is_initialized)
    return;

  got_tracker_status = true;

  mutex_tracker_status.lock();
  { tracker_status = *msg; }
  mutex_tracker_status.unlock();
}

//}

// --------------------------------------------------------------
// |                           timers                           |
// --------------------------------------------------------------

//{ mainTimer()

void ControlTest::mainTimer(const ros::TimerEvent &event) {

  if (!is_initialized)
    return;

  if (!got_odometry) {
    ROS_INFO_THROTTLE(1.0, "[ControlTest]: waiting for data, got_odometry: %s", got_odometry ? "YES" : "NO");
    return;
  }

  ROS_INFO_THROTTLE(5.0, " ");
  ROS_INFO_THROTTLE(5.0, "[ControlTest]: dessired: %f %f %f %f", des_x, des_y, des_z, des_yaw);
  ROS_INFO_THROTTLE(5.0, "[ControlTest]: cmd: %f %f %f %f", cmd_x, cmd_y, cmd_z, sanitizeYaw(cmd_yaw));
  ROS_INFO_THROTTLE(5.0, "[ControlTest]: odom: %f %f %f %f", odometry_x, odometry_y, odometry_z, sanitizeYaw(odometry_yaw));
  ROS_INFO_THROTTLE(5.0, " ");

  if ((ros::Time::now() - timeout).toSec() > 60.0) {
    ROS_ERROR("[ControlTest]: TIMEOUT, TEST FAILED!!");
    ros::shutdown();
  }

  switch (current_state) {

    case IDLE_STATE:

      changeState(ControlState_t(int(current_state) + 1));
      break;

    case TAKEOFF_STATE:

      if (odometry_z > 1.0) {

        /* changeState(ControlState_t(int(current_state) + 1)); */

        ROS_INFO("[ControlTest]: takeoff_num %d", takeoff_num);

        if (takeoff_num == 0) {
          changeState(GOTO_TOPIC_STATE);  // after the first takeoff
        } else if (takeoff_num == 1) {
          changeState(GOTO_ORIGIN_STATE);  // after testing land_home
        }

        takeoff_num++;
      }
      break;

    case GOTO_TOPIC_STATE:
      if (inDesiredState()) {
        changeState(ControlState_t(int(current_state) + 1));
      }
      break;

    case GOTO_RELATIVE_TOPIC_STATE:
      if (inDesiredState()) {
        changeState(ControlState_t(int(current_state) + 1));
      }
      break;

    case GOTO_ALTITUDE_TOPIC_STATE:
      if (inDesiredState()) {
        changeState(ControlState_t(int(current_state) + 1));
      }
      break;

    case SET_YAW_TOPIC_STATE:
      if (inDesiredState()) {
        changeState(ControlState_t(int(current_state) + 1));
      }
      break;

    case SET_YAW_RELATIVE_TOPIC_STATE:
      if (inDesiredState()) {
        changeState(ControlState_t(int(current_state) + 1));
      }
      break;

    case GOTO_SERVICE_STATE:
      if (inDesiredState()) {
        changeState(ControlState_t(int(current_state) + 1));
      }
      break;

    case GOTO_RELATIVE_SERVICE_STATE:
      if (inDesiredState()) {
        changeState(ControlState_t(int(current_state) + 1));
      }
      break;

    case GOTO_ALTITUDE_SERVICE_STATE:
      if (inDesiredState()) {
        changeState(ControlState_t(int(current_state) + 1));
      }
      break;

    case SET_YAW_SERVICE_STATE:
      if (inDesiredState()) {
        changeState(ControlState_t(int(current_state) + 1));
      }
      break;

    case SET_YAW_RELATIVE_SERVICE_STATE:
      if (inDesiredState()) {

        if (active_tracker == 1) {
          changeState(GOTO_TOPIC_STATE);
        } else if (active_tracker == 2) {
          changeState(TRAJECTORY_LOAD_STATIC_TOPIC_STATE);
        }
      }
      break;

    case TRAJECTORY_LOAD_STATIC_TOPIC_STATE:

      changeState(ControlState_t(int(current_state) + 1));
      break;

    case TRAJECTORY_FLY_TO_START_TOPIC_STATE:

      if (inDesiredState()) {
        changeState(ControlState_t(int(current_state) + 1));
      }
      break;

    case TRAJECTORY_START_FOLLOWING_TOPIC_STATE:

      if (inDesiredState()) {
        changeState(ControlState_t(int(current_state) + 1));
      }
      break;

    case TRAJECTORY_LOAD_STATIC_SERVICE_STATE:

      changeState(ControlState_t(int(current_state) + 1));
      break;

    case TRAJECTORY_FLY_TO_START_SERVICE_STATE:

      if (inDesiredState()) {
        changeState(ControlState_t(int(current_state) + 1));
      }
      break;

    case TRAJECTORY_START_FOLLOWING_SERVICE_STATE:

      if (inDesiredState()) {
        changeState(ControlState_t(int(current_state) + 1));
      }
      break;

    case TRAJECTORY_LOAD_DYNAMIC_TOPIC_STATE:

      if (inDesiredState()) {
        changeState(ControlState_t(int(current_state) + 1));
      }
      break;

    case TRAJECTORY_LOAD_DYNAMIC_SERVICE_STATE:

      if (inDesiredState()) {
        changeState(GOTO_ORIGIN_STATE);
      }
      break;

    case LAND_HOME_STATE:
      mutex_tracker_status.lock();
      {
        if (tracker_status.tracker.compare("mrs_mav_manager/NullTracker") == 0 && dist2d(home_x, odometry_x, home_y, odometry_y) < 1.0) {
          ROS_INFO("[ControlTest]: %s", tracker_status.tracker.c_str());
          changeState(TAKEOFF_STATE);
        }
      }
      mutex_tracker_status.unlock();
      break;

    case GOTO_ORIGIN_STATE:

      if (inDesiredState()) {
        if (takeoff_num == 1) {
          changeState(LAND_HOME_STATE);
        } else if (takeoff_num == 2) {
          changeState(LAND_STATE);
        }
      }
      break;

    case LAND_STATE:
      mutex_tracker_status.lock();
      {
        if (tracker_status.tracker.compare("mrs_mav_manager/NullTracker") == 0 && dist2d(des_x, odometry_x, des_y, odometry_y) < 1.0) {
          changeState(ControlState_t(int(current_state) + 1));
        }
      }
      mutex_tracker_status.unlock();
      break;

    case FINISHED_STATE:

      break;
  }
}

//}

// --------------------------------------------------------------
// |                       other methodsd                       |
// --------------------------------------------------------------

//{ changeState()

void ControlTest::changeState(ControlState_t new_state) {

  ROS_INFO("[ControlTest]: chaging state %s -> %s", state_names[current_state], state_names[new_state]);

  previous_state = current_state;
  current_state  = new_state;

  mrs_msgs::TrackerPointStamped  goal_tracker_point_stamped;
  std_msgs::Float64              goal_float64;
  mrs_msgs::Vec4                 goal_vec4;
  mrs_msgs::Vec1                 goal_vec1;
  std_srvs::SetBool              goal_bool;
  mavros_msgs::CommandBool       goal_mavros_commandbool;
  mavros_msgs::SetMode           goal_mavros_set_mode;
  std_srvs::Trigger              goal_trigger;
  mrs_msgs::TrackerTrajectory    goal_trajectory_topic;
  mrs_msgs::TrackerTrajectorySrv goal_trajectory_srv;
  double trajectory_length;

  ros::Duration wait(1.0);

  switch (new_state) {

    case IDLE_STATE:
      break;

    case GOTO_TOPIC_STATE:

      //{ test goto topic

      if (active_tracker == 0) {
        activateTracker("mrs_trackers/LineTracker");
        active_tracker++;
      } else if (active_tracker == 1) {
        activateTracker("mrs_trackers/MpcTracker");
        active_tracker++;
      }

      goal_tracker_point_stamped.position.x   = genXY();
      goal_tracker_point_stamped.position.y   = genXY();
      goal_tracker_point_stamped.position.z   = genZ();
      goal_tracker_point_stamped.position.yaw = sanitizeYaw(genYaw());

      des_x   = goal_tracker_point_stamped.position.x;
      des_y   = goal_tracker_point_stamped.position.y;
      des_z   = goal_tracker_point_stamped.position.z;
      des_yaw = goal_tracker_point_stamped.position.yaw;

      try {
        publisher_goto.publish(goal_tracker_point_stamped);
      }
      catch (...) {
        ROS_ERROR("Exception caught during publishing topic %s.", publisher_goto.getTopic().c_str());
      }

      //}

      break;

    case TAKEOFF_STATE:

      //{ testing takeoff
      // | ------------------------- motors ------------------------- |
      goal_bool.request.data = 1;
      service_client_motors.call(goal_bool);

      // | ------------------------- arming ------------------------- |
      goal_mavros_commandbool.request.value = 1;
      service_client_arm.call(goal_mavros_commandbool);

      // | ------------------------ offboard ------------------------ |
      goal_mavros_set_mode.request.base_mode   = 0;
      goal_mavros_set_mode.request.custom_mode = "offboard";
      service_client_offboard.call(goal_mavros_set_mode);

      // | ------------------------- takeoff ------------------------ |
      service_client_takeoff.call(goal_trigger);

      home_x = odometry_x;
      home_y = odometry_y;

      wait.sleep();

      //}

      break;

    case GOTO_RELATIVE_TOPIC_STATE:

      //{ test goto_relative topic

      goal_tracker_point_stamped.position.x   = genXY();
      goal_tracker_point_stamped.position.y   = genXY();
      goal_tracker_point_stamped.position.z   = randd(goto_relative_altitude_down_, goto_relative_altitude_up_);
      goal_tracker_point_stamped.position.yaw = sanitizeYaw(genYaw());

      mutex_cmd.lock();
      {
        des_x   = goal_tracker_point_stamped.position.x + cmd_x;
        des_y   = goal_tracker_point_stamped.position.y + cmd_y;
        des_z   = goal_tracker_point_stamped.position.z + cmd_z;
        des_yaw = sanitizeYaw(goal_tracker_point_stamped.position.yaw + cmd_yaw);
      }
      mutex_cmd.unlock();

      try {
        publisher_goto_relative.publish(goal_tracker_point_stamped);
      }
      catch (...) {
        ROS_ERROR("Exception caught during publishing topic %s.", publisher_goto_relative.getTopic().c_str());
      }

      //}

      break;

    case GOTO_ALTITUDE_TOPIC_STATE:

      //{ test goto_altitude topic

      goal_float64.data = genZ();

      des_z   = goal_float64.data;

      try {
        publisher_goto_altitude.publish(goal_float64);
      }
      catch (...) {
        ROS_ERROR("Exception caught during publishing topic %s.", publisher_goto_altitude.getTopic().c_str());
      }

      //}

      break;

    case SET_YAW_TOPIC_STATE:

      //{ test set_yaw topic

      goal_float64.data = sanitizeYaw(genYaw());

      des_yaw = goal_float64.data;

      try {
        publisher_set_yaw.publish(goal_float64);
      }
      catch (...) {
        ROS_ERROR("Exception caught during publishing topic %s.", publisher_set_yaw.getTopic().c_str());
      }

      //}

      break;

    case SET_YAW_RELATIVE_TOPIC_STATE:

      //{ test set_yaw_relative topic

      goal_float64.data = sanitizeYaw(genYaw());

      mutex_cmd.lock();
      {
        des_yaw = sanitizeYaw(cmd_yaw + goal_float64.data);
      }
      mutex_cmd.unlock();

      publisher_set_yaw_relative.publish(goal_float64);

      //}

      break;

    case GOTO_SERVICE_STATE:

      //{ test goto service

      goal_vec4.request.goal[0] = genXY();
      goal_vec4.request.goal[1] = genXY();
      goal_vec4.request.goal[2] = genZ();
      goal_vec4.request.goal[3] = sanitizeYaw(genYaw());

      des_x   = goal_vec4.request.goal[0];
      des_y   = goal_vec4.request.goal[1];
      des_z   = goal_vec4.request.goal[2];
      des_yaw = goal_vec4.request.goal[3];

      service_client_goto.call(goal_vec4);

      //}

      break;

    case GOTO_RELATIVE_SERVICE_STATE:

      //{ test goto_relative service

      goal_vec4.request.goal[0] = genXY();
      goal_vec4.request.goal[1] = genXY();
      goal_vec4.request.goal[2] = randd(goto_relative_altitude_down_, goto_relative_altitude_up_);
      goal_vec4.request.goal[3] = sanitizeYaw(genYaw());

      mutex_cmd.lock();
      {
        des_x   = cmd_x + goal_vec4.request.goal[0];
        des_y   = cmd_y + goal_vec4.request.goal[1];
        des_z   = cmd_z + goal_vec4.request.goal[2];
        des_yaw = sanitizeYaw(cmd_yaw + goal_vec4.request.goal[3]);
      }
      mutex_cmd.unlock();

      service_client_goto_relative.call(goal_vec4);

      //}

      break;

    case GOTO_ALTITUDE_SERVICE_STATE:

      //{ test goto_altitude service

      goal_vec1.request.goal = genZ();

      des_z   = goal_vec1.request.goal;

      service_client_goto_altitude.call(goal_vec1);

      //}

      break;

    case SET_YAW_SERVICE_STATE:

      //{ test set_yaw service

      goal_vec1.request.goal = sanitizeYaw(genYaw());

      des_yaw = goal_vec1.request.goal;

      service_client_set_yaw.call(goal_vec1);

      //}

      break;

    case SET_YAW_RELATIVE_SERVICE_STATE:

      //{ test set_yaw_relative service

      goal_vec1.request.goal = sanitizeYaw(genYaw());

      mutex_cmd.lock();
      {
        des_yaw = sanitizeYaw(cmd_yaw + goal_vec1.request.goal);
      }
      mutex_cmd.unlock();

      service_client_set_yaw_relative.call(goal_vec1);

      //}

      break;

    case TRAJECTORY_LOAD_STATIC_TOPIC_STATE:

      //{ test set_trajectory topic

      activateTracker("mrs_trackers/MpcTracker");

      goal_trajectory_topic.fly_now         = false;
      goal_trajectory_topic.header.frame_id = "local_origin";
      goal_trajectory_topic.header.stamp    = ros::Time::now();
      goal_trajectory_topic.loop            = false;
      goal_trajectory_topic.use_yaw         = true;
      goal_trajectory_topic.start_index     = 0;

      goal_tracker_point.x   = trajectory_p1_;
      goal_tracker_point.y   = trajectory_p1_;
      goal_tracker_point.z   = min_z_;
      goal_tracker_point.yaw = 1.57;
      goal_trajectory_topic.points.push_back(goal_tracker_point);

      des_x   = goal_tracker_point.x;
      des_y   = goal_tracker_point.y;
      des_z   = goal_tracker_point.z;
      des_yaw = sanitizeYaw(goal_tracker_point.yaw);

      trajectory_length = int(1.414*(5*fabs(trajectory_p2_-trajectory_p1_))/(trajectory_speed_));

      for (int i = 0; i < trajectory_length; i++) {

        goal_tracker_point.x += trajectory_speed_/5;
        goal_tracker_point.y += trajectory_speed_/5;
        goal_tracker_point.z += (max_z_-min_z_)/trajectory_length;
        goal_tracker_point.yaw = sanitizeYaw(goal_tracker_point.yaw + 0.1);
        /* ROS_INFO("[ControlTest]: x %f y %f z %f yaw %f", goal_tracker_point.x, goal_tracker_point.y, goal_tracker_point.z, goal_tracker_point.yaw); */
        goal_trajectory_topic.points.push_back(goal_tracker_point);
      }

      try {
        publisher_set_trajectory.publish(goal_trajectory_topic);
      }
      catch (...) {
        ROS_ERROR("Exception caught during publishing topic %s.", publisher_set_trajectory.getTopic().c_str());
      }

      wait.sleep();

      //}

      break;

    case TRAJECTORY_FLY_TO_START_TOPIC_STATE:

      //{ test fly_to_start_out service

      service_client_fly_to_start.call(goal_trigger);

      //}

      break;

    case TRAJECTORY_START_FOLLOWING_TOPIC_STATE:

      //{ test start_following_out service

      des_x   = goal_tracker_point.x;
      des_y   = goal_tracker_point.y;
      des_z   = goal_tracker_point.z;
      des_yaw = sanitizeYaw(goal_tracker_point.yaw);

      service_client_start_following.call(goal_trigger);

      //}

      break;

    case TRAJECTORY_LOAD_STATIC_SERVICE_STATE:

      //{ test trajectory loading using service

      activateTracker("mrs_trackers/MpcTracker");

      goal_trajectory_topic.fly_now         = false;
      goal_trajectory_topic.header.frame_id = "local_origin";
      goal_trajectory_topic.header.stamp    = ros::Time::now();
      goal_trajectory_topic.loop            = false;
      goal_trajectory_topic.use_yaw         = true;
      goal_trajectory_topic.start_index     = 0;

      goal_tracker_point.x   = trajectory_p1_;
      goal_tracker_point.y   = trajectory_p1_;
      goal_tracker_point.z   = max_z_;
      goal_tracker_point.yaw = 1.57;
      goal_trajectory_topic.points.push_back(goal_tracker_point);

      des_x   = goal_tracker_point.x;
      des_y   = goal_tracker_point.y;
      des_z   = goal_tracker_point.z;
      des_yaw = sanitizeYaw(goal_tracker_point.yaw);

      trajectory_length = int(1.414*(5*fabs(trajectory_p2_-trajectory_p1_))/(trajectory_speed_));

      for (int i = 0; i < trajectory_length; i++) {

        goal_tracker_point.x += trajectory_speed_/5;
        goal_tracker_point.y += trajectory_speed_/5;
        goal_tracker_point.z -= (max_z_-min_z_)/trajectory_length;
        goal_tracker_point.yaw = sanitizeYaw(goal_tracker_point.yaw - 0.1);
        /* ROS_INFO("[ControlTest]: x %f y %f z %f yaw %f", goal_tracker_point.x, goal_tracker_point.y, goal_tracker_point.z, goal_tracker_point.yaw); */
        goal_trajectory_topic.points.push_back(goal_tracker_point);
      }

      goal_trajectory_srv.request.trajectory_msg = goal_trajectory_topic;

      service_client_trajectory.call(goal_trajectory_srv);

      wait.sleep();

      //}

      break;

    case TRAJECTORY_FLY_TO_START_SERVICE_STATE:

      //{ test fly_to_start_out service

      service_client_fly_to_start.call(goal_trigger);

      //}

      break;

    case TRAJECTORY_START_FOLLOWING_SERVICE_STATE:

      //{ test start_following_out service

      des_x   = goal_tracker_point.x;
      des_y   = goal_tracker_point.y;
      des_z   = goal_tracker_point.z;
      des_yaw = sanitizeYaw(goal_tracker_point.yaw);

      service_client_start_following.call(goal_trigger);

      //}

      break;

    case TRAJECTORY_LOAD_DYNAMIC_TOPIC_STATE:

      //{ test set_trajectory topic with fly_now

      activateTracker("mrs_trackers/MpcTracker");

      goal_trajectory_topic.fly_now         = true;
      goal_trajectory_topic.header.frame_id = "local_origin";
      goal_trajectory_topic.header.stamp    = ros::Time::now();
      goal_trajectory_topic.loop            = false;
      goal_trajectory_topic.use_yaw         = true;
      goal_trajectory_topic.start_index     = 0;

      goal_tracker_point.x   = trajectory_p2_;
      goal_tracker_point.y   = trajectory_p2_;
      goal_tracker_point.z   = min_z_;
      goal_tracker_point.yaw = 0;
      goal_trajectory_topic.points.push_back(goal_tracker_point);

      trajectory_length = int(1.414*(5*fabs(trajectory_p2_-trajectory_p1_))/(trajectory_speed_));

      for (int i = 0; i < trajectory_length; i++) {

        goal_tracker_point.x -= trajectory_speed_/5;
        goal_tracker_point.y -= trajectory_speed_/5;
        goal_tracker_point.z += (max_z_-min_z_)/trajectory_length;
        goal_tracker_point.yaw = sanitizeYaw(goal_tracker_point.yaw + 0.1);
        /* ROS_INFO("[ControlTest]: x %f y %f z %f yaw %f", goal_tracker_point.x, goal_tracker_point.y, goal_tracker_point.z, goal_tracker_point.yaw); */
        goal_trajectory_topic.points.push_back(goal_tracker_point);
      }

      des_x   = goal_tracker_point.x;
      des_y   = goal_tracker_point.y;
      des_z   = goal_tracker_point.z;
      des_yaw = sanitizeYaw(goal_tracker_point.yaw);

      try {
        publisher_set_trajectory.publish(goal_trajectory_topic);
      }
      catch (...) {
        ROS_ERROR("Exception caught during publishing topic %s.", publisher_set_trajectory.getTopic().c_str());
      }

      /* wait.sleep(); */

      //}

      break;

    case TRAJECTORY_LOAD_DYNAMIC_SERVICE_STATE:

      //{ test trajectory loading using service

      activateTracker("mrs_trackers/MpcTracker");

      goal_trajectory_topic.fly_now         = true;
      goal_trajectory_topic.header.frame_id = "local_origin";
      goal_trajectory_topic.header.stamp    = ros::Time::now();
      goal_trajectory_topic.loop            = false;
      goal_trajectory_topic.use_yaw         = true;
      goal_trajectory_topic.start_index     = 0;

      goal_tracker_point.x   = trajectory_p1_;
      goal_tracker_point.y   = trajectory_p1_;
      goal_tracker_point.z   = max_z_;
      goal_tracker_point.yaw = 1.57;
      goal_trajectory_topic.points.push_back(goal_tracker_point);

      trajectory_length = int(1.414*(5*fabs(trajectory_p2_-trajectory_p1_))/(trajectory_speed_));

      for (int i = 0; i < trajectory_length; i++) {

        goal_tracker_point.x += trajectory_speed_/5;
        goal_tracker_point.y += trajectory_speed_/5;
        goal_tracker_point.z -= (max_z_-min_z_)/trajectory_length;
        goal_tracker_point.yaw = sanitizeYaw(goal_tracker_point.yaw - 0.1);
        /* ROS_INFO("[ControlTest]: x %f y %f z %f yaw %f", goal_tracker_point.x, goal_tracker_point.y, goal_tracker_point.z, goal_tracker_point.yaw); */
        goal_trajectory_topic.points.push_back(goal_tracker_point);
      }

      des_x   = goal_tracker_point.x;
      des_y   = goal_tracker_point.y;
      des_z   = goal_tracker_point.z;
      des_yaw = sanitizeYaw(goal_tracker_point.yaw);

      goal_trajectory_srv.request.trajectory_msg = goal_trajectory_topic;

      service_client_trajectory.call(goal_trajectory_srv);

      //}

      break;

    case LAND_HOME_STATE:

      //{ test land_home service

      service_client_land_home.call(goal_trigger);

      //}

      break;

    case GOTO_ORIGIN_STATE:

      //{ go to origin

      activateTracker("mrs_trackers/MpcTracker");

      goal_tracker_point_stamped.position.x   = 0;
      goal_tracker_point_stamped.position.y   = 0;
      goal_tracker_point_stamped.position.z   = 3;
      goal_tracker_point_stamped.position.yaw = 0;

      des_x   = goal_tracker_point_stamped.position.x;
      des_y   = goal_tracker_point_stamped.position.y;
      des_z   = goal_tracker_point_stamped.position.z;
      des_yaw = goal_tracker_point_stamped.position.yaw;

      try {
        publisher_goto.publish(goal_tracker_point_stamped);
      }
      catch (...) {
        ROS_ERROR("Exception caught during publishing topic %s.", publisher_goto.getTopic().c_str());
      }

      //}

      break;

    case LAND_STATE:

      //{ test land service

      service_client_land.call(goal_trigger);

      //}

      break;

    case FINISHED_STATE:

      //{ finish and end the node

      ROS_INFO(" ");
      ROS_INFO("[ControlTest]: TEST FINISHED");
      ros::shutdown();

      //}

      break;
  }

  timeout = ros::Time::now();
}

//}

//{ randd()

double ControlTest::randd(double from, double to) {

  double zero_to_one = double((float)rand()) / double(RAND_MAX);

  return floor(to - from) * zero_to_one + from;
}

//}

//{ genYaw()

double ControlTest::genYaw(void) {

  return randd(min_yaw_, max_yaw_);
}

//}

//{ genXY()

double ControlTest::genXY(void) {

  return randd(min_xy_, max_xy_);
}

//}

//{ genZ()

double ControlTest::genZ(void) {

  return randd(min_z_, max_z_);
}

//}

//{ dist3d()

double ControlTest::dist3d(double x1, double x2, double y1, double y2, double z1, double z2) {

  return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2) + pow(z1 - z2, 2));
}

//}

//{ dist2d()

double ControlTest::dist2d(double x1, double x2, double y1, double y2) {

  return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
}

//}

//{ sanitizeYaw()

double ControlTest::sanitizeYaw(const double yaw_in) {

  double yaw_out = yaw_in;

  // if desired yaw_out is grater then 2*PI mod it
  if (fabs(yaw_out) > 2 * PI) {
    yaw_out = fmod(yaw_out, 2 * PI);
  }

  // move it to its place
  if (yaw_out > PI) {
    yaw_out -= 2 * PI;
  } else if (yaw_out < -PI) {
    yaw_out += 2 * PI;
  }

  return yaw_out;
}

//}

//{ inDesiredState()

bool ControlTest::inDesiredState(void) {

  mutex_odometry.lock();
  {
    if (dist3d(odometry_x, des_x, odometry_y, des_y, odometry_z, des_z) < 0.15 && fabs(sanitizeYaw(odometry_yaw) - sanitizeYaw(des_yaw)) < 0.15) {
      mutex_odometry.unlock();
      ROS_WARN("[ControlTest]: The goal has been reached.");
      ros::Duration(1.0).sleep();
      return true;
    }
  }
  mutex_odometry.unlock();

  return false;
}

//}

//{ activateTracker()

void ControlTest::activateTracker(std::string tracker_name) {

  mrs_msgs::String goal_switch_tracker;
  goal_switch_tracker.request.value = tracker_name;

  ROS_INFO("[ControlTest]: switching to %s", goal_switch_tracker.request.value.c_str());
  service_client_switch_tracker.call(goal_switch_tracker);
  ros::Duration wait(1.0);
  wait.sleep();
}

//}

}  // namespace mrs_testing

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_testing::ControlTest, nodelet::Nodelet)
