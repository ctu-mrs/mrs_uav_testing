#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <std_srvs/Trigger.h>
#include <std_msgs/Float64.h>

#include <mrs_msgs/TrackerPointStamped.h>
#include <mrs_msgs/TrackerTrajectory.h>
#include <mrs_msgs/TrackerTrajectorySrv.h>
#include <mrs_msgs/Vec4.h>
#include <mrs_msgs/Vec1.h>

#include <mrs_msgs/SwitchTracker.h>
#include <mrs_msgs/PositionCommand.h>

#include <nav_msgs/Odometry.h>

#include <mutex>

#include <tf/transform_datatypes.h>

#define PI 3.141592653

namespace mrs_testing
{

//{ state machine states

// state machine
typedef enum
{
  IDLE_STATE                     = 0,
  GOTO_TOPIC_STATE               = 1,
  GOTO_RELATIVE_TOPIC_STATE      = 2,
  GOTO_ALTITUDE_TOPIC_STATE      = 3,
  SET_YAW_TOPIC_STATE            = 4,
  SET_YAW_RELATIVE_TOPIC_STATE   = 5,
  GOTO_SERVICE_STATE             = 6,
  GOTO_RELATIVE_SERVICE_STATE    = 7,
  GOTO_ALTITUDE_SERVICE_STATE    = 8,
  SET_YAW_SERVICE_STATE          = 9,
  SET_YAW_RELATIVE_SERVICE_STATE = 10,
} ControlState_t;

const char *state_names[11] = {
    "IDLE_STATE",
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
  ros::Publisher publisher_goto;
  ros::Publisher publisher_goto_relative;
  ros::Publisher publisher_goto_altitude;
  ros::Publisher publisher_set_yaw;
  ros::Publisher publisher_set_yaw_relative;
  ros::Publisher publisher_set_trajectory;

private:
  ros::ServiceClient service_client_switch_tracker;

  ros::ServiceClient service_client_goto;
  ros::ServiceClient service_client_goto_relative;
  ros::ServiceClient service_client_goto_altitude;
  ros::ServiceClient service_client_set_yaw;
  ros::ServiceClient service_client_set_yaw_relative;
  ros::ServiceClient service_client_trajectory;

private:
  void callbackOdometry(const nav_msgs::OdometryConstPtr &msg);
  void callbackPositionCommand(const mrs_msgs::PositionCommandConstPtr &msg);

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
  double randd(double from, double to);
  double genYaw(void);
  double genXY(void);
  double genZ(void);
  double sanitizeYaw(const double yaw_in);
  bool   inDesiredState(void);

  int active_tracker = -1;

  // | -------------------- for goto testing -------------------- |
private:
  double des_x, des_y, des_z, des_yaw;
};

//}

//{ onInit()

void ControlTest::onInit() {

  ros::NodeHandle nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  ros::Time::waitForValid();

  srand(static_cast<unsigned int>(ros::Time::now().toSec()));

  // --------------------------------------------------------------
  // |                           params                           |
  // --------------------------------------------------------------

  nh_.param("max_xy", max_xy_, -999.0);
  nh_.param("min_xy", max_xy_, -999.0);
  nh_.param("max_z", max_z_, -999.0);
  nh_.param("min_z", min_z_, -999.0);
  nh_.param("max_yaw", max_yaw_, -999.0);
  nh_.param("min_yaw", min_yaw_, -999.0);

  if (max_xy_ < -999) {
    ROS_ERROR("[ControlTest]: max_xy was not specified!");
    ros::shutdown();
  }

  if (min_xy_ < -999) {
    ROS_ERROR("[ControlTest]: min_xy was not specified!");
    ros::shutdown();
  }

  if (max_z_ < -999) {
    ROS_ERROR("[ControlTest]: max_z was not specified!");
    ros::shutdown();
  }

  if (min_z_ < -999) {
    ROS_ERROR("[ControlTest]: min_z was not specified!");
    ros::shutdown();
  }

  if (max_yaw_ < -999) {
    ROS_ERROR("[ControlTest]: max_yaw was not specified!");
    ros::shutdown();
  }

  if (min_yaw_ < -999) {
    ROS_ERROR("[ControlTest]: min_yaw was not specified!");
    ros::shutdown();
  }

  // --------------------------------------------------------------
  // |                         subscribers                        |
  // --------------------------------------------------------------

  subscriber_odometry         = nh_.subscribe("odometry_in", 1, &ControlTest::callbackOdometry, this, ros::TransportHints().tcpNoDelay());
  subscriber_position_command = nh_.subscribe("position_command_in", 1, &ControlTest::callbackPositionCommand, this, ros::TransportHints().tcpNoDelay());

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

  service_client_switch_tracker = nh_.serviceClient<mrs_msgs::SwitchTracker>("switch_tracker_out");

  // | ------------------ std tracker services ------------------ |

  service_client_goto             = nh_.serviceClient<mrs_msgs::Vec4>("goto_out");
  service_client_goto_relative    = nh_.serviceClient<mrs_msgs::Vec4>("goto_relative_out");
  service_client_goto_altitude    = nh_.serviceClient<mrs_msgs::Vec1>("goto_altitude_out");
  service_client_set_yaw          = nh_.serviceClient<mrs_msgs::Vec1>("set_yaw_out");
  service_client_set_yaw_relative = nh_.serviceClient<mrs_msgs::Vec1>("set_yaw_relative");

  // | -------------- additional trackers services -------------- |

  service_client_trajectory = nh_.serviceClient<mrs_msgs::SwitchTracker>("set_trajectory_out");

  // --------------------------------------------------------------
  // |                           timers                           |
  // --------------------------------------------------------------

  ROS_INFO("[ControlTest]: initialized");

  main_timer = nh_.createTimer(ros::Rate(10), &ControlTest::mainTimer, this);

  is_initialized = true;
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

// --------------------------------------------------------------
// |                           timers                           |
// --------------------------------------------------------------

//{ mainTimer()

void ControlTest::mainTimer(const ros::TimerEvent &event) {

  if (!is_initialized)
    return;

  if (!got_odometry || !got_position_command) {
    ROS_INFO_THROTTLE(1.0, "[ControlTest]: waiting for data");
    return;
  }

  ROS_INFO_THROTTLE(1.0, "");
  ROS_WARN_THROTTLE(1.0, "[ControlTest]: current state: %s", state_names[current_state]);
  ROS_INFO_THROTTLE(1.0, "[ControlTest]: dessired: %f %f %f %f", des_x, des_y, des_z, des_yaw);
  ROS_INFO_THROTTLE(1.0, "[ControlTest]: cmd: %f %f %f %f", cmd_x, cmd_y, cmd_z, sanitizeYaw(cmd_yaw));
  ROS_INFO_THROTTLE(1.0, "[ControlTest]: odom: %f %f %f %f", odometry_x, odometry_y, odometry_z, sanitizeYaw(odometry_yaw));

  switch (current_state) {

    case IDLE_STATE:

      changeState(ControlState_t(int(current_state) + 1));
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

        if (active_tracker == 0) {
          changeState(GOTO_TOPIC_STATE);
        } else if (active_tracker == 1) {
          ROS_INFO("");
          ROS_INFO("[ControlTest]: TEST FINISHED");
          ros::shutdown();
        }
      }
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

  mrs_msgs::TrackerPointStamped goal_tracker_point_stamped;
  std_msgs::Float64             goal_float64;
  mrs_msgs::Vec4                goal_vec4;
  mrs_msgs::Vec1                goal_vec1;
  mrs_msgs::SwitchTracker       switch_tracker;

  switch (new_state) {

    case IDLE_STATE:
      break;

    case GOTO_TOPIC_STATE:

      if (active_tracker == -1) {
        switch_tracker.request.tracker = "mrs_trackers/LineTracker";
      } else if (active_tracker == 0) {
        switch_tracker.request.tracker = "mrs_trackers/MpcTracker";
      }
      active_tracker++;
      ROS_INFO("[ControlTest]: switching to %s", switch_tracker.request.tracker.c_str());
      service_client_switch_tracker.call(switch_tracker);

      goal_tracker_point_stamped.position.x   = genXY();
      goal_tracker_point_stamped.position.y   = genXY();
      goal_tracker_point_stamped.position.z   = genZ();
      goal_tracker_point_stamped.position.yaw = genYaw();

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

      ROS_INFO("[ControlTest]: testing goto");

      break;

    case GOTO_RELATIVE_TOPIC_STATE:

      goal_tracker_point_stamped.position.x   = genXY();
      goal_tracker_point_stamped.position.y   = genXY();
      goal_tracker_point_stamped.position.z   = randd(-2, 2);
      goal_tracker_point_stamped.position.yaw = genYaw();

      mutex_odometry.lock();
      {
        des_x   = goal_tracker_point_stamped.position.x + odometry_x;
        des_y   = goal_tracker_point_stamped.position.y + odometry_y;
        des_z   = goal_tracker_point_stamped.position.z + odometry_z;
        des_yaw = goal_tracker_point_stamped.position.yaw + odometry_yaw;
      }
      mutex_odometry.unlock();

      try {
        publisher_goto_relative.publish(goal_tracker_point_stamped);
      }
      catch (...) {
        ROS_ERROR("Exception caught during publishing topic %s.", publisher_goto_relative.getTopic().c_str());
      }

      ROS_INFO("[ControlTest]: testing goto_relative");

      break;

    case GOTO_ALTITUDE_TOPIC_STATE:

      goal_float64.data = genZ();

      des_x   = cmd_x;
      des_y   = cmd_y;
      des_z   = goal_float64.data;
      des_yaw = cmd_yaw;

      try {
        publisher_goto_altitude.publish(goal_float64);
      }
      catch (...) {
        ROS_ERROR("Exception caught during publishing topic %s.", publisher_goto_altitude.getTopic().c_str());
      }

      ROS_INFO("[ControlTest]: testing goto_altitude");

      break;

    case SET_YAW_TOPIC_STATE:

      goal_float64.data = genYaw();

      des_x   = cmd_x;
      des_y   = cmd_y;
      des_z   = cmd_z;
      des_yaw = goal_float64.data;

      try {
        publisher_set_yaw.publish(goal_float64);
      }
      catch (...) {
        ROS_ERROR("Exception caught during publishing topic %s.", publisher_set_yaw.getTopic().c_str());
      }

      break;

    case SET_YAW_RELATIVE_TOPIC_STATE:

      goal_float64.data = genYaw();

      des_x   = cmd_x;
      des_y   = cmd_y;
      des_z   = cmd_z;
      des_yaw = cmd_yaw + goal_float64.data;

      publisher_set_yaw_relative.publish(goal_float64);

      break;

    case GOTO_SERVICE_STATE:

      goal_vec4.request.goal[0] = genXY();
      goal_vec4.request.goal[1] = genXY();
      goal_vec4.request.goal[2] = genZ();
      goal_vec4.request.goal[3] = genYaw();

      des_x   = goal_vec4.request.goal[0];
      des_y   = goal_vec4.request.goal[1];
      des_z   = goal_vec4.request.goal[2];
      des_yaw = goal_vec4.request.goal[3];

      service_client_goto.call(goal_vec4);

      break;

    case GOTO_RELATIVE_SERVICE_STATE:

      goal_vec4.request.goal[0] = genXY();
      goal_vec4.request.goal[1] = genXY();
      goal_vec4.request.goal[2] = randd(-2, 2);
      goal_vec4.request.goal[3] = genYaw();

      des_x   = cmd_x + goal_vec4.request.goal[0];
      des_y   = cmd_y + goal_vec4.request.goal[1];
      des_z   = cmd_z + goal_vec4.request.goal[2];
      des_yaw = cmd_yaw + goal_vec4.request.goal[3];

      service_client_goto_relative.call(goal_vec4);

      break;

    case GOTO_ALTITUDE_SERVICE_STATE:

      goal_vec1.request.goal = genZ();

      des_x   = cmd_x;
      des_y   = cmd_y;
      des_z   = goal_vec1.request.goal;
      des_yaw = cmd_yaw;

      service_client_goto_altitude.call(goal_vec1);

      break;

    case SET_YAW_SERVICE_STATE:

      goal_vec1.request.goal = genYaw();

      des_x   = cmd_x;
      des_y   = cmd_y;
      des_z   = cmd_z;
      des_yaw = goal_vec1.request.goal;

      service_client_set_yaw.call(goal_vec1);

      break;

    case SET_YAW_RELATIVE_SERVICE_STATE:

      goal_vec1.request.goal = genYaw();

      des_x   = cmd_x;
      des_y   = cmd_y;
      des_z   = cmd_z;
      des_yaw = cmd_yaw + goal_vec1.request.goal;

      service_client_set_yaw_relative.call(goal_vec1);

      break;
  }
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
    if (dist3d(odometry_x, des_x, odometry_y, des_y, odometry_z, des_z) < 0.2 && fabs(sanitizeYaw(odometry_yaw) - sanitizeYaw(des_yaw)) < 0.2) {
      mutex_odometry.unlock();
      ROS_WARN("[ControlTest]: We are at the goal!");
      return true;
    }
  }
  mutex_odometry.unlock();

  return false;
}

//}

}  // namespace mrs_testing

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_testing::ControlTest, nodelet::Nodelet)
