#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>

#include <mrs_msgs/ReferenceStamped.h>
#include <mrs_msgs/TrackerPoint.h>
#include <mrs_msgs/TrackerTrajectory.h>
#include <mrs_msgs/TrackerTrajectorySrv.h>
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

/* //{ state machine states */

//}

/* //{ class LoopingTest */

class LoopingTest : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  ros::NodeHandle nh_;
  bool            is_initialized = false;

  std::string uav_name_;

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

  ros::ServiceClient service_client_trajectory;

private:
  double radius_;
  double speed_;
  double ramp_length_;
  void   loadTrajectory();

private:
  void callbackOdometry(const nav_msgs::OdometryConstPtr &msg);
  void callbackPositionCommand(const mrs_msgs::PositionCommandConstPtr &msg);
  void callbackTrackerStatus(const mrs_msgs::TrackerStatusConstPtr &msg);

  double sanitizeYaw(const double yaw_in);

private:
  double trajectory_p1_, trajectory_p2_, trajectory_speed_;
};

//}

/* //{ onInit() */

void LoopingTest::onInit() {

  ros::NodeHandle nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  ros::Time::waitForValid();

  srand(static_cast<unsigned int>(ros::Time::now().toSec()));

  // --------------------------------------------------------------
  // |                           params                           |
  // --------------------------------------------------------------

  mrs_lib::ParamLoader param_loader(nh_, "LoopingTest");

  param_loader.load_param("uav_name", uav_name_);

  param_loader.load_param("radius", radius_);
  param_loader.load_param("speed", speed_);
  param_loader.load_param("ramp_length", ramp_length_);

  // --------------------------------------------------------------
  // |                         subscribers                        |
  // --------------------------------------------------------------

  subscriber_odometry         = nh_.subscribe("odometry_in", 1, &LoopingTest::callbackOdometry, this, ros::TransportHints().tcpNoDelay());
  subscriber_position_command = nh_.subscribe("position_command_in", 1, &LoopingTest::callbackPositionCommand, this, ros::TransportHints().tcpNoDelay());
  subscriber_tracker_status   = nh_.subscribe("tracker_status_in", 1, &LoopingTest::callbackTrackerStatus, this, ros::TransportHints().tcpNoDelay());

  // --------------------------------------------------------------
  // |                         publishers                         |
  // --------------------------------------------------------------

  // --------------------------------------------------------------
  // |                          services                          |
  // --------------------------------------------------------------

  // | -------------- additional trackers services -------------- |

  service_client_trajectory = nh_.serviceClient<mrs_msgs::TrackerTrajectorySrv>("set_trajectory_out");

  // | ---------------------- finish inint ---------------------- |

  if (!param_loader.loaded_successfully()) {
    ros::shutdown();
  }

  is_initialized = true;

  ROS_INFO("[LoopingTest]: initialized");

  ros::Duration pes(1.0);
  pes.sleep();

  loadTrajectory();
}

//}

// --------------------------------------------------------------
// |                          callbacks                         |
// --------------------------------------------------------------

/* //{ callbackOdometry() */

void LoopingTest::callbackOdometry(const nav_msgs::OdometryConstPtr &msg) {

  if (!is_initialized)
    return;

  got_odometry = true;

  {
    std::scoped_lock lock(mutex_odometry);

    odometry   = *msg;
    odometry_x = odometry.pose.pose.position.x;
    odometry_y = odometry.pose.pose.position.y;
    odometry_z = odometry.pose.pose.position.z;

    // calculate the euler angles
    tf::Quaternion quaternion_odometry;
    quaternionMsgToTF(odometry.pose.pose.orientation, quaternion_odometry);
    tf::Matrix3x3 m(quaternion_odometry);
    m.getRPY(odometry_roll, odometry_pitch, odometry_yaw);
  }
}

//}

/* //{ callbackPositionCommand() */

void LoopingTest::callbackPositionCommand(const mrs_msgs::PositionCommandConstPtr &msg) {

  if (!is_initialized)
    return;

  got_position_command = true;

  {
    std::scoped_lock lock(mutex_cmd);

    position_command = *msg;

    cmd_x   = position_command.position.x;
    cmd_y   = position_command.position.y;
    cmd_z   = position_command.position.z;
    cmd_yaw = position_command.yaw;
  }
}

//}

/* //{ callbackTrackerStatus() */

void LoopingTest::callbackTrackerStatus(const mrs_msgs::TrackerStatusConstPtr &msg) {

  if (!is_initialized)
    return;

  got_tracker_status = true;

  {
    std::scoped_lock lock(mutex_tracker_status);
    tracker_status = *msg;
  }
}

//}

// --------------------------------------------------------------
// |                       other methodsd                       |
// --------------------------------------------------------------

void LoopingTest::loadTrajectory() {

  mrs_msgs::TrackerTrajectorySrv goal_trajectory_srv;
  mrs_msgs::TrackerTrajectory    goal_trajectory_topic;
  [[maybe_unused]] double        trajectory_length;

  goal_trajectory_topic.fly_now         = true;
  goal_trajectory_topic.header.frame_id = uav_name_ + "/local_origin";
  goal_trajectory_topic.header.stamp    = ros::Time::now();
  goal_trajectory_topic.loop            = false;
  goal_trajectory_topic.use_yaw         = false;

  mrs_msgs::TrackerPoint point;

  point.x   = odometry_x;
  point.y   = odometry_y;
  point.z   = odometry_z;
  point.yaw = odometry_yaw;
  goal_trajectory_topic.points.push_back(point);

  double ramp_pts = (ramp_length_ / speed_) * 5.0;

  // generate the initial ramp
  for (int i = 0; i < ramp_pts; i++) {

    point.x += speed_ / 5.0;
    goal_trajectory_topic.points.push_back(point);
  }

  double                  circle_center_x = point.x;
  [[maybe_unused]] double circle_center_y = point.y;
  double                  circle_center_z = point.z + radius_;

  double angular_speed = speed_ / radius_;
  double circle_pts    = (2.0 * PI / angular_speed) * 5.0;

  double angle = -PI / 2.0;

  // generate the circle of death
  // generate the initial ramp
  for (int i = 0; i < circle_pts; i++) {

    point.x = circle_center_x + radius_ * cos(angle);
    point.z = circle_center_z + radius_ * sin(angle);
    goal_trajectory_topic.points.push_back(point);

    angle += angular_speed / 5.0;
  }

  // generate the final ramp
  for (int i = 0; i < ramp_pts; i++) {

    point.x += speed_ / 5.0;
    goal_trajectory_topic.points.push_back(point);
  }

  goal_trajectory_srv.request.trajectory_msg = goal_trajectory_topic;

  service_client_trajectory.call(goal_trajectory_srv);
}

/* //{ sanitizeYaw() */

double LoopingTest::sanitizeYaw(const double yaw_in) {

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

}  // namespace mrs_testing

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_testing::LoopingTest, nodelet::Nodelet)
