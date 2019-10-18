/* includes //{ */

// some ros includes
#include <ros/ros.h>
#include <nodelet/nodelet.h>

// some std includes
#include <stdio.h>
#include <stdlib.h>

// mutexes are good to keep your shared variables our of trouble
#include <mutex>

#include <nav_msgs/Odometry.h>
#include <mrs_msgs/Vec4.h>
#include <mrs_msgs/TrackerTrajectorySrv.h>  // ros message to set desired goal
#include <mrs_msgs/TrackerPoint.h>          // ros message to set desired goal

#include <mrs_lib/ParamLoader.h>

#include <std_srvs/Trigger.h>

#include <random>

//}

namespace mrs_testing
{

/* class TrajectoryFlier //{ */

class TrajectoryFlier : public nodelet::Nodelet {

public:
  virtual void onInit();

  bool is_initialized = false;

private:
  void   callbackControlCmd(const nav_msgs::OdometryConstPtr& msg);
  bool   callbackActivate([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  void   mainTimer(const ros::TimerEvent& event);
  double randd(double from, double to);

private:
  ros::Subscriber    subscriber_control_cmd;
  ros::Publisher     publisher_goto;
  ros::ServiceServer service_server_activate;
  ros::ServiceClient service_client_trajectory;
  ros::Timer         main_timer;

  double main_timer_rate_;

  // parameters loaded from config file
  double x_min_, x_max_, y_min_, y_max_;
  double height_;
  bool   single_d_           = false;
  bool   active              = true;
  bool   randomize_distance_ = false;
  double speed_;
  double acceleration_;
  double jerk_;
  double max_distance_;

  nav_msgs::Odometry control_cmd;
  std::mutex         mutex_control_cmd;
  bool               got_control_cmd = false;

  ros::Time last_successfull_command;
};

//}

/* onInit() //{ */

void TrajectoryFlier::onInit(void) {

  ros::NodeHandle nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  ros::Time::waitForValid();

  mrs_lib::ParamLoader param_loader(nh_, "TrajectoryFlier");

  // load parameters from config file
  param_loader.load_param("main_timer_rate", main_timer_rate_);
  param_loader.load_param("height", height_);
  param_loader.load_param("active", active);
  param_loader.load_param("randomize_distance", randomize_distance_);
  param_loader.load_param("speed", speed_);
  param_loader.load_param("acceleration", acceleration_);
  param_loader.load_param("jerk", jerk_);
  param_loader.load_param("max_distance", max_distance_);

  subscriber_control_cmd = nh_.subscribe("control_cmd_in", 1, &TrajectoryFlier::callbackControlCmd, this, ros::TransportHints().tcpNoDelay());

  service_server_activate   = nh_.advertiseService("activate_in", &TrajectoryFlier::callbackActivate, this);
  service_client_trajectory = nh_.serviceClient<mrs_msgs::TrackerTrajectorySrv>("trajectory_out");

  // initialize the random number generator
  /* srand(static_cast<unsigned int>(time(0))); */
  srand(time(NULL));

  last_successfull_command = ros::Time(0);

  main_timer = nh_.createTimer(ros::Rate(main_timer_rate_), &TrajectoryFlier::mainTimer, this);

  // | ----------------------- finish init ---------------------- |

  if (!param_loader.loaded_successfully()) {
    ROS_ERROR("[UavManager]: Could not load all parameters!");
    ros::shutdown();
  }

  is_initialized = true;

  ROS_INFO("[TrajectoryFlier]: initialized");
}

//}

// --------------------------------------------------------------
// |                          callbacks                         |
// --------------------------------------------------------------

/* callbackControlCmd() //{ */

void TrajectoryFlier::callbackControlCmd(const nav_msgs::OdometryConstPtr& msg) {

  if (!is_initialized) {
    return;
  }

  std::scoped_lock lock(mutex_control_cmd);

  control_cmd = *msg;

  got_control_cmd = true;

  ROS_INFO_ONCE("[TrajectoryFlier]: getting control_cmd");
}

//}

/* callbackActivate() //{ */

bool TrajectoryFlier::callbackActivate([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {

  if (!is_initialized) {
    return false;
  }

  active = true;

  res.success = true;
  res.message = "home reseted";

  return true;
}

//}

// --------------------------------------------------------------
// |                           timers                           |
// --------------------------------------------------------------

/* mainTimer() //{ */

void TrajectoryFlier::mainTimer([[maybe_unused]] const ros::TimerEvent& event) {

  if (!is_initialized) {
    return;
  }

  if (!active) {

    ROS_INFO("waiting for activation");
    return;
  }

  if (!got_control_cmd) {

    ROS_INFO_THROTTLE(1.0, "waiting for data");
    return;
  }

  // keeps ros in the loop
  {
    std::scoped_lock lock(mutex_control_cmd);

    // if the uav reach the previousy set destination
    if ((ros::Time::now() - last_successfull_command).toSec() > 1.0 && fabs(control_cmd.twist.twist.linear.x) < 0.01 &&
        fabs(control_cmd.twist.twist.linear.y) < 0.01) {

      // create new point to fly to
      mrs_msgs::TrackerTrajectorySrv trajectory;
      trajectory.request.trajectory_msg.fly_now = true;
      trajectory.request.trajectory_msg.use_yaw = false;
      trajectory.request.trajectory_msg.use_yaw = false;

      double dist, direction;

      if (randomize_distance_) {
        dist = randd(0, max_distance_);
      } else {
        dist = max_distance_;
      }

      direction = randd(-M_PI, M_PI);

      int n_points = (dist / speed_) * 5.0;

      double speed        = 0;
      double acceleration = 0;
      double pos_x        = control_cmd.pose.pose.position.x;
      double pos_y        = control_cmd.pose.pose.position.y;

      for (int it = 0; it < n_points; it++) {

        acceleration += (jerk_ * 0.2);

        if (acceleration >= acceleration_) {
          acceleration = acceleration_;
        }

        speed += (acceleration * 0.2);

        if (speed >= speed_) {
          speed = speed_;
        }

        ROS_INFO("[TrajectoryFlier]: jerk %.2f acceleration %.2f speed %.2f", jerk_, acceleration, speed);

        pos_x += cos(direction) * (speed * 0.2);
        pos_y += sin(direction) * (speed * 0.2);

        mrs_msgs::TrackerPoint new_point;
        new_point.x   = pos_x;
        new_point.y   = pos_y;
        new_point.z   = height_;
        new_point.yaw = 0;

        trajectory.request.trajectory_msg.points.push_back(new_point);
      }

      if (service_client_trajectory.call(trajectory)) {

        if (trajectory.response.success) {

          ROS_INFO("[TrajectoryFlier]: trajectory set");

          last_successfull_command = ros::Time::now();
        }
      }
    }
  }
}

//}

// --------------------------------------------------------------
// |                          routines                          |
// --------------------------------------------------------------

/* randd() //{ */

double TrajectoryFlier::randd(double from, double to) {

  double zero_to_one = double((float)rand()) / double(RAND_MAX);

  return floor(to - from) * zero_to_one + from;
}

//}

}  // namespace mrs_testing

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_testing::TrajectoryFlier, nodelet::Nodelet)
