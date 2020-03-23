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
#include <mrs_msgs/ReferenceStampedSrv.h>

#include <mrs_lib/ParamLoader.h>

#include <std_srvs/Trigger.h>

#include <random>

//}

namespace mrs_testing
{

/* class RandomFlier //{ */

class RandomFlier : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  bool is_initialized_ = false;

  void   callbackControlCmd(const nav_msgs::OdometryConstPtr& msg);
  bool   callbackActivate([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  void   timerMain(const ros::TimerEvent& event);
  double randd(double from, double to);

  ros::Subscriber subscriber_control_cmd_;

  ros::ServiceServer service_server_activate_;

  ros::ServiceClient service_client_reference_;

  ros::Timer main_timer_;

  double _main_timer_rate_;

  // parameters loaded from config file
  double _height_;
  bool   _randomize_distance_ = false;
  double _max_distance_;

  bool active_ = true;

  nav_msgs::Odometry control_cmd_;
  std::mutex         mutex_control_cmd_;
  bool               got_control_cmd_ = false;

  ros::Time last_successfull_command_;
};

//}

/* onInit() //{ */

void RandomFlier::onInit(void) {

  ros::NodeHandle nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  ros::Time::waitForValid();

  mrs_lib::ParamLoader param_loader(nh_, "RandomFlier");

  // load parameters from config file
  param_loader.load_param("main_timer_rate", _main_timer_rate_);
  param_loader.load_param("height", _height_);
  param_loader.load_param("active", active_);
  param_loader.load_param("randomize_distance", _randomize_distance_);
  param_loader.load_param("max_distance", _max_distance_);

  if (!param_loader.loaded_successfully()) {
    ROS_ERROR("[RandomFlier]: Could not load all parameters!");
    ros::shutdown();
  }

  subscriber_control_cmd_ = nh_.subscribe("control_cmd_in", 1, &RandomFlier::callbackControlCmd, this, ros::TransportHints().tcpNoDelay());

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

// --------------------------------------------------------------
// |                          callbacks                         |
// --------------------------------------------------------------

/* callbackControlCmd() //{ */

void RandomFlier::callbackControlCmd(const nav_msgs::OdometryConstPtr& msg) {

  if (!is_initialized_) {
    return;
  }

  std::scoped_lock lock(mutex_control_cmd_);

  control_cmd_ = *msg;

  got_control_cmd_ = true;

  ROS_INFO_ONCE("[RandomFlier]: getting control_cmd");
}

//}

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

// --------------------------------------------------------------
// |                           timers                           |
// --------------------------------------------------------------

/* timerMain() //{ */

void RandomFlier::timerMain([[maybe_unused]] const ros::TimerEvent& event) {

  if (!is_initialized_) {
    return;
  }

  if (!active_) {

    ROS_INFO_THROTTLE(1.0, "[RandomFlier]: waiting for activation");
    return;
  }

  if (!got_control_cmd_) {

    ROS_INFO_THROTTLE(1.0, "[RandomFlier]: waiting for data");
    return;
  }

  // keeps ros in the loop
  {
    std::scoped_lock lock(mutex_control_cmd_);

    // if the uav reach the previousy set destination
    if ((ros::Time::now() - last_successfull_command_).toSec() > 1.0 && fabs(control_cmd_.twist.twist.linear.x) < 0.01 &&
        fabs(control_cmd_.twist.twist.linear.y) < 0.01) {

      // create new point to fly to
      mrs_msgs::ReferenceStampedSrv new_point;
      new_point.request.header.frame_id = "gps_origin";

      double dist, direction;

      if (_randomize_distance_) {
        dist = randd(0, _max_distance_);
      } else {
        dist = _max_distance_;
      }

      direction = randd(-M_PI, M_PI);

      while (true) {

        new_point.request.reference.position.x = control_cmd_.pose.pose.position.x + cos(direction) * dist;
        new_point.request.reference.position.y = control_cmd_.pose.pose.position.y + sin(direction) * dist;
        new_point.request.reference.position.z = _height_;
        new_point.request.reference.yaw        = 0;

        if (service_client_reference_.call(new_point)) {

          if (new_point.response.success) {

            ROS_INFO("New goal: %.2f %.2f", new_point.request.reference.position.x, new_point.request.reference.position.y);

            last_successfull_command_ = ros::Time::now();
            break;

          } else {

            /* ROS_WARN("New goal: %2.2f %2.2f failed", new_point.request.goal[0], new_point.request.goal[1]); */

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

double RandomFlier::randd(double from, double to) {

  double zero_to_one = double((float)rand()) / double(RAND_MAX);

  return floor(to - from) * zero_to_one + from;
}

//}

}  // namespace mrs_testing

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_testing::RandomFlier, nodelet::Nodelet)
