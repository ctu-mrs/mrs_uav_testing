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
#include <mrs_msgs/TrajectoryReferenceSrv.h>

#include <mrs_lib/ParamLoader.h>

#include <std_srvs/Trigger.h>

#include <random>

//}

namespace mrs_testing
{

/* class TrajectoryRandomFlier //{ */

class TrajectoryRandomFlier : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  bool is_initialized_ = false;

  void   callbackControlCmd(const nav_msgs::OdometryConstPtr& msg);
  bool   callbackActivate([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  void   timerMain(const ros::TimerEvent& event);
  double randd(double from, double to);
  bool   setTrajectorySrv(const mrs_msgs::TrajectoryReference trajectory);

  ros::Subscriber subscriber_control_cmd_;

  ros::Publisher publisher_goto_;

  ros::ServiceServer service_server_activate_;

  ros::ServiceClient service_client_trajectory_;

  ros::Timer timer_main_;

  double _main_timer_rate_;

  // parameters loaded from config file
  double _height_;
  double _speed_;
  double _acceleration_;
  double _jerk_;
  double _max_distance_;
  double _trajectory_dt_;
  bool   _randomize_distance_ = false;

  bool active_ = true;

  nav_msgs::Odometry control_cmd;
  std::mutex         mutex_control_cmd_;
  bool               got_control_cmd_ = false;

  ros::Time last_successfull_command_;
};

//}

/* onInit() //{ */

void TrajectoryRandomFlier::onInit(void) {

  ros::NodeHandle nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  ros::Time::waitForValid();

  mrs_lib::ParamLoader param_loader(nh_, "TrajectoryRandomFlier");

  // load parameters from config file
  param_loader.load_param("main_timer_rate", _main_timer_rate_);
  param_loader.load_param("height", _height_);
  param_loader.load_param("active", active_);
  param_loader.load_param("randomize_distance", _randomize_distance_);
  param_loader.load_param("speed", _speed_);
  param_loader.load_param("acceleration", _acceleration_);
  param_loader.load_param("jerk", _jerk_);
  param_loader.load_param("max_distance", _max_distance_);
  param_loader.load_param("trajectory_dt", _trajectory_dt_);

  if (!param_loader.loaded_successfully()) {
    ROS_ERROR("[TrajectoryRandomFlier]: Could not load all parameters!");
    ros::shutdown();
  }

  subscriber_control_cmd_ = nh_.subscribe("control_cmd_in", 1, &TrajectoryRandomFlier::callbackControlCmd, this, ros::TransportHints().tcpNoDelay());

  service_server_activate_   = nh_.advertiseService("activate_in", &TrajectoryRandomFlier::callbackActivate, this);
  service_client_trajectory_ = nh_.serviceClient<mrs_msgs::TrajectoryReferenceSrv>("trajectory_reference_out");

  // initialize the random number generator
  /* srand(static_cast<unsigned int>(time(0))); */
  srand(time(NULL));

  last_successfull_command_ = ros::Time(0);

  timer_main_ = nh_.createTimer(ros::Rate(_main_timer_rate_), &TrajectoryRandomFlier::timerMain, this);

  // | ----------------------- finish init ---------------------- |

  is_initialized_ = true;

  ROS_INFO("[TrajectoryRandomFlier]: initialized");
}

//}

// | ------------------------ callbacks ----------------------- |

/* callbackControlCmd() //{ */

void TrajectoryRandomFlier::callbackControlCmd(const nav_msgs::OdometryConstPtr& msg) {

  if (!is_initialized_) {
    return;
  }

  std::scoped_lock lock(mutex_control_cmd_);

  control_cmd = *msg;

  got_control_cmd_ = true;

  ROS_INFO_ONCE("[TrajectoryRandomFlier]: getting control_cmd");
}

//}

/* callbackActivate() //{ */

bool TrajectoryRandomFlier::callbackActivate([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {

  if (!is_initialized_) {
    return false;
  }

  active_ = true;

  res.success = true;
  res.message = "activated";

  return true;
}

//}

// | ------------------------- timers ------------------------- |

/* timerMain() //{ */

void TrajectoryRandomFlier::timerMain([[maybe_unused]] const ros::TimerEvent& event) {

  if (!is_initialized_) {
    return;
  }

  if (!active_) {

    ROS_INFO_ONCE("[TrajectoryRandomFlier]: waiting for initialization");
    return;
  }

  if (!got_control_cmd_) {

    ROS_INFO_THROTTLE(1.0, "waiting for data");
    return;
  }

  // keeps ros in the loop
  {
    std::scoped_lock lock(mutex_control_cmd_);

    // if the uav reached the previousy set destination
    if ((ros::Time::now() - last_successfull_command_).toSec() > 1.0 && fabs(control_cmd.twist.twist.linear.x) < 0.01 &&
        fabs(control_cmd.twist.twist.linear.y) < 0.01) {

      // create new point to fly to
      mrs_msgs::TrajectoryReference trajectory;
      trajectory.fly_now = true;
      trajectory.dt      = _trajectory_dt_;
      trajectory.use_yaw = false;

      double dist, direction;

      if (_randomize_distance_) {
        dist = randd(0, _max_distance_);
      } else {
        dist = _max_distance_;
      }

      direction = randd(-M_PI, M_PI);

      int n_points = (dist / _speed_) / _trajectory_dt_;

      double speed        = 0;
      double acceleration = 0;
      double pos_x        = control_cmd.pose.pose.position.x;
      double pos_y        = control_cmd.pose.pose.position.y;

      for (int it = 0; it < n_points; it++) {

        acceleration += (_jerk_ * _trajectory_dt_);

        if (acceleration >= _acceleration_) {
          acceleration = _acceleration_;
        }

        speed += (acceleration * _trajectory_dt_);

        if (speed >= _speed_) {
          speed = _speed_;
        }

        pos_x += cos(direction) * (speed * _trajectory_dt_);
        pos_y += sin(direction) * (speed * _trajectory_dt_);

        mrs_msgs::Reference new_point;
        new_point.position.x = pos_x;
        new_point.position.y = pos_y;
        new_point.position.z = _height_;
        new_point.yaw        = 0;

        trajectory.points.push_back(new_point);
      }

      if (setTrajectorySrv(trajectory)) {

        ROS_INFO("[TrajectoryRandomFlier]: trajectory set");

        last_successfull_command_ = ros::Time::now();
      }
    }
  }
}

//}

// | ------------------------ routines ------------------------ |

/* randd() //{ */

double TrajectoryRandomFlier::randd(double from, double to) {

  double zero_to_one = double((float)rand()) / double(RAND_MAX);

  return floor(to - from) * zero_to_one + from;
}

//}

/* setTrajectorySrv() //{ */

bool TrajectoryRandomFlier::setTrajectorySrv(const mrs_msgs::TrajectoryReference trajectory) {

  mrs_msgs::TrajectoryReferenceSrv srv;
  srv.request.trajectory = trajectory;

  bool success = service_client_trajectory_.call(srv);

  if (success) {

    if (!srv.response.success) {
      ROS_ERROR_STREAM("[ControlTest]: service call for setting trajectory failed: " << srv.response.message);
      return false;
    } else {
      return true;
    }

  } else {
    ROS_ERROR("[ControlTest]: service call for setting trajectory failed");
    return false;
  }
}

//}

}  // namespace mrs_testing

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_testing::TrajectoryRandomFlier, nodelet::Nodelet)
