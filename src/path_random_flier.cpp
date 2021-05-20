/* includes //{ */

#include "mrs_msgs/Path.h"
#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <stdio.h>
#include <stdlib.h>

#include <mrs_msgs/PathSrv.h>
#include <mrs_msgs/Reference.h>
#include <mrs_msgs/ControlManagerDiagnostics.h>
#include <mrs_msgs/MpcPredictionFullState.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/msg_extractor.h>

#include <std_srvs/Trigger.h>

#include <random>

//}

namespace mrs_uav_testing
{

/* class PathRandomFlier //{ */

class PathRandomFlier : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  bool is_initialized_ = false;

  bool   callbackActivate([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  void   timerMain(const ros::TimerEvent& event);
  double randd(const double from, const double to);
  int    randi(const int from, const int to);
  bool   setPathSrv(const mrs_msgs::Path path_in);

  mrs_lib::SubscribeHandler<mrs_msgs::PositionCommand>           sh_position_cmd_;
  mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics> sh_control_manager_diag_;
  mrs_lib::SubscribeHandler<mrs_msgs::MpcPredictionFullState>    sh_mpc_predition_;

  ros::Publisher publisher_goto_;

  ros::ServiceServer service_server_activate_;

  ros::ServiceClient service_client_path_;

  ros::Timer timer_main_;

  double _main_timer_rate_;

  int _n_points_min_;
  int _n_points_max_;

  double _point_distance_min_;
  double _point_distance_max_;

  double _z_value_;
  double _z_deviation_;

  double _future_stamp_prob_;
  double _future_stamp_min_;
  double _future_stamp_max_;

  double _replanning_time_min_;
  double _replanning_time_max_;

  double _heading_change_;
  double _bearing_change_;

  bool active_ = true;

  bool      next_wait_for_finish_ = false;
  ros::Time next_replan_time_;

  ros::Time last_successfull_command_;
};

//}

/* onInit() //{ */

void PathRandomFlier::onInit(void) {

  ros::NodeHandle nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  ros::Time::waitForValid();

  mrs_lib::ParamLoader param_loader(nh_, "PathRandomFlier");

  // load parameters from config file
  param_loader.loadParam("main_timer_rate", _main_timer_rate_);
  param_loader.loadParam("active", active_);

  param_loader.loadParam("heading_change", _heading_change_);
  param_loader.loadParam("bearing_change", _bearing_change_);
  param_loader.loadParam("n_points/min", _n_points_min_);
  param_loader.loadParam("n_points/max", _n_points_max_);
  param_loader.loadParam("point_distance/min", _point_distance_min_);
  param_loader.loadParam("point_distance/max", _point_distance_max_);
  param_loader.loadParam("z/value", _z_value_);
  param_loader.loadParam("z/deviation", _z_deviation_);

  param_loader.loadParam("future_stamp/prob", _future_stamp_prob_);
  param_loader.loadParam("future_stamp/min", _future_stamp_min_);
  param_loader.loadParam("future_stamp/max", _future_stamp_max_);

  param_loader.loadParam("replanning_time/min", _replanning_time_min_);
  param_loader.loadParam("replanning_time/max", _replanning_time_max_);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[PathRandomFlier]: Could not load all parameters!");
    ros::requestShutdown();
  }

  mrs_lib::SubscribeHandlerOptions shopts;
  shopts.nh                 = nh_;
  shopts.node_name          = "PathRandomFlier";
  shopts.no_message_timeout = mrs_lib::no_timeout;
  shopts.threadsafe         = true;

  sh_position_cmd_         = mrs_lib::SubscribeHandler<mrs_msgs::PositionCommand>(shopts, "position_command_in");
  sh_control_manager_diag_ = mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics>(shopts, "control_manager_diag_in");
  sh_mpc_predition_        = mrs_lib::SubscribeHandler<mrs_msgs::MpcPredictionFullState>(shopts, "mpc_prediction_in");

  service_server_activate_ = nh_.advertiseService("activate_in", &PathRandomFlier::callbackActivate, this);
  service_client_path_     = nh_.serviceClient<mrs_msgs::PathSrv>("path_out");

  // initialize the random number generator
  srand(static_cast<unsigned int>(ros::Time::now().nsec));
  /* srand(time(NULL)); */

  last_successfull_command_ = ros::Time(0);

  timer_main_ = nh_.createTimer(ros::Rate(_main_timer_rate_), &PathRandomFlier::timerMain, this);

  // | ----------------------- finish init ---------------------- |

  is_initialized_ = true;

  ROS_INFO("[PathRandomFlier]: initialized");
}

//}

// | ------------------------ callbacks ----------------------- |

/* callbackActivate() //{ */

bool PathRandomFlier::callbackActivate([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {

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

void PathRandomFlier::timerMain([[maybe_unused]] const ros::TimerEvent& event) {

  if (!is_initialized_) {
    return;
  }

  if (!active_) {

    ROS_INFO_ONCE("[PathRandomFlier]: waiting for initialization");
    return;
  }

  if (!sh_position_cmd_.hasMsg()) {

    ROS_INFO_THROTTLE(1.0, "waiting for PositionCommand");
    return;
  }

  if (!sh_mpc_predition_.hasMsg()) {

    ROS_INFO_THROTTLE(1.0, "waiting for MPC prediction");
    return;
  }

  if (!sh_control_manager_diag_.hasMsg()) {

    ROS_INFO_THROTTLE(1.0, "waiting for ControlManager diagnostics");
    return;
  }

  auto [cmd_speed_x, cmd_speed_y, cmd_speed_z] = mrs_lib::getVelocity(sh_position_cmd_.getMsg());
  auto [cmd_x, cmd_y, cmd_z]                   = mrs_lib::getPosition(sh_position_cmd_.getMsg());
  bool has_goal                                = sh_control_manager_diag_.getMsg()->tracker_status.have_goal;

  // if the uav reached the previousy set destination
  if ((ros::Time::now() - last_successfull_command_).toSec() > 1.0 &&
      (!has_goal || (!next_wait_for_finish_ && (next_replan_time_ - ros::Time::now()).toSec() < 0))) {

    // create new point to fly to
    mrs_msgs::Path path;
    path.fly_now     = true;
    path.use_heading = true;

    double pos_x, pos_y, pos_z;

    if (!next_wait_for_finish_) {

      double time_offset = randd(_future_stamp_min_, _future_stamp_max_);

      int prediction_idx = int(round((time_offset - 0.01) / 0.2));

      pos_x = sh_mpc_predition_.getMsg()->position[prediction_idx].x;
      pos_y = sh_mpc_predition_.getMsg()->position[prediction_idx].y;
      pos_z = sh_mpc_predition_.getMsg()->position[prediction_idx].z;

      if (has_goal) {
        path.header.stamp = ros::Time::now() + ros::Duration(time_offset);
      } else {
        path.header.stamp = ros::Time(0);
      }

    } else {

      pos_x = cmd_x;
      pos_y = cmd_y;
      pos_z = _z_value_ + randd(-_z_deviation_, _z_deviation_);

      path.header.stamp = ros::Time(0);
    }

    double dist;

    double bearing = randd(-M_PI, M_PI);
    double heading = randd(-M_PI, M_PI);

    ROS_INFO("[PathRandomFlier]: pos_x: %.2f, pos_y: %.2f", pos_x, pos_y);

    int n_points = randi(_n_points_min_, _n_points_max_);

    for (int it = 0; it < n_points; it++) {

      double heading_change = randd(-_heading_change_, _heading_change_);

      mrs_msgs::Reference new_point;
      new_point.position.x = pos_x;
      new_point.position.y = pos_y;
      new_point.position.z = pos_z;
      new_point.heading    = bearing;

      path.points.push_back(new_point);

      bearing += randd(-_bearing_change_, _bearing_change_);

      heading += randd(-_heading_change_, _heading_change_);

      double distance = randd(_point_distance_min_, _point_distance_max_);

      pos_x += cos(bearing) * distance;
      pos_y += sin(bearing) * distance;
    }

    next_wait_for_finish_ = randd(0, 10) <= 10 * _future_stamp_prob_ ? false : true;

    if (!next_wait_for_finish_) {
      double replan_time = randd(_replanning_time_min_, _replanning_time_max_);
      next_replan_time_  = ros::Time::now() + ros::Duration(replan_time);
      ROS_INFO("[PathRandomFlier]: replanning in %.2f s", replan_time);
    }

    if (setPathSrv(path)) {

      ROS_INFO("[PathRandomFlier]: path set");

      last_successfull_command_ = ros::Time::now();
    }
  }
}  // namespace mrs_uav_testing

//}

// | ------------------------ routines ------------------------ |

/* randd() //{ */

double PathRandomFlier::randd(const double from, const double to) {

  double zero_to_one = double((float)rand()) / double(RAND_MAX);

  return (to - from) * zero_to_one + from;
}

//}

/* randi() //{ */

int PathRandomFlier::randi(const int from, const int to) {

  double zero_to_one = double((float)rand()) / double(RAND_MAX);

  return int(double(to - from) * zero_to_one + from);
}

//}

/* setPathServ() //{ */

bool PathRandomFlier::setPathSrv(const mrs_msgs::Path path_in) {

  mrs_msgs::PathSrv srv;
  srv.request.path = path_in;

  bool success = service_client_path_.call(srv);

  if (success) {

    if (!srv.response.success) {
      ROS_ERROR_STREAM_THROTTLE(1.0, "[PathRandomFlier]: service call for setting path failed: " << srv.response.message);
      return false;
    } else {
      return true;
    }

  } else {
    ROS_ERROR_THROTTLE(1.0, "[PathRandomFlier]: service call for setting path failed");
    return false;
  }
}

//}

}  // namespace mrs_uav_testing

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_uav_testing::PathRandomFlier, nodelet::Nodelet)
