/* includes //{ */

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <stdio.h>
#include <stdlib.h>

#include <mrs_msgs/PositionCommand.h>
#include <mrs_msgs/ReferenceStampedSrv.h>
#include <std_srvs/Trigger.h>

#include <mrs_lib/ParamLoader.h>
#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/msg_extractor.h>

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

  bool   callbackActivate([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  void   timerMain(const ros::TimerEvent& event);
  double randd(double from, double to);

  mrs_lib::SubscribeHandler<mrs_msgs::PositionCommand> sh_position_cmd_;

  ros::ServiceServer service_server_activate_;

  ros::ServiceClient service_client_reference_;

  ros::Timer main_timer_;

  double _main_timer_rate_;

  // parameters loaded from config file
  double _height_;
  bool   _randomize_distance_ = false;
  double _max_distance_;

  bool active_ = true;

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

  const std::string node_name  = "RandomFlier";
  const bool        threadsafe = true;

  mrs_lib::SubscribeHandlerOptions shopts{
      .nh = nh_, .node_name = node_name, .topic_name = "pes", .no_message_timeout = mrs_lib::no_timeout, .threadsafe = true};


  shopts.topic_name = "position_command_in";
  sh_position_cmd_  = mrs_lib::SubscribeHandler<mrs_msgs::PositionCommand>(shopts);

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

// | ------------------------ callbacks ----------------------- |

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

// | ------------------------- timers ------------------------- |

/* timerMain() //{ */

void RandomFlier::timerMain([[maybe_unused]] const ros::TimerEvent& event) {

  if (!is_initialized_) {
    return;
  }

  if (!active_) {

    ROS_INFO_THROTTLE(1.0, "[RandomFlier]: waiting for activation");
    return;
  }

  if (!sh_position_cmd_.has_data()) {

    ROS_INFO_THROTTLE(1.0, "[RandomFlier]: waiting for PositionCommand");
    return;
  }

  auto [cmd_speed_x, cmd_speed_y, cmd_speed_z] = mrs_lib::getVelocity(sh_position_cmd_.get_data());
  auto [cmd_x, cmd_y, cmd_z]                   = mrs_lib::getPosition(sh_position_cmd_.get_data());

  // if the uav reach the previousy set destination
  if ((ros::Time::now() - last_successfull_command_).toSec() > 1.0 && fabs(cmd_speed_x) < 0.01 && fabs(cmd_speed_y) < 0.01) {

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

      new_point.request.reference.position.x = cmd_x + cos(direction) * dist;
      new_point.request.reference.position.y = cmd_y + sin(direction) * dist;
      new_point.request.reference.position.z = _height_;
      new_point.request.reference.heading    = 0;

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

//}

// | ------------------------ routines ------------------------ |

/* randd() //{ */

double RandomFlier::randd(double from, double to) {

  double zero_to_one = double((float)rand()) / double(RAND_MAX);

  return floor(to - from) * zero_to_one + from;
}

//}

}  // namespace mrs_testing

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_testing::RandomFlier, nodelet::Nodelet)
