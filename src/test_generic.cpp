#include <mrs_uav_testing/test_generic.h>

namespace mrs_uav_testing
{

/* constructors //{ */

TestGeneric::TestGeneric() {

  initialize();
}

//}

/* initialize() //{ */

void TestGeneric::initialize(void) {

  nh_ = ros::NodeHandle("~");

  ROS_INFO("[%s]: ROS node initialized", name_.c_str());

  ros::Time::waitForValid();

  spinner_ = make_shared<ros::AsyncSpinner>(4);
  spinner_->start();

  // | ----------------------- load params ---------------------- |

  pl_ = std::make_shared<mrs_lib::ParamLoader>(nh_, "Test");

  pl_->loadParam("uav_name", _uav_name_, std::string());
  pl_->loadParam("test_name", _test_name_, std::string());
  pl_->loadParam("gazebo_spawner_params", _gazebo_spawner_params_, std::string());

  name_ = _uav_name_ + "/" + _test_name_;

  // | ----------------------- transformer ---------------------- |

  transformer_ = std::make_shared<mrs_lib::Transformer>(nh_, "Test");
  transformer_->setDefaultPrefix(_uav_name_);
  transformer_->retryLookupNewest(true);

  // | ----------------------- subscribers ---------------------- |

  shopts_.nh                 = nh_;
  shopts_.node_name          = name_;
  shopts_.no_message_timeout = mrs_lib::no_timeout;
  shopts_.threadsafe         = true;
  shopts_.autostart          = true;
  shopts_.queue_size         = 10;
  shopts_.transport_hints    = ros::TransportHints().tcpNoDelay();

  sh_control_manager_diag_ = mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics>(shopts_, "/" + _uav_name_ + "/control_manager/diagnostics");
  sh_uav_manager_diag_     = mrs_lib::SubscribeHandler<mrs_msgs::UavManagerDiagnostics>(shopts_, "/" + _uav_name_ + "/uav_manager/diagnostics");
  sh_estim_manager_diag_   = mrs_lib::SubscribeHandler<mrs_msgs::EstimationDiagnostics>(shopts_, "/" + _uav_name_ + "/estimation_manager/diagnostics");
  sh_constraint_manager_diag_ =
      mrs_lib::SubscribeHandler<mrs_msgs::ConstraintManagerDiagnostics>(shopts_, "/" + _uav_name_ + "/constraint_manager/diagnostics");
  sh_gain_manager_diag_   = mrs_lib::SubscribeHandler<mrs_msgs::GainManagerDiagnostics>(shopts_, "/" + _uav_name_ + "/gain_manager/diagnostics");
  sh_uav_state_           = mrs_lib::SubscribeHandler<mrs_msgs::UavState>(shopts_, "/" + _uav_name_ + "/estimation_manager/uav_state");
  sh_gazebo_spawner_diag_ = mrs_lib::SubscribeHandler<mrs_msgs::GazeboSpawnerDiagnostics>(shopts_, "/mrs_drone_spawner/diagnostics");

  sh_hw_api_status_ = mrs_lib::SubscribeHandler<mrs_msgs::HwApiStatus>(shopts_, "/" + _uav_name_ + "/hw_api/status");

  // | --------------------- service clients -------------------- |

  sch_arming_            = mrs_lib::ServiceClientHandler<std_srvs::SetBool>(nh_, "/" + _uav_name_ + "/hw_api/arming");
  sch_offboard_          = mrs_lib::ServiceClientHandler<std_srvs::Trigger>(nh_, "/" + _uav_name_ + "/hw_api/offboard");
  sch_midair_activation_ = mrs_lib::ServiceClientHandler<std_srvs::Trigger>(nh_, "/" + _uav_name_ + "/uav_manager/midair_activation");
  sch_spawn_gazebo_uav_  = mrs_lib::ServiceClientHandler<mrs_msgs::String>(nh_, "/mrs_drone_spawner/spawn");

  sch_goto_          = mrs_lib::ServiceClientHandler<mrs_msgs::Vec4>(nh_, "/" + _uav_name_ + "/control_manager/goto");
  sch_goto_relative_ = mrs_lib::ServiceClientHandler<mrs_msgs::Vec4>(nh_, "/" + _uav_name_ + "/control_manager/goto_relative");

  sch_path_ = mrs_lib::ServiceClientHandler<mrs_msgs::PathSrv>(nh_, "/" + _uav_name_ + "/trajectory_generation/path");

  // | ----------------------- publishers ----------------------- |

  ph_path_ = mrs_lib::PublisherHandler<mrs_msgs::Path>(nh_, "/" + _uav_name_ + "/trajectory_generation/path");

  // | --------------------- finish the init -------------------- |

  ROS_INFO("[%s]: initialized", name_.c_str());
}

//}

// | --------------------- action methods --------------------- |

/* spawnGazeboUav() //{ */

tuple<bool, string> TestGeneric::spawnGazeboUav() {

  // | ------------ wait for the spawner to be ready ------------ |

  while (true) {

    if (!ros::ok()) {
      return {false, "shut down from outside"};
    }

    ROS_INFO_THROTTLE(1.0, "[%s]: waiting for the Gazebo drone spawner", name_.c_str());

    if (sh_gazebo_spawner_diag_.hasMsg()) {
      break;
    }

    sleep(0.01);
  }

  // | -------------------------- wait  ------------------------- |

  sleep(1.0);

  // | ------------------- call spawn service ------------------- |

  {

    mrs_msgs::String srv;

    srv.request.value = _gazebo_spawner_params_;

    bool service_call = sch_spawn_gazebo_uav_.call(srv);

    if (!service_call || !srv.response.success) {
      return {false, "gazebo drone spawner service call failed"};
    }
  }

  // | ------------------ wait while processing ----------------- |

  while (true) {

    if (!ros::ok()) {
      return {false, "shut down from outside"};
    }

    ROS_INFO_THROTTLE(1.0, "[%s]: waiting for the drone to spawn", name_.c_str());

    if (!sh_gazebo_spawner_diag_.getMsg()->processing) {
      break;
    }

    sleep(0.01);
  }

  return {true, "drone spawned"};
}

//}

/* sleep() //{ */

void TestGeneric::sleep(const double &duration) {

  ros::Duration(duration).sleep();
}

//}

/* takeoff() //{ */

tuple<bool, string> TestGeneric::takeoff(void) {

  // | ---------------- wait for ready to takeoff --------------- |

  while (true) {

    if (!ros::ok()) {
      return {false, "shut down from outside"};
    }

    ROS_INFO_THROTTLE(1.0, "[%s]: waiting for the MRS UAV System", name_.c_str());

    if (mrsSystemReady()) {
      ROS_INFO("[%s]: MRS UAV System is ready", name_.c_str());
      break;
    }

    sleep(0.01);
  }

  // | ---------------------- arm the drone --------------------- |

  ROS_INFO("[%s]: arming the drone", name_.c_str());

  {
    std_srvs::SetBool srv;
    srv.request.data = true;

    {
      bool service_call = sch_arming_.call(srv);

      if (!service_call || !srv.response.success) {
        return {false, "arming service call failed"};
      }
    }
  }

  // | ---------------------- wait a second --------------------- |

  sleep(2.0);

  // | --------------------- check if armed --------------------- |

  if (!sh_hw_api_status_.getMsg()->armed) {
    return {false, "not armed"};
  }

  // | ------------------- switch to offboard ------------------- |

  {
    std_srvs::Trigger srv;

    {
      bool service_call = sch_offboard_.call(srv);

      if (!service_call || !srv.response.success) {
        return {false, "offboard service call failed"};
      }
    }
  }

  // | -------------------------- wait -------------------------- |

  sleep(0.1);

  // | ------------------ check if in offboard ------------------ |

  if (!sh_hw_api_status_.getMsg()->offboard) {
    return {false, "not in offboard"};
  }

  // | --------------- wait for takeoff to finish --------------- |

  while (true) {

    if (!ros::ok()) {
      return {false, "shut down from outside"};
    }

    ROS_INFO_THROTTLE(1.0, "[%s]: waiting for the takeoff to finish", name_.c_str());

    if (sh_control_manager_diag_.getMsg()->flying_normally) {

      return {true, "takeoff finished"};
    }

    sleep(0.01);
  }

  return {false, "reached end of the method without assertion"};
}

//}

/* activateMidAir() //{ */

tuple<bool, string> TestGeneric::activateMidAir(void) {

  // | ---------------- wait for ready to takeoff --------------- |

  while (true) {

    if (!ros::ok()) {
      return {false, "shut down from outside"};
    }

    ROS_INFO_THROTTLE(1.0, "[%s]: waiting for the MRS UAV System", name_.c_str());

    if (mrsSystemReady()) {
      ROS_INFO("[%s]: MRS UAV System is ready", name_.c_str());
      break;
    }

    sleep(0.01);
  }

  // | ---------------------- arm the drone --------------------- |

  ROS_INFO("[%s]: arming the drone", name_.c_str());

  {
    std_srvs::SetBool srv;
    srv.request.data = true;

    {
      bool service_call = sch_arming_.call(srv);

      if (!service_call || !srv.response.success) {
        return {false, "arming service call failed"};
      }
    }
  }

  // | -------------------------- wait -------------------------- |

  sleep(0.1);

  // | --------------------- check if armed --------------------- |

  if (!sh_hw_api_status_.getMsg()->armed) {
    return {false, "not armed"};
  }

  // | ----------------- call midair activation ----------------- |

  {
    std_srvs::Trigger srv;

    {
      bool service_call = sch_midair_activation_.call(srv);

      if (!service_call || !srv.response.success) {
        return {false, "midair activation service call failed"};
      }
    }
  }

  // | --------------- wait for takeoff to finish --------------- |

  while (true) {

    if (!ros::ok()) {
      return {false, "shut down from outside"};
    }

    ROS_INFO_THROTTLE(1.0, "[%s]: waiting for the midair activation to finish", name_.c_str());

    if (sh_control_manager_diag_.getMsg()->flying_normally) {

      return {true, "midair activation finished"};
    }

    sleep(0.01);
  }

  return {false, "reached end of the method without assertion"};
}

//}

/* gotoAbs() //{ */

tuple<bool, string> TestGeneric::gotoAbs(const double &x, const double &y, const double &z, const double &hdg) {

  mrs_msgs::Vec4 srv;

  srv.request.goal[0] = x;
  srv.request.goal[1] = y;
  srv.request.goal[2] = z;
  srv.request.goal[3] = hdg;

  {
    bool service_call = sch_goto_.call(srv);

    if (!service_call || !srv.response.success) {
      return {false, "goto service call failed"};
    }
  }

  // | -------------------- check for result -------------------- |

  while (true) {

    if (!ros::ok()) {
      return {false, "shut down from outside"};
    }

    if (!isFlyingNormally()) {
      return {false, "not flying normally"};
    }

    if (isAtPosition(x, y, z, hdg, 0.1)) {

      return {true, "goal reached"};
    }

    sleep(0.01);
  }

  return {false, "reached end of the method without assertion"};
}

//}

/* gotoRel() //{ */

tuple<bool, string> TestGeneric::gotoRel(const double &x, const double &y, const double &z, const double &hdg) {

  auto start_pose = sh_estim_manager_diag_.getMsg()->pose.position;
  auto start_hdg  = mrs_lib::AttitudeConverter(sh_estim_manager_diag_.getMsg()->pose.orientation).getHeading();

  {
    mrs_msgs::Vec4 srv;

    srv.request.goal[0] = x;
    srv.request.goal[1] = y;
    srv.request.goal[2] = z;
    srv.request.goal[3] = hdg;

    {
      bool service_call = sch_goto_relative_.call(srv);

      if (!service_call || !srv.response.success) {
        return {false, "goto relative service call failed"};
      }
    }
  }

  // | -------------------- check for result -------------------- |

  while (true) {

    if (!ros::ok()) {
      return {false, "shut down from outside"};
    }

    if (!isFlyingNormally()) {
      return {false, "not flying normally"};
    }

    if (isAtPosition(start_pose.x + x, start_pose.y + y, start_pose.z + z, start_hdg + hdg, 0.1)) {
      return {true, "goal reached"};
    }

    sleep(0.01);
  }

  return {false, "reached end of the method without assertion"};
}

//}

/* setPathSrv() //{ */

tuple<bool, string> TestGeneric::setPathSrv(const mrs_msgs::Path &path_in) {

  mrs_msgs::PathSrv srv;
  srv.request.path = path_in;

  {
    bool service_call = sch_path_.call(srv);

    if (!service_call || !srv.response.success) {
      return {false, "path service call failed"};
    }
  }

  return {true, "path set"};
}

//}

/* setPathTopic() //{ */

tuple<bool, string> TestGeneric::setPathTopic(const mrs_msgs::Path &path_in) {

  ph_path_.publish(path_in);

  return {true, "path set"};
}

//}

// | ------------------------- getters ------------------------ |

/* isAtPosition() //{ */

bool TestGeneric::isAtPosition(const double &x, const double &y, const double &z, const double &hdg, const double &pos_tolerance) {

  auto uav_state = sh_uav_state_.getMsg();

  auto current_hdg = mrs_lib::AttitudeConverter(uav_state->pose.orientation).getHeading();

  if (abs(x - uav_state->pose.position.x) < pos_tolerance && abs(y - uav_state->pose.position.y) < pos_tolerance &&
      abs(z - uav_state->pose.position.z) < pos_tolerance && abs(sradians::diff(hdg, current_hdg)) < 0.2) {

    return true;

  } else {

    return false;
  }
}

//}

/* getActiveController() //{ */

std::string TestGeneric::getActiveTracker(void) {

  if (!sh_control_manager_diag_.getMsg()) {
    return "";
  }

  return sh_control_manager_diag_.getMsg()->active_tracker;
}

//}

/* getActiveController() //{ */

std::string TestGeneric::getActiveController(void) {

  if (!sh_control_manager_diag_.getMsg()) {
    return "";
  }

  return sh_control_manager_diag_.getMsg()->active_controller;
}

//}

/* hasGoal() //{ */

bool TestGeneric::hasGoal(void) {

  if (sh_control_manager_diag_.hasMsg()) {
    return sh_control_manager_diag_.getMsg()->tracker_status.have_goal;
  } else {
    return false;
  }
}

//}

/* mrsSystemReady() //{ */

bool TestGeneric::mrsSystemReady(void) {

  bool got_control_manager_diag    = sh_control_manager_diag_.hasMsg();
  bool got_estimation_manager_diag = sh_estim_manager_diag_.hasMsg();
  bool got_uav_manager_diag        = sh_uav_manager_diag_.hasMsg();
  bool got_gain_manager_diag       = sh_gain_manager_diag_.hasMsg();
  bool got_constraint_manager_diag = sh_constraint_manager_diag_.hasMsg();

  return got_control_manager_diag && got_estimation_manager_diag && got_uav_manager_diag && got_gain_manager_diag && got_constraint_manager_diag;
}

//}

/* isFlyingNormally() //{ */

bool TestGeneric::isFlyingNormally(void) {

  if (sh_control_manager_diag_.hasMsg()) {
    return sh_control_manager_diag_.getMsg()->flying_normally;
  } else {
    return false;
  }
}

//}

/* isOutputEnabled() //{ */

bool TestGeneric::isOutputEnabled(void) {

  if (sh_control_manager_diag_.hasMsg()) {
    return sh_control_manager_diag_.getMsg()->output_enabled;
  } else {
    return false;
  }
}

//}

}  // namespace mrs_uav_testing
