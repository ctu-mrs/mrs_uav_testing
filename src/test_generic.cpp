#include <mrs_uav_testing/test_generic.h>

namespace mrs_uav_testing
{

UAVHandler::UAVHandler(std::string uav_name, std::shared_ptr<mrs_lib::SubscribeHandlerOptions> shopts, std::shared_ptr<mrs_lib::Transformer> transformer,
                       bool use_hw_api) {

  initialize(uav_name, shopts, transformer, use_hw_api);
}

void UAVHandler::initialize(std::string uav_name, std::shared_ptr<mrs_lib::SubscribeHandlerOptions> shopts, std::shared_ptr<mrs_lib::Transformer> transformer,
                            bool use_hw_api) {

  _uav_name_   = uav_name;
  shopts_      = shopts;
  nh_          = shopts_->nh;
  name_        = shopts->node_name;
  transformer_ = transformer;

  use_hw_api_ = use_hw_api;

  sh_control_manager_diag_ = mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics>(*shopts_, "/" + _uav_name_ + "/control_manager/diagnostics");
  sh_current_constraints_  = mrs_lib::SubscribeHandler<mrs_msgs::DynamicsConstraints>(*shopts_, "/" + _uav_name_ + "/control_manager/current_constraints");
  sh_uav_manager_diag_     = mrs_lib::SubscribeHandler<mrs_msgs::UavManagerDiagnostics>(*shopts_, "/" + _uav_name_ + "/uav_manager/diagnostics");
  sh_tracker_cmd_          = mrs_lib::SubscribeHandler<mrs_msgs::TrackerCommand>(*shopts_, "/" + _uav_name_ + "/control_manager/tracker_cmd");
  sh_estim_manager_diag_   = mrs_lib::SubscribeHandler<mrs_msgs::EstimationDiagnostics>(*shopts_, "/" + _uav_name_ + "/estimation_manager/diagnostics");
  sh_constraint_manager_diag_ =
      mrs_lib::SubscribeHandler<mrs_msgs::ConstraintManagerDiagnostics>(*shopts_, "/" + _uav_name_ + "/constraint_manager/diagnostics");
  sh_gain_manager_diag_ = mrs_lib::SubscribeHandler<mrs_msgs::GainManagerDiagnostics>(*shopts_, "/" + _uav_name_ + "/gain_manager/diagnostics");
  sh_uav_state_         = mrs_lib::SubscribeHandler<mrs_msgs::UavState>(*shopts_, "/" + _uav_name_ + "/estimation_manager/uav_state");
  sh_height_agl_        = mrs_lib::SubscribeHandler<mrs_msgs::Float64Stamped>(*shopts_, "/" + _uav_name_ + "/estimation_manager/height_agl");
  sh_max_height_        = mrs_lib::SubscribeHandler<mrs_msgs::Float64Stamped>(*shopts_, "/" + _uav_name_ + "/estimation_manager/max_flight_z_agl");
  sh_speed_             = mrs_lib::SubscribeHandler<mrs_msgs::Float64Stamped>(*shopts_, "/" + _uav_name_ + "/control_manager/speed");

  sh_hw_api_status_ = mrs_lib::SubscribeHandler<mrs_msgs::HwApiStatus>(*shopts_, "/" + _uav_name_ + "/hw_api/status");

  // | --------------------- service clients -------------------- |

  sch_arming_            = mrs_lib::ServiceClientHandler<std_srvs::SetBool>(nh_, "/" + _uav_name_ + "/hw_api/arming");
  sch_offboard_          = mrs_lib::ServiceClientHandler<std_srvs::Trigger>(nh_, "/" + _uav_name_ + "/hw_api/offboard");
  sch_midair_activation_ = mrs_lib::ServiceClientHandler<std_srvs::Trigger>(nh_, "/" + _uav_name_ + "/uav_manager/midair_activation");
  sch_land_              = mrs_lib::ServiceClientHandler<std_srvs::Trigger>(nh_, "/" + _uav_name_ + "/uav_manager/land");
  sch_land_home_         = mrs_lib::ServiceClientHandler<std_srvs::Trigger>(nh_, "/" + _uav_name_ + "/uav_manager/land_home");
  sch_switch_estimator_  = mrs_lib::ServiceClientHandler<mrs_msgs::String>(nh_, "/" + _uav_name_ + "/estimation_manager/change_estimator");
  sch_switch_controller_ = mrs_lib::ServiceClientHandler<mrs_msgs::String>(nh_, "/" + _uav_name_ + "/control_manager/switch_controller");
  sch_switch_tracker_    = mrs_lib::ServiceClientHandler<mrs_msgs::String>(nh_, "/" + _uav_name_ + "/control_manager/switch_tracker");
  sch_set_gains_         = mrs_lib::ServiceClientHandler<mrs_msgs::String>(nh_, "/" + _uav_name_ + "/gain_manager/set_gains");
  sch_set_constraints_   = mrs_lib::ServiceClientHandler<mrs_msgs::String>(nh_, "/" + _uav_name_ + "/constraint_manager/set_constraints");

  sch_goto_                 = mrs_lib::ServiceClientHandler<mrs_msgs::Vec4>(nh_, "/" + _uav_name_ + "/control_manager/goto");
  sch_goto_fcu_             = mrs_lib::ServiceClientHandler<mrs_msgs::Vec4>(nh_, "/" + _uav_name_ + "/control_manager/goto_fcu");
  sch_goto_relative_        = mrs_lib::ServiceClientHandler<mrs_msgs::Vec4>(nh_, "/" + _uav_name_ + "/control_manager/goto_relative");
  sch_set_heading_          = mrs_lib::ServiceClientHandler<mrs_msgs::Vec1>(nh_, "/" + _uav_name_ + "/control_manager/set_heading");
  sch_set_heading_relative_ = mrs_lib::ServiceClientHandler<mrs_msgs::Vec1>(nh_, "/" + _uav_name_ + "/control_manager/set_heading_relative");
  sch_goto_altitude_        = mrs_lib::ServiceClientHandler<mrs_msgs::Vec1>(nh_, "/" + _uav_name_ + "/control_manager/goto_altitude");

  sch_reference_ = mrs_lib::ServiceClientHandler<mrs_msgs::ReferenceStampedSrv>(nh_, "/" + _uav_name_ + "/control_manager/reference");

  sch_goto_trajectory_start_      = mrs_lib::ServiceClientHandler<std_srvs::Trigger>(nh_, "/" + _uav_name_ + "/control_manager/goto_trajectory_start");
  sch_start_trajectory_tracking_  = mrs_lib::ServiceClientHandler<std_srvs::Trigger>(nh_, "/" + _uav_name_ + "/control_manager/start_trajectory_tracking");
  sch_stop_trajectory_tracking_   = mrs_lib::ServiceClientHandler<std_srvs::Trigger>(nh_, "/" + _uav_name_ + "/control_manager/stop_trajectory_tracking");
  sch_resume_trajectory_tracking_ = mrs_lib::ServiceClientHandler<std_srvs::Trigger>(nh_, "/" + _uav_name_ + "/control_manager/resume_trajectory_tracking");

  sch_hover_ = mrs_lib::ServiceClientHandler<std_srvs::Trigger>(nh_, "/" + _uav_name_ + "/control_manager/hover");

  sch_path_     = mrs_lib::ServiceClientHandler<mrs_msgs::PathSrv>(nh_, "/" + _uav_name_ + "/trajectory_generation/path");
  sch_get_path_ = mrs_lib::ServiceClientHandler<mrs_msgs::GetPathSrv>(nh_, "/" + _uav_name_ + "/trajectory_generation/get_path");

  sch_validate_reference_ = mrs_lib::ServiceClientHandler<mrs_msgs::ValidateReference>(nh_, "/" + _uav_name_ + "/control_manager/validate_reference");
  sch_validate_reference_array_ =
      mrs_lib::ServiceClientHandler<mrs_msgs::ValidateReferenceArray>(nh_, "/" + _uav_name_ + "/control_manager/validate_reference_array");

  sch_tranform_reference_ = mrs_lib::ServiceClientHandler<mrs_msgs::TransformReferenceSrv>(nh_, "/" + _uav_name_ + "/control_manager/transform_reference");
  sch_tranform_vector3_   = mrs_lib::ServiceClientHandler<mrs_msgs::TransformVector3Srv>(nh_, "/" + _uav_name_ + "/control_manager/transform_vector3");
  sch_tranform_pose_      = mrs_lib::ServiceClientHandler<mrs_msgs::TransformPoseSrv>(nh_, "/" + _uav_name_ + "/control_manager/transform_pose");

  // | ----------------------- publishers ----------------------- |

  ph_path_               = mrs_lib::PublisherHandler<mrs_msgs::Path>(nh_, "/" + _uav_name_ + "/trajectory_generation/path");
  ph_trajectory_         = mrs_lib::PublisherHandler<mrs_msgs::TrajectoryReference>(nh_, "/" + _uav_name_ + "/control_manager/trajectory_reference");
  ph_velocity_reference_ = mrs_lib::PublisherHandler<mrs_msgs::VelocityReferenceStamped>(nh_, "/" + _uav_name_ + "/control_manager/velocity_reference");
  ph_reference_          = mrs_lib::PublisherHandler<mrs_msgs::ReferenceStamped>(nh_, "/" + _uav_name_ + "/control_manager/reference");

  initialized_ = true;
}

//}

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
  pl_->loadParam("test", _test_name_, std::string());

  name_ = "test/" + _uav_name_ + "/" + _test_name_;

  // | ----------------------- transformer ---------------------- |

  transformer_ = std::make_shared<mrs_lib::Transformer>(nh_, "Test");
  transformer_->retryLookupNewest(true);

  // | ----------------------- subscribers ---------------------- |

  shopts_ = std::make_shared<mrs_lib::SubscribeHandlerOptions>();

  shopts_->nh                 = nh_;
  shopts_->node_name          = name_;
  shopts_->no_message_timeout = mrs_lib::no_timeout;
  shopts_->threadsafe         = true;
  shopts_->autostart          = true;
  shopts_->queue_size         = 10;
  shopts_->transport_hints    = ros::TransportHints().tcpNoDelay();

  // | --------------------- finish the init -------------------- |

  initialized_ = true;

  ROS_INFO("[%s]: initialized", name_.c_str());
}

//}

// | --------------------- action methods --------------------- |

/* checkPreconditions() //{ */

tuple<bool, string> UAVHandler::checkPreconditions(void) {

  if (!initialized_) {
    ROS_ERROR_STREAM("[" << ros::this_node::getName().c_str() << "]: UAV handler for " << _uav_name_ << " is not initialized!");
    return {false, "UAV handler for " + _uav_name_ + " is not initialized!"};
  }

  return {true, "All clear."};
}

//}

/* getUAVHandler() //{ */

std::tuple<std::optional<std::shared_ptr<UAVHandler>>, string> TestGeneric::getUAVHandler(const string &uav_name, const bool use_hw_api) {

  if (!initialized_) {
    return {std::nullopt, string("Can not obtain UAV handler for  " + uav_name + " - testing is not initialized yet!")};
  } else {
    return {std::make_shared<UAVHandler>(uav_name, shopts_, transformer_, use_hw_api), "Success!"};
  }
}

//}

/* takeoff() //{ */

tuple<bool, string> UAVHandler::takeoff(void) {

  auto res = checkPreconditions();

  if (!(std::get<0>(res))) {
    return res;
  }

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

  sleep(0.2);

  // | --------------- check if output is enabled --------------- |

  if (!isOutputEnabled()) {
    return {false, "output not enabled by automatic start"};
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

/* land() //{ */

tuple<bool, string> UAVHandler::land(void) {

  auto res = checkPreconditions();

  if (!(std::get<0>(res))) {
    return res;
  }

  if (!isFlyingNormally()) {
    return {false, "not flying normally in the beginning"};
  }

  // | -------------------- call land service ------------------- |

  ROS_INFO("[%s]: calling for landing", name_.c_str());

  {
    std_srvs::Trigger srv;

    {
      bool service_call = sch_land_.call(srv);

      if (!service_call || !srv.response.success) {
        return {false, "land service call failed"};
      }
    }
  }

  // | ---------------------- wait a second --------------------- |

  sleep(1.0);

  // | -------- wait till the right controller is active -------- |

  while (true) {

    if (!ros::ok()) {
      return {false, "shut down from outside"};
    }

    if (sh_control_manager_diag_.getMsg()->active_tracker == "LandoffTracker" && sh_control_manager_diag_.getMsg()->active_controller == "MpcController") {
      break;
    }

    sleep(0.01);
  }

  // | ------------- wait for the landing to finish ------------- |

  while (true) {

    if (!ros::ok()) {
      return {false, "shut down from outside"};
    }

    ROS_INFO_THROTTLE(1.0, "[%s]: waiting for the landing to finish", name_.c_str());

    if (!isOutputEnabled()) {

      return {true, "landing finished"};
    }

    sleep(0.01);
  }

  return {false, "reached end of the method without assertion"};
}

//}

/* landHome() //{ */

tuple<bool, string> UAVHandler::landHome(void) {

  auto res = checkPreconditions();

  if (!(std::get<0>(res))) {
    return res;
  }

  if (!isFlyingNormally()) {
    return {false, "not flying normally in the beginning"};
  }

  // | ----------------- call land home service ----------------- |

  ROS_INFO("[%s]: calling for landing home", name_.c_str());

  {
    std_srvs::Trigger srv;

    {
      bool service_call = sch_land_home_.call(srv);

      if (!service_call || !srv.response.success) {
        return {false, "land home service call failed"};
      }
    }
  }

  // | ---------------------- wait a second --------------------- |

  sleep(1.0);

  // | -------- wait till the right controller is active -------- |

  while (true) {

    if (!ros::ok()) {
      return {false, "shut down from outside"};
    }

    if (sh_control_manager_diag_.getMsg()->active_tracker == "LandoffTracker" && sh_control_manager_diag_.getMsg()->active_controller == "MpcController") {
      break;
    }
  }

  // | ------------- wait for the landing to finish ------------- |

  while (true) {

    if (!ros::ok()) {
      return {false, "shut down from outside"};
    }

    ROS_INFO_THROTTLE(1.0, "[%s]: waiting for the landing to finish", name_.c_str());

    if (!isOutputEnabled()) {

      return {true, "landing finished"};
    }

    sleep(0.01);
  }

  return {false, "reached end of the method without assertion"};
}

//}

/* activateMidAir() //{ */

tuple<bool, string> UAVHandler::activateMidAir(void) {

  auto res = checkPreconditions();

  if (!(std::get<0>(res))) {
    return res;
  }

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

  // | --------------- waiting for flying normally -------------- |

  while (true) {

    if (!ros::ok()) {
      return {false, "shut down from outside"};
    }

    ROS_INFO_THROTTLE(1.0, "[%s]: waiting for the midair activation to finish", name_.c_str());

    auto control_diag = sh_control_manager_diag_.getMsg();

    if (control_diag->flying_normally && control_diag->active_controller != "MidairActivationController" &&
        control_diag->active_tracker != "MidairActivationTracker") {
      return {true, "midair activation finished"};
    }

    if (!control_diag->flying_normally && control_diag->active_controller == "EmergencyController" && control_diag->active_tracker == "LandoffTracker") {
      return {false, "midair activation failed to finish"};
    }

    sleep(0.01);
  }

  return {false, "reached end of the method without assertion"};
}

//}

/* gotoAbs() //{ */

tuple<bool, string> UAVHandler::gotoAbs(const double &x, const double &y, const double &z, const double &hdg) {

  auto res = checkPreconditions();

  if (!(std::get<0>(res))) {
    return res;
  }

  {
    auto [success, message] = gotoService(x, y, z, hdg);

    if (!success) {
      ROS_ERROR("[%s]: goto service failed with message: '%s'", ros::this_node::getName().c_str(), message.c_str());
      return {success, message};
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

/* gotoService() //{ */

tuple<bool, string> UAVHandler::gotoService(const double &x, const double &y, const double &z, const double &hdg) {

  auto res = checkPreconditions();

  if (!(std::get<0>(res))) {
    return res;
  }

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

  return {true, "goto service triggered"};
}

//}

/* gotoRel() //{ */

tuple<bool, string> UAVHandler::gotoRel(const double &x, const double &y, const double &z, const double &hdg) {

  auto res = checkPreconditions();

  if (!(std::get<0>(res))) {
    return res;
  }

  auto start_pose = sh_tracker_cmd_.getMsg()->position;
  auto start_hdg  = sh_tracker_cmd_.getMsg()->heading;

  {
    auto [success, message] = gotoRelativeService(x, y, z, hdg);

    if (!success) {
      return {false, message};
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

/* gotoFcu() //{ */

tuple<bool, string> UAVHandler::gotoFcu(const double &x, const double &y, const double &z, const double &hdg) {

  auto res = checkPreconditions();

  if (!(std::get<0>(res))) {
    return res;
  }

  this->sleep(2.0);

  auto tracker_cmd = getTrackerCmd();

  if (!tracker_cmd) {
    return {false, "missing tracker_cmd"};
  }

  // | ------- get the desired position in the world frame ------ |

  mrs_msgs::ReferenceStamped ref_in;

  ref_in.header.frame_id      = _uav_name_ + "/fcu_untilted";
  ref_in.reference.position.x = x;
  ref_in.reference.position.y = y;
  ref_in.reference.position.z = z;
  ref_in.reference.heading    = hdg;

  auto ref_transformed = transformer_->transformSingle(ref_in, tracker_cmd->header.frame_id);

  if (!ref_transformed) {
    return {false, "could not transform the reference"};
  }

  // | -------------------- call the service -------------------- |

  {
    auto [success, message] = gotoFcuService(x, y, z, hdg);

    if (!success) {
      return {false, message};
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

    if (isAtPosition(ref_transformed->reference.position.x, ref_transformed->reference.position.y, ref_transformed->reference.position.z,
                     ref_transformed->reference.heading, 0.3, ref_transformed->header.frame_id)) {
      return {true, "goal reached"};
    }

    sleep(0.01);
  }

  return {false, "reached end of the method without assertion"};
}

//}

/* gotoReference() //{ */

tuple<bool, string> UAVHandler::gotoReference(const double &x, const double &y, const double &z, const double &hdg, const std::string &frame_id) {

  auto res = checkPreconditions();

  if (!(std::get<0>(res))) {
    return res;
  }

  // | ------------------ create the reference ------------------ |

  mrs_msgs::ReferenceStamped ref_in;

  ref_in.header.frame_id      = frame_id;
  ref_in.reference.position.x = x;
  ref_in.reference.position.y = y;
  ref_in.reference.position.z = z;
  ref_in.reference.heading    = hdg;

  // | ------------------- issue the reference ------------------ |

  {
    auto [success, message] = referenceService(x, y, z, hdg, frame_id);

    if (!success) {
      ROS_ERROR("[%s]: reference service failed with message: '%s'", ros::this_node::getName().c_str(), message.c_str());
      return {success, message};
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

    if (isAtPosition(x, y, z, hdg, 0.1, frame_id)) {
      return {true, "goal reached"};
    }

    sleep(0.01);
  }

  return {false, "reached end of the method without assertion"};
}

//}

/* gotoReferenceTopic() //{ */

tuple<bool, string> UAVHandler::gotoReferenceTopic(const double &x, const double &y, const double &z, const double &hdg, const std::string &frame_id) {

  auto res = checkPreconditions();

  if (!(std::get<0>(res))) {
    return res;
  }

  // | ------------------ create the reference ------------------ |

  mrs_msgs::ReferenceStamped ref_in;

  ref_in.header.frame_id      = frame_id;
  ref_in.reference.position.x = x;
  ref_in.reference.position.y = y;
  ref_in.reference.position.z = z;
  ref_in.reference.heading    = hdg;

  // | ------------------- issue the reference ------------------ |

  {
    auto [success, message] = referenceTopic(x, y, z, hdg, frame_id);

    if (!success) {
      ROS_ERROR("[%s]: reference topic failed with message: '%s'", ros::this_node::getName().c_str(), message.c_str());
      return {success, message};
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

    if (isAtPosition(x, y, z, hdg, 0.1, frame_id)) {
      return {true, "goal reached"};
    }

    sleep(0.01);
  }

  return {false, "reached end of the method without assertion"};
}

//}

/* referenceService() //{ */

tuple<bool, string> UAVHandler::referenceService(const double &x, const double &y, const double &z, const double &hdg, const std::string &frame_id) {

  auto res = checkPreconditions();

  if (!(std::get<0>(res))) {
    return res;
  }

  mrs_msgs::ReferenceStampedSrv srv;

  srv.request.header.frame_id = frame_id;
  srv.request.header.stamp    = ros::Time::now();

  srv.request.reference.position.x = x;
  srv.request.reference.position.y = y;
  srv.request.reference.position.z = z;
  srv.request.reference.heading    = hdg;

  {
    bool service_call = sch_reference_.call(srv);

    if (!service_call || !srv.response.success) {
      return {false, "reference service call failed"};
    }
  }

  return {true, "reference service triggered"};
}

//}

/* referenceTopic() //{ */

tuple<bool, string> UAVHandler::referenceTopic(const double &x, const double &y, const double &z, const double &hdg, const std::string &frame_id) {

  auto res = checkPreconditions();

  if (!(std::get<0>(res))) {
    return res;
  }

  mrs_msgs::ReferenceStamped msg;

  msg.header.frame_id = frame_id;
  msg.header.stamp    = ros::Time::now();

  msg.reference.position.x = x;
  msg.reference.position.y = y;
  msg.reference.position.z = z;
  msg.reference.heading    = hdg;

  ph_reference_.publish(msg);

  return {true, "reference published"};
}

//}

/* gotoRelativeService() //{ */

tuple<bool, string> UAVHandler::gotoRelativeService(const double &x, const double &y, const double &z, const double &hdg) {

  auto res = checkPreconditions();

  if (!(std::get<0>(res))) {
    return res;
  }

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

  return {true, "goto relative service triggered"};
}

//}

/* gotoFcuService() //{ */

tuple<bool, string> UAVHandler::gotoFcuService(const double &x, const double &y, const double &z, const double &hdg) {

  auto res = checkPreconditions();

  if (!(std::get<0>(res))) {
    return res;
  }

  {
    mrs_msgs::Vec4 srv;

    srv.request.goal[0] = x;
    srv.request.goal[1] = y;
    srv.request.goal[2] = z;
    srv.request.goal[3] = hdg;

    {
      bool service_call = sch_goto_fcu_.call(srv);

      if (!service_call || !srv.response.success) {
        return {false, "goto fcu service call failed"};
      }
    }
  }

  return {true, "goto fcu service triggered"};
}

//}

/* setHeading() //{ */

tuple<bool, string> UAVHandler::setHeading(const double &setpoint) {

  auto res = checkPreconditions();

  if (!(std::get<0>(res))) {
    return res;
  }

  mrs_msgs::Vec1 srv;

  srv.request.goal = setpoint;

  {
    bool service_call = sch_set_heading_.call(srv);

    if (!service_call || !srv.response.success) {
      return {false, "set heading service call failed"};
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

    auto heading = getHeading();

    if (!heading) {
      return {false, "could not obtain current heading"};
    }

    if (abs(sradians::diff(setpoint, heading.value())) < 0.1) {
      return {true, "heading goal reached"};
    }

    sleep(0.01);
  }

  return {false, "reached end of the method without assertion"};
}

//}

/* setHeadingRelative() //{ */

tuple<bool, string> UAVHandler::setHeadingRelative(const double &setpoint) {

  auto res = checkPreconditions();

  if (!(std::get<0>(res))) {
    return res;
  }

  // | -------------- remember the initial heading -------------- |

  auto initial_heading = getHeading();

  if (!initial_heading) {
    return {false, "could not obtain the initial heading"};
  }

  // | ------------------ set heading relative ------------------ |

  mrs_msgs::Vec1 srv;

  srv.request.goal = setpoint;

  {
    bool service_call = sch_set_heading_relative_.call(srv);

    if (!service_call || !srv.response.success) {
      return {false, "set heading relative service call failed"};
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

    auto heading = getHeading();

    if (!heading) {
      return {false, "could not obtain current heading"};
    }

    if (abs(sradians::diff(initial_heading.value(), heading.value())) < 0.1) {
      return {true, "heading goal reached"};
    }

    sleep(0.01);
  }

  return {false, "reached end of the method without assertion"};
}

//}

/* gotoAltitude() //{ */

tuple<bool, string> UAVHandler::gotoAltitude(const double &z) {

  auto res = checkPreconditions();

  if (!(std::get<0>(res))) {
    return res;
  }

  // | ----------------- save the initial state ----------------- |

  auto start_pose = sh_tracker_cmd_.getMsg()->position;
  auto start_hdg  = sh_tracker_cmd_.getMsg()->heading;

  // | -------------------- call the service -------------------- |

  mrs_msgs::Vec1 srv;

  srv.request.goal = z;

  {
    bool service_call = sch_goto_altitude_.call(srv);

    if (!service_call || !srv.response.success) {
      return {false, "goto altitude service call failed"};
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

    if (isAtPosition(start_pose.x, start_pose.y, z, start_hdg, 0.1)) {
      return {true, "goal reached"};
    }

    sleep(0.01);
  }

  return {false, "reached end of the method without assertion"};
}

//}

/* setPathSrv() //{ */

tuple<bool, string> UAVHandler::setPathSrv(const mrs_msgs::Path &path_in) {

  auto res = checkPreconditions();

  if (!(std::get<0>(res))) {
    return res;
  }

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

tuple<bool, string> UAVHandler::setPathTopic(const mrs_msgs::Path &path_in) {

  auto res = checkPreconditions();

  if (!(std::get<0>(res))) {
    return res;
  }

  ph_path_.publish(path_in);

  return {true, "path set"};
}

//}

/* switchEstimator() //{ */

tuple<bool, string> UAVHandler::switchEstimator(const std::string &estimator) {

  auto res = checkPreconditions();

  if (!(std::get<0>(res))) {
    return res;
  }

  mrs_msgs::String srv;
  srv.request.value = estimator;

  {
    bool service_call = sch_switch_estimator_.call(srv);

    if (!service_call || !srv.response.success) {
      return {false, "estimator switching service call failed"};
    }
  }

  return {true, "estimator switched"};
}

//}

/* switchController() //{ */

tuple<bool, string> UAVHandler::switchController(const std::string &controller) {

  auto res = checkPreconditions();

  if (!(std::get<0>(res))) {
    return res;
  }

  mrs_msgs::String srv;
  srv.request.value = controller;

  {
    bool service_call = sch_switch_controller_.call(srv);

    if (!service_call || !srv.response.success) {
      return {false, "controller switching service call failed"};
    }
  }

  return {true, "controller switched"};
}

//}

/* switchTracker() //{ */

tuple<bool, string> UAVHandler::switchTracker(const std::string &tracker) {

  auto res = checkPreconditions();

  if (!(std::get<0>(res))) {
    return res;
  }

  mrs_msgs::String srv;
  srv.request.value = tracker;

  {
    bool service_call = sch_switch_tracker_.call(srv);

    if (!service_call || !srv.response.success) {
      return {false, "tracker switching service call failed"};
    }
  }

  return {true, "tracker switched"};
}

//}

/* setGains() //{ */

tuple<bool, string> UAVHandler::setGains(const std::string &gains) {

  auto res = checkPreconditions();

  if (!(std::get<0>(res))) {
    return res;
  }

  mrs_msgs::String srv;
  srv.request.value = gains;

  {
    bool service_call = sch_set_gains_.call(srv);

    if (!service_call || !srv.response.success) {
      return {false, "gain setting service call failed"};
    }
  }

  return {true, "gains set"};
}

//}

/* setConstraints() //{ */

tuple<bool, string> UAVHandler::setConstraints(const std::string &constraints) {

  auto res = checkPreconditions();

  if (!(std::get<0>(res))) {
    return res;
  }

  mrs_msgs::String srv;
  srv.request.value = constraints;

  {
    bool service_call = sch_set_constraints_.call(srv);

    if (!service_call || !srv.response.success) {
      return {false, "gain setting service call failed"};
    }
  }

  return {true, "constraints set"};
}

//}

/* gotoTrajectoryStart() //{ */

tuple<bool, string> UAVHandler::gotoTrajectoryStart() {

  auto res = checkPreconditions();

  if (!(std::get<0>(res))) {
    return res;
  }

  {
    std_srvs::Trigger srv;

    {
      bool service_call = sch_goto_trajectory_start_.call(srv);

      if (!service_call || !srv.response.success) {
        return {false, "goto trajectory start service call failed"};
      }
    }
  }

  sleep(1.0);

  // | -------------------- check for result -------------------- |

  while (true) {

    if (!ros::ok()) {
      return {false, "shut down from outside"};
    }

    if (!isFlyingNormally()) {
      return {false, "not flying normally"};
    }

    if (!hasGoal()) {
      return {true, "goal reached"};
    }

    sleep(0.01);
  }

  return {false, "reached end of the method without assertion"};
}

//}

/* startTrajectoryTracking() //{ */

tuple<bool, string> UAVHandler::startTrajectoryTracking() {

  auto res = checkPreconditions();

  if (!(std::get<0>(res))) {
    return res;
  }

  {
    std_srvs::Trigger srv;

    {
      bool service_call = sch_start_trajectory_tracking_.call(srv);

      if (!service_call || !srv.response.success) {
        return {false, "start trajectory tracking service call failed"};
      }
    }
  }

  sleep(1.0);

  if (sh_control_manager_diag_.getMsg()->tracker_status.tracking_trajectory) {
    return {true, "tracking trajectory"};
  }

  return {false, "failed to start trajectory tracking"};
}

//}

/* stopTrajectoryTracking() //{ */

tuple<bool, string> UAVHandler::stopTrajectoryTracking() {

  auto res = checkPreconditions();

  if (!(std::get<0>(res))) {
    return res;
  }

  {
    std_srvs::Trigger srv;

    {
      bool service_call = sch_stop_trajectory_tracking_.call(srv);

      if (!service_call || !srv.response.success) {
        return {false, "stop trajectory tracking service call failed"};
      }
    }
  }

  sleep(1.0);

  if (!sh_control_manager_diag_.getMsg()->tracker_status.tracking_trajectory) {
    return {true, "tracking trajectory stopped"};
  }

  return {false, "failed to stop trajectory tracking"};
}

//}

/* resumeTrajectoryTracking() //{ */

tuple<bool, string> UAVHandler::resumeTrajectoryTracking() {

  auto res = checkPreconditions();

  if (!(std::get<0>(res))) {
    return res;
  }

  {
    std_srvs::Trigger srv;

    {
      bool service_call = sch_resume_trajectory_tracking_.call(srv);

      if (!service_call || !srv.response.success) {
        return {false, "resume trajectory tracking service call failed"};
      }
    }
  }

  sleep(1.0);

  if (sh_control_manager_diag_.getMsg()->tracker_status.tracking_trajectory) {
    return {true, "tracking trajectory"};
  }

  return {false, "failed to resume trajectory tracking"};
}

//}

/* hover() //{ */

tuple<bool, string> UAVHandler::hover() {

  auto res = checkPreconditions();

  if (!(std::get<0>(res))) {
    return res;
  }

  {
    std_srvs::Trigger srv;

    {
      bool service_call = sch_hover_.call(srv);

      if (!service_call || !srv.response.success) {
        return {false, "hover service call failed"};
      }
    }
  }

  return {true, "hover triggered"};
}

//}

/* getPathSrv() //{ */

tuple<std::optional<mrs_msgs::TrajectoryReference>, string> UAVHandler::getPathSrv(const mrs_msgs::Path &path_in) {

  auto res = checkPreconditions();

  if (!(std::get<0>(res))) {
    return std::make_tuple<std::optional<mrs_msgs::TrajectoryReference>, string>({}, std::string(std::get<1>(res)));
  }

  mrs_msgs::GetPathSrv srv;
  srv.request.path = path_in;

  {
    bool service_call = sch_get_path_.call(srv);

    if (!service_call || !srv.response.success) {
      return std::make_tuple<std::optional<mrs_msgs::TrajectoryReference>, string>({}, "path service call failed");
    }
  }

  return {srv.response.trajectory, "path set"};
}

//}

/* validateReference() //{ */

tuple<bool, string> UAVHandler::validateReference(const mrs_msgs::ReferenceStamped &msg) {

  auto res = checkPreconditions();

  if (!(std::get<0>(res))) {
    return res;
  }

  mrs_msgs::ValidateReference srv;
  srv.request.reference = msg;

  {
    bool service_call = sch_validate_reference_.call(srv);

    if (!service_call) {
      return {false, "reference validation service call failed"};
    } else {
      return {srv.response.success, srv.response.message};
    }
  }
}

//}

/* transformReference() //{ */

std::tuple<bool, std::optional<std::string>, std::optional<mrs_msgs::ReferenceStamped>> UAVHandler::transformReference(const mrs_msgs::ReferenceStamped &msg,
                                                                                                                       std::string target_frame) {

  auto res = checkPreconditions();

  if (!(std::get<0>(res))) {
    return {};
  }

  mrs_msgs::TransformReferenceSrv srv;
  srv.request.reference = msg;
  srv.request.frame_id  = target_frame;

  {
    bool service_call = sch_tranform_reference_.call(srv);

    if (!service_call) {
      return {false, "Reference validation service call failed", {}};
    } else {
      return {srv.response.success, srv.response.message, srv.response.reference};
    }
  }
}

//}

/* transformPose() //{ */

std::tuple<bool, std::optional<std::string>, std::optional<geometry_msgs::PoseStamped>> UAVHandler::transformPose(const geometry_msgs::PoseStamped &msg,
                                                                                                                  std::string target_frame) {

  auto res = checkPreconditions();

  if (!(std::get<0>(res))) {
    return {};
  }

  mrs_msgs::TransformPoseSrv srv;
  srv.request.pose     = msg;
  srv.request.frame_id = target_frame;

  {
    bool service_call = sch_tranform_pose_.call(srv);

    if (!service_call) {
      return {false, "Pose validation service call failed", {}};
    } else {
      return {srv.response.success, srv.response.message, srv.response.pose};
    }
  }
}

//}

/* transformVector3() //{ */

std::tuple<bool, std::optional<std::string>, std::optional<geometry_msgs::Vector3Stamped>> UAVHandler::transformVector3(
    const geometry_msgs::Vector3Stamped &msg, std::string target_frame) {

  auto res = checkPreconditions();

  if (!(std::get<0>(res))) {
    return {};
  }

  mrs_msgs::TransformVector3Srv srv;
  srv.request.vector   = msg;
  srv.request.frame_id = target_frame;

  {
    bool service_call = sch_tranform_vector3_.call(srv);

    if (!service_call) {
      return {false, "Vector3 validation service call failed", {}};
    } else {
      return {srv.response.success, srv.response.message, srv.response.vector};
    }
  }
}

//}

/* ValidateReferenceArray() //{ */

tuple<bool, std::optional<mrs_msgs::ValidateReferenceArray::Response>> UAVHandler::validateReferenceArray(
    const mrs_msgs::ValidateReferenceArray::Request &request) {

  auto res = checkPreconditions();

  if (!(std::get<0>(res))) {
    return {false, {}};
  }

  mrs_msgs::ValidateReferenceArray srv;
  srv.request = request;

  {
    bool service_call = sch_validate_reference_array_.call(srv);

    if (!service_call) {
      return {false, srv.response};
    } else {
      return {true, srv.response};
    }
  }
}

//}

/* sleep() //{ */

void UAVHandler::sleep(const double &duration) {

  ros::Duration(duration).sleep();
}

//}

/* sleep() //{ */

void TestGeneric::sleep(const double &duration) {

  ros::Duration(duration).sleep();
}

//}

// | ------------------------- getters ------------------------ |

/* getHeightAgl() //{ */

std::optional<double> UAVHandler::getHeightAgl(void) {

  auto res = checkPreconditions();

  if (!(std::get<0>(res))) {
    return {};
  }

  if (sh_height_agl_.hasMsg()) {
    return {sh_height_agl_.getMsg()->value};
  } else {
    return {};
  }
}

//}

/* getCurrentConstraints() //{ */

std::optional<mrs_msgs::DynamicsConstraints> UAVHandler::getCurrentConstraints(void) {

  auto res = checkPreconditions();

  if (!(std::get<0>(res))) {
    return {};
  }

  if (sh_current_constraints_.hasMsg()) {
    return {*sh_current_constraints_.getMsg()};
  } else {
    return {};
  }
}

//}

/* getTrackerCmd() //{ */

std::optional<mrs_msgs::TrackerCommand> UAVHandler::getTrackerCmd(void) {

  auto res = checkPreconditions();

  if (!(std::get<0>(res))) {
    return {};
  }

  if (sh_tracker_cmd_.hasMsg()) {
    return {*sh_tracker_cmd_.getMsg()};
  } else {
    return {};
  }
}

//}

/* isAtPosition(const double &x, const double &y, const double &z, const double &hdg, const double &pos_tolerance, const std::string frame_id) //{ */

bool UAVHandler::isAtPosition(const double &x, const double &y, const double &z, const double &hdg, const double &pos_tolerance, const std::string frame_id) {

  auto res = checkPreconditions();

  if (!(std::get<0>(res))) {
    return false;
  }

  if (!sh_uav_state_.hasMsg()) {
    return false;
  }

  auto heading = getHeading();

  if (!heading) {
    return false;
  }

  auto uav_state = sh_uav_state_.getMsg();

  // | ---- transform the input to the current control frame ---- |

  mrs_msgs::ReferenceStamped ref_in;

  ref_in.header.frame_id      = frame_id == "" ? uav_state->header.frame_id : frame_id;
  ref_in.reference.position.x = x;
  ref_in.reference.position.y = y;
  ref_in.reference.position.z = z;
  ref_in.reference.heading    = hdg;

  auto ref_transformed = transformer_->transformSingle(ref_in, uav_state->header.frame_id);

  if (!ref_transformed) {
    return false;
  }

  if (abs(ref_transformed->reference.position.x - uav_state->pose.position.x) < pos_tolerance &&
      abs(ref_transformed->reference.position.y - uav_state->pose.position.y) < pos_tolerance &&
      abs(ref_transformed->reference.position.z - uav_state->pose.position.z) < pos_tolerance &&
      abs(sradians::diff(ref_transformed->reference.heading, heading.value())) < 0.2) {

    return true;

  } else {

    return false;
  }
}

//}

/* isAtPosition(const double &x, const double &y, const double &hdg, const double &pos_tolerance, const std::string frame_id) //{ */

bool UAVHandler::isAtPosition(const double &x, const double &y, const double &hdg, const double &pos_tolerance, const std::string frame_id) {

  auto res = checkPreconditions();

  if (!(std::get<0>(res))) {
    return false;
  }

  if (!sh_uav_state_.hasMsg()) {
    return false;
  }

  auto heading = getHeading();

  if (!heading) {
    return false;
  }

  auto uav_state = sh_uav_state_.getMsg();

  // | ---- transform the input to the current control frame ---- |

  mrs_msgs::ReferenceStamped ref_in;

  ref_in.header.frame_id      = frame_id == "" ? uav_state->header.frame_id : frame_id;
  ref_in.reference.position.x = x;
  ref_in.reference.position.y = y;
  ref_in.reference.position.z = uav_state->pose.position.z;
  ref_in.reference.heading    = hdg;

  auto ref_transformed = transformer_->transformSingle(ref_in, uav_state->header.frame_id);

  if (!ref_transformed) {
    return false;
  }

  if (abs(ref_transformed->reference.position.x - uav_state->pose.position.x) < pos_tolerance &&
      abs(ref_transformed->reference.position.y - uav_state->pose.position.y) < pos_tolerance &&
      abs(sradians::diff(ref_transformed->reference.heading, heading.value())) < 0.2) {

    return true;

  } else {

    return false;
  }
}

//}

/* isReferenceAtPosition() //{ */

bool UAVHandler::isReferenceAtPosition(const double &x, const double &y, const double &z, const double &hdg, const double &pos_tolerance) {

  if (!sh_tracker_cmd_.hasMsg()) {
    return false;
  }

  auto tracker_cmd = sh_tracker_cmd_.getMsg();

  if (abs(x - tracker_cmd->position.x) < pos_tolerance && abs(y - tracker_cmd->position.y) < pos_tolerance &&
      abs(z - tracker_cmd->position.z) < pos_tolerance && abs(sradians::diff(hdg, tracker_cmd->heading)) < 0.2) {

    return true;

  } else {

    return false;
  }
}

//}

/* getActiveTracker() //{ */

std::string UAVHandler::getActiveTracker(void) {

  if (!sh_control_manager_diag_.getMsg()) {
    return "";
  }

  return sh_control_manager_diag_.getMsg()->active_tracker;
}

//}

/* getActiveController() //{ */

std::string UAVHandler::getActiveController(void) {

  if (!sh_control_manager_diag_.getMsg()) {
    return "";
  }

  return sh_control_manager_diag_.getMsg()->active_controller;
}

//}

/* getActiveEstimator() //{ */

std::string UAVHandler::getActiveEstimator(void) {

  if (!sh_estim_manager_diag_.getMsg()) {
    return "";
  }

  return sh_estim_manager_diag_.getMsg()->current_state_estimator;
}

//}

/* hasGoal() //{ */

bool UAVHandler::hasGoal(void) {

  if (sh_control_manager_diag_.hasMsg()) {
    return sh_control_manager_diag_.getMsg()->tracker_status.have_goal;
  } else {
    return false;
  }
}

//}

/* mrsSystemReady() //{ */

bool UAVHandler::mrsSystemReady(void) {

  bool got_control_manager_diag    = sh_control_manager_diag_.hasMsg();
  bool got_uav_manager_diag        = sh_uav_manager_diag_.hasMsg();
  bool got_gain_manager_diag       = sh_gain_manager_diag_.hasMsg();
  bool got_constraint_manager_diag = sh_constraint_manager_diag_.hasMsg();
  bool got_estimation_manager_diag = sh_estim_manager_diag_.hasMsg();
  bool got_uav_state               = sh_uav_state_.hasMsg();

  return got_control_manager_diag && got_estimation_manager_diag && got_uav_manager_diag && got_gain_manager_diag && got_constraint_manager_diag &&
         got_uav_state;
}

//}

/* isFlyingNormally() //{ */

bool UAVHandler::isFlyingNormally(void) {

  if (sh_control_manager_diag_.hasMsg()) {
    return sh_control_manager_diag_.getMsg()->flying_normally;
  } else {
    return false;
  }
}

//}

/* isStationary() //{ */

std::optional<bool> UAVHandler::isStationary(void) {

  if (!sh_control_manager_diag_.hasMsg()) {
    return {};
  }

  auto ctrl_diag = sh_control_manager_diag_.getMsg();

  std::optional<double> speed = getSpeed();

  if (!speed) {
    return {};
  }

  if (!ctrl_diag->tracker_status.have_goal && std::abs(speed.value()) < 0.2) {
    return true;
  } else {
    return false;
  }
}

//}

/* isOutputEnabled() //{ */

bool UAVHandler::isOutputEnabled(void) {

  if (sh_control_manager_diag_.hasMsg()) {
    return sh_control_manager_diag_.getMsg()->output_enabled;
  } else {
    return false;
  }
}

//}

/* getSpeed() //{ */

std::optional<double> UAVHandler::getSpeed(void) {

  if (!sh_speed_.hasMsg()) {
    return {};
  }

  return sh_speed_.getMsg()->value;
}

//}

/* getHeading() //{ */

std::optional<double> UAVHandler::getHeading(void) {

  if (!sh_uav_state_.hasMsg()) {
    return {};
  }

  auto uav_state = sh_uav_state_.getMsg();

  double heading;

  try {
    heading = mrs_lib::AttitudeConverter(uav_state->pose.orientation).getHeading();
  }
  catch (...) {
    return {};
  }

  return {heading};
}

//}

/* getVelocity() //{ */

std::optional<Eigen::Vector3d> UAVHandler::getVelocity(const std::string frame_id) {

  if (!sh_uav_state_.hasMsg()) {
    return {};
  }

  auto uav_state = sh_uav_state_.getMsg();

  geometry_msgs::Vector3Stamped vel_world;

  vel_world.header = uav_state->header;

  vel_world.vector.x = uav_state->velocity.linear.x;
  vel_world.vector.y = uav_state->velocity.linear.y;
  vel_world.vector.z = uav_state->velocity.linear.z;

  auto res = transformer_->transformSingle(vel_world, frame_id);

  if (res) {
    return Eigen::Vector3d(res->vector.x, res->vector.y, res->vector.z);
  } else {
    return {};
  }
}

//}

}  // namespace mrs_uav_testing
