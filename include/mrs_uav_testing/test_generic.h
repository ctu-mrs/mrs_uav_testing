#ifndef TEST_GENERIC_H
#define TEST_GENERIC_H

/* includes //{ */

#include <ros/ros.h>
#include <ros/console.h>
#include <log4cxx/logger.h>

#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/service_client_handler.h>
#include <mrs_lib/transformer.h>
#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/geometry/cyclic.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/publisher_handler.h>

#include <mrs_msgs/ControlManagerDiagnostics.h>
#include <mrs_msgs/UavManagerDiagnostics.h>
#include <mrs_msgs/HwApiStatus.h>
#include <mrs_msgs/EstimationDiagnostics.h>
#include <mrs_msgs/Vec4.h>
#include <mrs_msgs/GainManagerDiagnostics.h>
#include <mrs_msgs/ConstraintManagerDiagnostics.h>
#include <mrs_msgs/GazeboSpawnerDiagnostics.h>
#include <mrs_msgs/String.h>
#include <mrs_msgs/UavState.h>
#include <mrs_msgs/PathSrv.h>
#include <mrs_msgs/Float64Stamped.h>
#include <mrs_msgs/TrackerCommand.h>
#include <mrs_msgs/DynamicsConstraints.h>
#include <mrs_msgs/String.h>

#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>

//}

namespace mrs_uav_testing
{

using radians  = mrs_lib::geometry::radians;
using sradians = mrs_lib::geometry::sradians;

using namespace std;

class TestGeneric {

public:
  TestGeneric();

  void initialize(void);

  virtual bool test() = 0;

  std::shared_ptr<mrs_lib::ParamLoader> pl_;

protected:
  ros::NodeHandle                       nh_;
  std::shared_ptr<mrs_lib::Transformer> transformer_;

  mrs_lib::SubscribeHandlerOptions shopts_;

  tuple<bool, string> takeoff(void);
  tuple<bool, string> land(void);
  tuple<bool, string> landHome(void);
  tuple<bool, string> activateMidAir(void);

  tuple<bool, string> gotoAbs(const double &x, const double &y, const double &z, const double &hdg);
  tuple<bool, string> gotoRel(const double &x, const double &y, const double &z, const double &hdg);
  tuple<bool, string> spawnGazeboUav();

  void sleep(const double &duration);

  bool hasGoal(void);
  bool isFlyingNormally(void);
  bool isOutputEnabled(void);
  bool isAtPosition(const double &x, const double &y, const double &z, const double &hdg, const double &pos_tolerance);
  bool isReferenceAtPosition(const double &x, const double &y, const double &z, const double &hdg, const double &pos_tolerance);

  std::string                                  getActiveTracker(void);
  std::string                                  getActiveController(void);
  std::string                                  getActiveEstimator(void);
  std::optional<mrs_msgs::TrackerCommand>      getTrackerCmd(void);
  std::optional<double>                        getHeightAgl(void);
  std::optional<mrs_msgs::DynamicsConstraints> getCurrentConstraints(void);

  mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics>    sh_control_manager_diag_;
  mrs_lib::SubscribeHandler<mrs_msgs::DynamicsConstraints>          sh_current_constraints_;
  mrs_lib::SubscribeHandler<mrs_msgs::UavManagerDiagnostics>        sh_uav_manager_diag_;
  mrs_lib::SubscribeHandler<mrs_msgs::EstimationDiagnostics>        sh_estim_manager_diag_;
  mrs_lib::SubscribeHandler<mrs_msgs::GainManagerDiagnostics>       sh_gain_manager_diag_;
  mrs_lib::SubscribeHandler<mrs_msgs::ConstraintManagerDiagnostics> sh_constraint_manager_diag_;
  mrs_lib::SubscribeHandler<mrs_msgs::GazeboSpawnerDiagnostics>     sh_gazebo_spawner_diag_;
  mrs_lib::SubscribeHandler<mrs_msgs::UavState>                     sh_uav_state_;
  mrs_lib::SubscribeHandler<mrs_msgs::TrackerCommand>               sh_tracker_cmd_;
  mrs_lib::SubscribeHandler<mrs_msgs::Float64Stamped>               sh_height_agl_;

  mrs_lib::SubscribeHandler<mrs_msgs::HwApiStatus> sh_hw_api_status_;

  mrs_lib::ServiceClientHandler<std_srvs::SetBool> sch_arming_;
  mrs_lib::ServiceClientHandler<std_srvs::Trigger> sch_offboard_;
  mrs_lib::ServiceClientHandler<std_srvs::Trigger> sch_midair_activation_;
  mrs_lib::ServiceClientHandler<mrs_msgs::String>  sch_spawn_gazebo_uav_;
  mrs_lib::ServiceClientHandler<std_srvs::Trigger> sch_land_;
  mrs_lib::ServiceClientHandler<std_srvs::Trigger> sch_land_home_;
  mrs_lib::ServiceClientHandler<mrs_msgs::String>  sch_switch_estimator_;

  mrs_lib::ServiceClientHandler<mrs_msgs::Vec4>    sch_goto_;
  mrs_lib::ServiceClientHandler<mrs_msgs::PathSrv> sch_path_;
  mrs_lib::ServiceClientHandler<mrs_msgs::Vec4>    sch_goto_relative_;

  mrs_lib::PublisherHandler<mrs_msgs::Path> ph_path_;

  tuple<bool, string> setPathSrv(const mrs_msgs::Path &path_in);
  tuple<bool, string> setPathTopic(const mrs_msgs::Path &path_in);
  tuple<bool, string> switchEstimator(const std::string &estimator);

  string _uav_name_;
  string _gazebo_spawner_params_;
  string _test_name_;
  string name_;

  bool mrsSystemReady(void);

private:
  shared_ptr<ros::AsyncSpinner> spinner_;
};

}  // namespace mrs_uav_testing

#endif  // TEST_GENERIC_H
