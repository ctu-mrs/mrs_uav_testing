#ifndef TEST_GENERIC_H
#define TEST_GENERIC_H

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

#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>

namespace mrs_uav_testing
{

using radians  = mrs_lib::geometry::radians;
using sradians = mrs_lib::geometry::sradians;

using namespace std;

/* TestGeneric //{ */

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
  tuple<bool, string> activateMidAir(void);

  tuple<bool, string> gotoAbs(const double &x, const double &y, const double &z, const double &hdg);
  tuple<bool, string> gotoRel(const double &x, const double &y, const double &z, const double &hdg);
  tuple<bool, string> spawnGazeboUav();

  void sleep(const double &duration);

  bool        hasGoal(void);
  bool        isFlyingNormally(void);
  bool        isOutputEnabled(void);
  bool        isAtPosition(const double &x, const double &y, const double &z, const double &hdg, const double &pos_tolerance);
  std::string getActiveTracker(void);
  std::string getActiveController(void);

  mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics>    sh_control_manager_diag_;
  mrs_lib::SubscribeHandler<mrs_msgs::UavManagerDiagnostics>        sh_uav_manager_diag_;
  mrs_lib::SubscribeHandler<mrs_msgs::EstimationDiagnostics>        sh_estim_manager_diag_;
  mrs_lib::SubscribeHandler<mrs_msgs::GainManagerDiagnostics>       sh_gain_manager_diag_;
  mrs_lib::SubscribeHandler<mrs_msgs::ConstraintManagerDiagnostics> sh_constraint_manager_diag_;
  mrs_lib::SubscribeHandler<mrs_msgs::GazeboSpawnerDiagnostics>     sh_gazebo_spawner_diag_;
  mrs_lib::SubscribeHandler<mrs_msgs::UavState>                     sh_uav_state_;

  mrs_lib::SubscribeHandler<mrs_msgs::HwApiStatus> sh_hw_api_status_;

  mrs_lib::ServiceClientHandler<std_srvs::SetBool> sch_arming_;
  mrs_lib::ServiceClientHandler<std_srvs::Trigger> sch_offboard_;
  mrs_lib::ServiceClientHandler<std_srvs::Trigger> sch_midair_activation_;
  mrs_lib::ServiceClientHandler<mrs_msgs::String>  sch_spawn_gazebo_uav_;

  mrs_lib::ServiceClientHandler<mrs_msgs::Vec4>    sch_goto_;
  mrs_lib::ServiceClientHandler<mrs_msgs::PathSrv> sch_path_;
  mrs_lib::ServiceClientHandler<mrs_msgs::Vec4>    sch_goto_relative_;

  tuple<bool, string> setPathSrv(const mrs_msgs::Path &path_in);

  string _uav_name_;
  string _gazebo_spawner_params_;
  string _test_name_;
  string name_;

  bool mrsSystemReady(void);

private:
  shared_ptr<ros::AsyncSpinner> spinner_;
};

//}

}  // namespace mrs_uav_testing

#endif  // TEST_GENERIC_H
