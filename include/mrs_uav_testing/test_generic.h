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

#include <mrs_msgs/ControlManagerDiagnostics.h>
#include <mrs_msgs/UavManagerDiagnostics.h>
#include <mrs_msgs/HwApiStatus.h>
#include <mrs_msgs/EstimationDiagnostics.h>
#include <mrs_msgs/Vec4.h>
#include <mrs_msgs/GainManagerDiagnostics.h>
#include <mrs_msgs/ConstraintManagerDiagnostics.h>

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
  TestGeneric(const string &test_name);
  TestGeneric(const string &test_name, const string &uav_name);

  void initialize(void);

  virtual bool test() = 0;

  tuple<bool, string> takeoff(void);
  tuple<bool, string> activateMidAir(void);

  tuple<bool, string> gotoAbs(const double &x, const double &y, const double &z, const double &hdg);
  tuple<bool, string> gotoRel(const double &x, const double &y, const double &z, const double &hdg);

  void sleep(const double &duration);

  bool mrsSystemReady(void);

  bool flyingNormally(void);

  bool canTakeOff(void);

protected:
  ros::NodeHandle nh_;

  string name_;

  mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics>    sh_control_manager_diag_;
  mrs_lib::SubscribeHandler<mrs_msgs::UavManagerDiagnostics>        sh_uav_manager_diag_;
  mrs_lib::SubscribeHandler<mrs_msgs::EstimationDiagnostics>        sh_estim_manager_diag_;
  mrs_lib::SubscribeHandler<mrs_msgs::GainManagerDiagnostics>       sh_gain_manager_diag_;
  mrs_lib::SubscribeHandler<mrs_msgs::ConstraintManagerDiagnostics> sh_constraint_manager_diag_;

  mrs_lib::SubscribeHandler<mrs_msgs::HwApiStatus> sh_hw_api_status_;

  mrs_lib::ServiceClientHandler<std_srvs::SetBool> sch_arming_;
  mrs_lib::ServiceClientHandler<std_srvs::Trigger> sch_offboard_;
  mrs_lib::ServiceClientHandler<std_srvs::Trigger> sch_midair_activation_;

  mrs_lib::ServiceClientHandler<mrs_msgs::Vec4> sch_goto_;
  mrs_lib::ServiceClientHandler<mrs_msgs::Vec4> sch_goto_relative_;

private:
  shared_ptr<ros::AsyncSpinner> spinner_;

  string _uav_name_;
  string _test_name_;
};

//}

}  // namespace mrs_uav_testing

#endif  // TEST_GENERIC_H
