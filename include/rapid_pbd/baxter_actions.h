// This file adapts the BAXTER actuator actions, which use a BAXTER-specific
// interface, to use the more standard interfaces in the control_msgs package.

#ifndef _RAPID_PBD_BAXTER_H_
#define _RAPID_PBD_BAXTER_H_

#include <string>

#include "actionlib/client/simple_action_client.h"
#include "actionlib/server/simple_action_server.h"
#include "control_msgs/GripperCommandAction.h"

#include "rapid_pbd_msgs/ArmControllerState.h"
#include "rapid_pbd_msgs/FreezeArm.h"
#include "rapid_pbd_msgs/RelaxArm.h"

namespace {
typedef actionlib::SimpleActionClient<
    control_msgs::GripperCommandAction>
    BaxterGripperClient;
using control_msgs::GripperCommandFeedback;
using control_msgs::GripperCommandResultConstPtr;
}  // namespace

namespace rapid {
namespace pbd {
namespace baxter {
class GripperAction {
 public:
  GripperAction(const std::string& name, const std::string& baxter_action_name);

  void Start();
  void Execute(const control_msgs::GripperCommandGoalConstPtr& goal);
  void HandleFeedback(const GripperCommandFeedback::ConstPtr& baxter_feedback);

 private:
  actionlib::SimpleActionServer<control_msgs::GripperCommandAction> server_;
  BaxterGripperClient baxter_client_;
};

// ArmControllerManager manages the controllers running on the BAXTER arms.
class ArmControllerManager {
 public:
  ArmControllerManager(const ros::Publisher& state_pub,
                       const ros::ServiceClient& list_client,
                       const ros::ServiceClient& switch_client);

  // Publishes an initial state message.
  void Start();

  bool HandleFreeze(rapid_pbd_msgs::FreezeArmRequest& request,
                    rapid_pbd_msgs::FreezeArmResponse& response);
  bool HandleRelax(rapid_pbd_msgs::RelaxArmRequest& request,
                   rapid_pbd_msgs::RelaxArmResponse& response);

 private:
  void Update();

  ros::Publisher state_pub_;
  ros::ServiceClient list_client_;
  ros::ServiceClient switch_client_;

  bool is_l_arm_active_;
  bool is_r_arm_active_;
};
}  // namespace baxter
}  // namespace pbd
}  // namespace rapid

#endif  // _RAPID_PBD_BAXTER_H_
