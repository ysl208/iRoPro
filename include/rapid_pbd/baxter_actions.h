// This file adapts the BAXTER actuator actions, which use a BAXTER-specific
// interface, to use the more standard interfaces in the control_msgs package.

#ifndef _RAPID_PBD_BAXTER_H_
#define _RAPID_PBD_BAXTER_H_

#include <string>

#include "actionlib/client/simple_action_client.h"
#include "actionlib/server/simple_action_server.h"
#include "control_msgs/FollowJointTrajectoryAction.h"
#include "control_msgs/GripperCommandAction.h"
#include "control_msgs/SingleJointPositionAction.h"

namespace {
typedef actionlib::SimpleActionClient<control_msgs::GripperCommandAction>
    BaxterGripperClient;
using control_msgs::GripperCommandFeedback;
using control_msgs::GripperCommandResultConstPtr;
}  // namespace

namespace {
typedef actionlib::SimpleActionClient<control_msgs::SingleJointPositionAction>
    BaxterHeadClient;
using control_msgs::SingleJointPositionFeedback;
using control_msgs::SingleJointPositionResultConstPtr;
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

// HeadAction manages the controllers running on the BAXTER head.
class HeadAction {
 public:
  HeadAction(const std::string& head_server_name,
             const std::string& head_client_name);

  void Start();
  void Execute(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal);
  void HandleFeedback(
      const SingleJointPositionFeedback::ConstPtr& baxter_feedback);

 private:
  actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction>
      server_;
  BaxterHeadClient baxter_client_;
};

}  // namespace baxter
}  // namespace pbd
}  // namespace rapid

#endif  // _RAPID_PBD_BAXTER_H_
