#ifndef _RAPID_PBD_UR_ACTIONS_H_
#define _RAPID_PBD_UR_ACTIONS_H_

#include "actionlib/client/simple_action_client.h"
#include "actionlib/server/simple_action_server.h"
#include "control_msgs/GripperCommandAction.h"
#include "control_msgs/FollowJointTrajectoryAction.h"
#include "rapid_pbd_msgs/ArmControllerState.h"
#include "ros/ros.h"

namespace {
typedef actionlib::SimpleActionClient<control_msgs::GripperCommandAction>
    UniversalRobotGripperClient;
using control_msgs::GripperCommandFeedback;
using control_msgs::GripperCommandResultConstPtr;
}  // namespace

namespace msgs = rapid_pbd_msgs;
namespace rapid {
namespace pbd {
namespace ur {

class GripperAction {
 public:
  GripperAction(const std::string& name, const std::string& ur_action_name);

  void Start();
  void Execute(const control_msgs::GripperCommandGoalConstPtr& goal);
  void HandleFeedback(const GripperCommandFeedback::ConstPtr& ur_feedback);

 private:
  actionlib::SimpleActionServer<control_msgs::GripperCommandAction> server_;
  UniversalRobotGripperClient ur_client_;
};

// ArmControllerManager manages the UR arm controllers.
// class ArmControllerManager {
//  public:
//   ArmControllerManager(
//       const ros::Publisher& state_pub,
//       actionlib::SimpleActionClient<
//           control_msgs::FollowJointTrajectoryAction>* client);

//   // Publishes an initial state message.
//   void Start();

// //   bool HandleFreeze(msgs::FreezeArmRequest& request,
// //                     msgs::FreezeArmResponse& response);
// //   bool HandleRelax(msgs::RelaxArmRequest& request,
// //                    msgs::RelaxArmResponse& response);

//  private:
//   void Update();

//   ros::Publisher state_pub_;
//   actionlib::SimpleActionClient<
//       control_msgs::FollowJointTrajectoryAction>* client_;

//   bool is_arm_active_;
// };

}  // namespace ur
}  // namespace pbd
}  // namespace rapid

#endif  // _RAPID_PBD_UR_ACTIONS_H_
