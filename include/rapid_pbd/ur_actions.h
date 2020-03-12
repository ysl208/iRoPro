#ifndef _RAPID_PBD_UR_ACTIONS_H_
#define _RAPID_PBD_UR_ACTIONS_H_

#include "actionlib/client/simple_action_client.h"
#include "rapid_pbd_msgs/ArmControllerState.h"
#include "rapid_pbd_msgs/FreezeArm.h"
#include "rapid_pbd_msgs/RelaxArm.h"
#include "robot_controllers_msgs/QueryControllerStatesAction.h"
#include "ros/ros.h"

namespace msgs = rapid_pbd_msgs;
namespace rapid {
namespace pbd {
namespace ur {
// ArmControllerManager manages the UR arm controllers.
class ArmControllerManager {
 public:
  ArmControllerManager(
      const ros::Publisher& state_pub,
      actionlib::SimpleActionClient<
          robot_controllers_msgs::QueryControllerStatesAction>* client);

  // Publishes an initial state message.
  void Start();

  bool HandleFreeze(msgs::FreezeArmRequest& request,
                    msgs::FreezeArmResponse& response);
  bool HandleRelax(msgs::RelaxArmRequest& request,
                   msgs::RelaxArmResponse& response);

 private:
  void Update();

  ros::Publisher state_pub_;
  actionlib::SimpleActionClient<
      robot_controllers_msgs::QueryControllerStatesAction>* client_;

  bool is_arm_active_;
};
}  // namespace ur
}  // namespace pbd
}  // namespace rapid

#endif  // _RAPID_PBD_UR_ACTIONS_H_
