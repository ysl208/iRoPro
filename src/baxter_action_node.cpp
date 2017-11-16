// Node that runs Baxter action servers using the control_msgs API.

#include "rapid_pbd/action_names.h"
#include "rapid_pbd/baxter_actions.h"
#include "ros/ros.h"

namespace pbd = rapid::pbd;

int main(int argc, char** argv) {
  ros::init(argc, argv, "baxter_actuator_server");
  ros::NodeHandle nh;

  pbd::baxter::GripperAction left_gripper(pbd::kLeftGripperActionName,
                                          pbd::baxter::kLeftGripperActionName);
  pbd::baxter::GripperAction right_gripper(
      pbd::kRightGripperActionName, pbd::baxter::kRightGripperActionName);

  pbd::baxter::HeadAction head_action(pbd::kHeadActionName,
                                      pbd::baxter::kHeadActionName);

  left_gripper.Start();
  right_gripper.Start();
  head_action.Start();
  ros::spin();
  return 0;
}
