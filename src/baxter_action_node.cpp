// Node that runs Baxter action servers using the control_msgs API.

#include "rapid_pbd/action_names.h"
#include "rapid_pbd/baxter_actions.h"
#include "ros/ros.h"

namespace pbd = rapid::pbd;

int main(int argc, char** argv) {
  ros::init(argc, argv, "baxter_actuator_server");
  ros::NodeHandle nh;

  std::vector<std::string> grippers, gripper_types;
  ros::param::param<std::vector<std::string> >("robot_grippers/actuator_groups",
                                               grippers, grippers);
  ros::param::param<std::vector<std::string> >("robot_grippers/gripper_types",
                                               gripper_types, gripper_types);
  size_t right_pos = find(grippers.begin(), grippers.end(), "right gripper") -
                     grippers.begin();
  size_t left_pos =
      find(grippers.begin(), grippers.end(), "left gripper") - grippers.begin();

  pbd::baxter::GripperAction left_gripper(pbd::kLeftGripperActionName,
                                          pbd::baxter::kLeftGripperActionName,
                                          gripper_types[left_pos]);
  pbd::baxter::GripperAction right_gripper(pbd::kRightGripperActionName,
                                           pbd::baxter::kRightGripperActionName,
                                           gripper_types[right_pos]);

  pbd::baxter::HeadAction head_action(pbd::kHeadActionName,
                                      pbd::baxter::kHeadActionName);

  left_gripper.Start();
  right_gripper.Start();
  head_action.Start();
  ros::spin();
  return 0;
}
