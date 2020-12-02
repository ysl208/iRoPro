#include "rapid_pbd/action_names.h"
#include "rapid_pbd/ur_actions.h"
#include "rapid_pbd_msgs/ArmControllerState.h"
#include "control_msgs/FollowJointTrajectoryAction.h"
#include "ros/ros.h"

namespace pbd = rapid::pbd;
namespace msgs = rapid_pbd_msgs;

int main(int argc, char** argv) {
  ros::init(argc, argv, "ur_actuator_server");
  ros::NodeHandle nh;

  std::vector<std::string> grippers;
  ros::param::param<std::vector<std::string> >("robot_grippers/actuator_groups",
                                               grippers, grippers);
  size_t pos = find(grippers.begin(), grippers.end(), "right gripper") -
                     grippers.begin();
  pbd::ur::GripperAction gripper(pbd::kGripperActionName,
                                           pbd::ur::kGripperActionName);
  gripper.Start();

//  TO DO
  // ros::Publisher arm_controller_state_pub =
  //     nh.advertise<msgs::ArmControllerState>(pbd::kArmControllerStateTopic, 5,
  //                                            true);
  // actionlib::SimpleActionClient<
  //     robot_controllers_msgs::QueryControllerStatesAction>
  //     client(pbd::ur::kControllerActionName, true);
  // while (ros::ok() && !client.waitForServer(ros::Duration(5.0))) {
  //   ROS_WARN("Waiting for arm controller manager." );
  // }
  // pbd::ur::ArmControllerManager arm_controller_manager(
  //     arm_controller_state_pub, &client);
  // ros::ServiceServer freeze_srv = nh.advertiseService(
  //     pbd::kFreezeArmService, &pbd::ur::ArmControllerManager::HandleFreeze,
  //     &arm_controller_manager);
  // ros::ServiceServer relax_srv = nh.advertiseService(
  //     pbd::kRelaxArmService, &pbd::ur::ArmControllerManager::HandleRelax,
  //     &arm_controller_manager);

  // arm_controller_manager.Start();
  ros::spin();
  return 0;
}
