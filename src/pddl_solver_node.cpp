#include <string>

#include "pddl_msgs/PDDLPlannerAction.h"
#include "rapid_pbd/action_names.h"
#include "rapid_pbd/pddl_solver_action.h"
#include "ros/ros.h"

namespace pbd = rapid::pbd;

int main(int argc, char** argv) {
  ros::init(argc, argv, "pddl_solver_node");
  ros::NodeHandle nh;
  if (argc < 2) {
    ROS_ERROR("Must supply pddl topic as arg");
    return 1;
  }
  std::string topic(argv[1]);
  ROS_INFO("PDDLSolverAction node topics: client publishing on %s and listening to server %s", topic.c_str(),
           pbd::kPDDLSolverActionName);
  rapid::pbd::PDDLSolverAction action(pbd::kPDDLSolverActionName, topic);
  action.Start();
  ros::spin();
  return 0;
}
