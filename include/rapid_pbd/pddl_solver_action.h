#ifndef _RAPID_PBD_PDDL_SOLVER_ACTION_H_
#define _RAPID_PBD_PDDL_SOLVER_ACTION_H_

#include <string>

#include "actionlib/client/simple_action_client.h"
#include "actionlib/server/simple_action_server.h"
#include "pddl_msgs/PDDLPlannerAction.h"
#include "ros/ros.h"

using pddl_msgs::PDDLPlannerFeedback;
using pddl_msgs::PDDLPlannerResultConstPtr;

namespace msgs = pddl_msgs;
namespace rapid {
namespace pbd {
class PDDLSolverAction {
 public:
  PDDLSolverAction(const std::string& pddl_server_name,
                   const std::string& pddl_client_name);
  void Start();
  void Execute(const msgs::PDDLPlannerGoalConstPtr& goal);
  void HandleFeedback(const PDDLPlannerFeedback::ConstPtr& pddl_feedback);

 private:
  std::string topic_;
  actionlib::SimpleActionServer<msgs::PDDLPlannerAction> as_;
  actionlib::SimpleActionClient<msgs::PDDLPlannerAction> pddl_client_;
  ros::NodeHandle nh_;
};
}  // namespace pbd
}  // namespace rapid

#endif  // _RAPID_PBD_PDDL_SOLVER_ACTION_H_
