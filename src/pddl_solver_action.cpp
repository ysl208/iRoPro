#include "rapid_pbd/pddl_solver_action.h"
#include "rapid_pbd/action_names.h"

#include <sstream>
#include <string>

#include "actionlib/client/simple_action_client.h"
#include "actionlib/server/simple_action_server.h"
#include "pddl_msgs/PDDLPlannerAction.h"
#include "ros/ros.h"

using actionlib::SimpleClientGoalState;
using pddl_msgs::PDDLPlannerFeedback;
using pddl_msgs::PDDLPlannerGoal;
using pddl_msgs::PDDLPlannerResult;
namespace msgs = pddl_msgs;
namespace rapid {
namespace pbd {
PDDLSolverAction::PDDLSolverAction(const std::string& pddl_server_name,
                                   const std::string& pddl_client_name)
    : as_(pddl_server_name, boost::bind(&PDDLSolverAction::Execute, this, _1),
          false),
      pddl_client_(pddl_client_name, true),
      nh_() {}

void PDDLSolverAction::Start() {
  while (!pddl_client_.waitForServer(ros::Duration(5))) {
    ROS_WARN("Waiting for pddl planner server to come up.");
  }
  as_.start();
}

void PDDLSolverAction::Execute(const msgs::PDDLPlannerGoalConstPtr& pddl_goal) {
  msgs::PDDLPlannerGoal goal = *pddl_goal;
  ros::Time start = ros::Time::now();
  // ROS_INFO("PDDLSolverAction::Execute... ");
  pddl_client_.sendGoal(
      goal,
      boost::function<void(const SimpleClientGoalState&,
                           const msgs::PDDLPlannerResult::ConstPtr&)>(),
      boost::function<void()>(),
      boost::bind(&PDDLSolverAction::HandleFeedback, this, _1));
  while (!pddl_client_.getState().isDone()) {
    if (as_.isPreemptRequested() || !ros::ok()) {
      pddl_client_.cancelAllGoals();
      as_.setPreempted();
      return;
    }
    ros::spinOnce();
  }
  if (pddl_client_.getState() == SimpleClientGoalState::PREEMPTED) {
    pddl_client_.cancelAllGoals();
    as_.setPreempted();
    return;
  } else if (pddl_client_.getState() == SimpleClientGoalState::ABORTED) {
    pddl_client_.cancelAllGoals();
    as_.setAborted();
    return;
  }

  msgs::PDDLPlannerResult::ConstPtr result = pddl_client_.getResult();
  as_.setSucceeded(*result);
}

void PDDLSolverAction::HandleFeedback(
    const msgs::PDDLPlannerFeedback::ConstPtr& pddl_feedback) {
  as_.publishFeedback(*pddl_feedback);
}
}  // namespace pbd
}  // namespace rapid
