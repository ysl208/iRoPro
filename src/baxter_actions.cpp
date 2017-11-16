#include "rapid_pbd/baxter_actions.h"
#include <string>
#include "actionlib/client/simple_action_client.h"
#include "actionlib/server/simple_action_server.h"
#include "control_msgs/FollowJointTrajectoryAction.h"
#include "control_msgs/GripperCommandAction.h"
#include "control_msgs/SingleJointPositionAction.h"

#include "rapid_pbd_msgs/Action.h"

using actionlib::SimpleClientGoalState;
using control_msgs::FollowJointTrajectoryFeedback;
using control_msgs::FollowJointTrajectoryResult;
using control_msgs::GripperCommandFeedback;
using control_msgs::GripperCommandResult;
using control_msgs::SingleJointPositionFeedback;
using control_msgs::SingleJointPositionGoal;
using control_msgs::SingleJointPositionResult;
namespace msgs = rapid_pbd_msgs;

namespace rapid {
namespace pbd {
namespace baxter {
GripperAction::GripperAction(const std::string& action_name,
                             const std::string& baxter_action_name)
    : server_(action_name, boost::bind(&GripperAction::Execute, this, _1),
              false),
      baxter_client_(baxter_action_name, true) {}

void GripperAction::Start() {
  while (!baxter_client_.waitForServer(ros::Duration(5))) {
    ROS_WARN("Waiting for Baxter gripper server to come up.");
  }
  server_.start();
}

void GripperAction::Execute(
    const control_msgs::GripperCommandGoalConstPtr& goal) {
  control_msgs::GripperCommandGoal baxter_goal;
  // Baxter uses 0-100% corresponding to 0-0.1m
  const float kMaxWidth = 0.1;
  baxter_goal.command.position = goal->command.position / kMaxWidth * 100;
  baxter_goal.command.max_effort = goal->command.max_effort;
  baxter_client_.sendGoal(
      baxter_goal,
      boost::function<void(const SimpleClientGoalState&,
                           const GripperCommandResult::ConstPtr&)>(),
      boost::function<void()>(),
      boost::bind(&GripperAction::HandleFeedback, this, _1));
  while (!baxter_client_.getState().isDone()) {
    if (server_.isPreemptRequested() || !ros::ok()) {
      baxter_client_.cancelAllGoals();
      server_.setPreempted();
      return;
    }
    ros::spinOnce();
  }
  if (baxter_client_.getState() == SimpleClientGoalState::PREEMPTED) {
    baxter_client_.cancelAllGoals();
    server_.setPreempted();
    return;
  } else if (baxter_client_.getState() == SimpleClientGoalState::ABORTED) {
    baxter_client_.cancelAllGoals();
    server_.setAborted();
    return;
  }

  GripperCommandResult::ConstPtr baxter_result = baxter_client_.getResult();
  server_.setSucceeded(*baxter_result);
}

void GripperAction::HandleFeedback(
    const GripperCommandFeedback::ConstPtr& baxter_feedback) {
  server_.publishFeedback(*baxter_feedback);
}

HeadAction::HeadAction(const std::string& head_server_name,
                       const std::string& head_client_name)
    : server_(head_server_name, boost::bind(&HeadAction::Execute, this, _1),
              false),
      baxter_client_(head_client_name, true) {}

void HeadAction::Start() {
  while (!baxter_client_.waitForServer(ros::Duration(5))) {
    ROS_WARN("Waiting for Baxter head server to come up.");
  }
  server_.start();
}

void HeadAction::Execute(
    const control_msgs::FollowJointTrajectoryGoalConstPtr& goal) {
  // baxter head action uses SingleJointPositionAction instead of
  // FollowJointTrajectoryAction
  control_msgs::SingleJointPositionGoal baxter_head_goal;
  baxter_head_goal.position = goal->trajectory.points[0].positions[0];
  baxter_head_goal.max_velocity = 1.0;
  baxter_client_.sendGoal(baxter_head_goal);
  while (!baxter_client_.getState().isDone()) {
    if (server_.isPreemptRequested() || !ros::ok()) {
      baxter_client_.cancelAllGoals();
      server_.setPreempted();
      return;
    }
    ros::spinOnce();
  }
  if (baxter_client_.getState() == SimpleClientGoalState::PREEMPTED) {
    baxter_client_.cancelAllGoals();
    server_.setPreempted();
    return;
  } else if (baxter_client_.getState() == SimpleClientGoalState::ABORTED) {
    baxter_client_.cancelAllGoals();
    server_.setAborted();
    return;
  }

  SingleJointPositionResult::ConstPtr baxter_result =
      baxter_client_.getResult();
  // convert baxter client result to FollowJointTrajectory again
  control_msgs::FollowJointTrajectoryResult result;
  result.error_code = 0;

  // To Do: generate correct result, but SingleJointPositionResult msg seems to
  // be empty
  server_.setSucceeded();
}

}  // namespace baxter
}  // namespace pbd
}  // namespace rapid
