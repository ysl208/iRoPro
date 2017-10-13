#include "rapid_pbd/baxter_actions.h"

#include <string>

#include "actionlib/client/simple_action_client.h"
#include "actionlib/server/simple_action_server.h"
#include "control_msgs/FollowJointTrajectoryAction.h"
#include "control_msgs/GripperCommandAction.h"
#include "control_msgs/SingleJointPositionAction.h"
#include "controller_manager_msgs/ListControllers.h"
#include "controller_manager_msgs/SwitchController.h"

#include "rapid_pbd_msgs/Action.h"
#include "rapid_pbd_msgs/ArmControllerState.h"
#include "rapid_pbd_msgs/FreezeArm.h"
#include "rapid_pbd_msgs/RelaxArm.h"

using actionlib::SimpleClientGoalState;
using control_msgs::FollowJointTrajectoryFeedback;
using control_msgs::FollowJointTrajectoryResult;
using control_msgs::GripperCommandFeedback;
using control_msgs::GripperCommandResult;
using control_msgs::SingleJointPositionFeedback;
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
  baxter_goal.command.position = goal->command.position;
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
  control_msgs::GripperCommandResult result;
  result.effort = baxter_result->effort;
  result.position = baxter_result->position;
  result.reached_goal = baxter_result->reached_goal;
  result.stalled = baxter_result->stalled;
  server_.setSucceeded(result);
}

void GripperAction::HandleFeedback(
    const GripperCommandFeedback::ConstPtr& baxter_feedback) {
  control_msgs::GripperCommandFeedback feedback;
  feedback.effort = baxter_feedback->effort;
  feedback.position = baxter_feedback->position;
  feedback.reached_goal = baxter_feedback->reached_goal;
  feedback.stalled = baxter_feedback->stalled;
  server_.publishFeedback(feedback);
}

ArmControllerManager::ArmControllerManager(
    const ros::Publisher& state_pub, const ros::ServiceClient& list_client,
    const ros::ServiceClient& switch_client)
    : state_pub_(state_pub),
      list_client_(list_client),
      switch_client_(switch_client),
      is_l_arm_active_(true),
      is_r_arm_active_(true) {}

void ArmControllerManager::Start() { Update(); }

bool ArmControllerManager::HandleFreeze(msgs::FreezeArmRequest& request,
                                        msgs::FreezeArmResponse& response) {
  controller_manager_msgs::SwitchControllerRequest req;
  req.strictness = controller_manager_msgs::SwitchControllerRequest::BEST_EFFORT;
  controller_manager_msgs::SwitchControllerResponse res;
  if (request.actuator_group == msgs::Action::LEFT_ARM) {
    req.start_controllers.push_back("l_arm_controller");
  } else if (request.actuator_group == msgs::Action::RIGHT_ARM) {
    req.start_controllers.push_back("r_arm_controller");
  } else {
    response.error =
        "Invalid actuator group \"" + request.actuator_group + "\"";
    ROS_ERROR("%s", response.error.c_str());
    return true;
  }
  bool success = switch_client_.call(req, res);
  if (!success) {
    response.error = "Failed to freeze \"" + request.actuator_group + "\"";
    ROS_ERROR("%s", response.error.c_str());
    return true;
  }
  Update();

  return true;
}

bool ArmControllerManager::HandleRelax(msgs::RelaxArmRequest& request,
                                       msgs::RelaxArmResponse& response) {
  controller_manager_msgs::SwitchControllerRequest req;
  req.strictness = controller_manager_msgs::SwitchControllerRequest::BEST_EFFORT;
  controller_manager_msgs::SwitchControllerResponse res;
  if (request.actuator_group == msgs::Action::LEFT_ARM) {
    req.stop_controllers.push_back("l_arm_controller");
  } else if (request.actuator_group == msgs::Action::RIGHT_ARM) {
    req.stop_controllers.push_back("r_arm_controller");
  } else {
    response.error =
        "Invalid actuator group \"" + request.actuator_group + "\"";
    ROS_ERROR("%s", response.error.c_str());
    return true;
  }
  bool success = switch_client_.call(req, res);
  if (!success) {
    response.error = "Failed to relax \"" + request.actuator_group + "\"";
    ROS_ERROR("%s", response.error.c_str());
    return true;
  }
  Update();

  return true;
}

void ArmControllerManager::Update() {
  controller_manager_msgs::ListControllersRequest req;
  controller_manager_msgs::ListControllersResponse res;
  while (!list_client_.waitForExistence(ros::Duration(5))) {
    ROS_WARN("Waiting for baxter_controller_manager list service...");
  }
  bool success = list_client_.call(req, res);
  if (!success) {
    ROS_ERROR("baxter_controller_manager list service call failed.");
  }
  for (size_t i = 0; i < res.controller.size(); ++i) {
    const std::string& name = res.controller[i].name;
    const std::string& state = res.controller[i].state;
    bool is_running = (state == "running");
    if (name == "l_arm_controller") {
      is_l_arm_active_ = is_running;
    } else if (name == "r_arm_controller") {
      is_r_arm_active_ = is_running;
    }
  }

  msgs::ArmControllerState msg;
  if (is_l_arm_active_) {
    msg.l_arm_controller = msgs::ArmControllerState::FROZEN;
  } else {
    msg.l_arm_controller = msgs::ArmControllerState::RELAXED;
  }
  if (is_r_arm_active_) {
    msg.r_arm_controller = msgs::ArmControllerState::FROZEN;
  } else {
    msg.r_arm_controller = msgs::ArmControllerState::RELAXED;
  }
  state_pub_.publish(msg);
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
      // baxter head action uses SingleJointPositionAction instead of FollowJointTrajectoryAction
  control_msgs::SingleJointPositionGoal baxter_head_goal;
  baxter_head_goal.position = goal->trajectory.points[0].positions[0];
  baxter_head_goal.max_velocity = 1.0;

  baxter_client_.sendGoal(
      baxter_head_goal,
      boost::function<void(const SimpleClientGoalState&,
                           const SingleJointPositionResult::ConstPtr&)>(),
      boost::function<void()>(),
      boost::bind(&HeadAction::HandleFeedback, this, _1));
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

  SingleJointPositionResult::ConstPtr baxter_result = baxter_client_.getResult();
  // convert baxter client result to FollowJointTrajectory again
  control_msgs::FollowJointTrajectoryResult result;
  // result.effort = baxter_result->effort;
  // result.position = baxter_result->position;
  // result.reached_goal = baxter_result->reached_goal;
  // result.stalled = baxter_result->stalled;
  server_.setSucceeded(result);
}

void HeadAction::HandleFeedback(
    const SingleJointPositionFeedback::ConstPtr& baxter_feedback) {
  control_msgs::FollowJointTrajectoryFeedback feedback;
  // feedback.effort = baxter_feedback->effort;
  feedback.actual.positions[0] = baxter_feedback->position;
  feedback.actual.velocities[0] = baxter_feedback->velocity;
  
  // feedback.reached_goal = baxter_feedback->reached_goal;
  // feedback.stalled = baxter_feedback->stalled;
  server_.publishFeedback(feedback);
}

}  // namespace baxter
}  // namespace pbd
}  // namespace rapid
