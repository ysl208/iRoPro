#include "rapid_pbd/baxter_actions.h"

#include <string>

#include "actionlib/client/simple_action_client.h"
#include "actionlib/server/simple_action_server.h"
#include "control_msgs/GripperCommandAction.h"
#include "baxter_controllers_msgs/BaxterGripperCommandAction.h"
#include "baxter_mechanism_msgs/ListControllers.h"
#include "baxter_mechanism_msgs/SwitchController.h"

#include "rapid_pbd_msgs/Action.h"
#include "rapid_pbd_msgs/ArmControllerState.h"
#include "rapid_pbd_msgs/FreezeArm.h"
#include "rapid_pbd_msgs/RelaxArm.h"

using actionlib::SimpleClientGoalState;
using baxter_controllers_msgs::BaxterGripperCommandFeedback;
using baxter_controllers_msgs::BaxterGripperCommandResult;
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
  baxter_controllers_msgs::BaxterGripperCommandGoal baxter_goal;
  baxter_goal.command.position = goal->command.position;
  baxter_goal.command.max_effort = goal->command.max_effort;
  baxter_client_.sendGoal(
      baxter_goal,
      boost::function<void(const SimpleClientGoalState&,
                           const BaxterGripperCommandResult::ConstPtr&)>(),
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

  BaxterGripperCommandResult::ConstPtr baxter_result = baxter_client_.getResult();
  control_msgs::GripperCommandResult result;
  result.effort = baxter_result->effort;
  result.position = baxter_result->position;
  result.reached_goal = baxter_result->reached_goal;
  result.stalled = baxter_result->stalled;
  server_.setSucceeded(result);
}

void GripperAction::HandleFeedback(
    const BaxterGripperCommandFeedback::ConstPtr& baxter_feedback) {
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
  baxter_mechanism_msgs::SwitchControllerRequest req;
  req.strictness = baxter_mechanism_msgs::SwitchControllerRequest::BEST_EFFORT;
  baxter_mechanism_msgs::SwitchControllerResponse res;
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
  baxter_mechanism_msgs::SwitchControllerRequest req;
  req.strictness = baxter_mechanism_msgs::SwitchControllerRequest::BEST_EFFORT;
  baxter_mechanism_msgs::SwitchControllerResponse res;
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
  baxter_mechanism_msgs::ListControllersRequest req;
  baxter_mechanism_msgs::ListControllersResponse res;
  while (!list_client_.waitForExistence(ros::Duration(5))) {
    ROS_WARN("Waiting for baxter_controller_manager list service...");
  }
  bool success = list_client_.call(req, res);
  if (!success) {
    ROS_ERROR("baxter_controller_manager list service call failed.");
  }
  for (size_t i = 0; i < res.controllers.size(); ++i) {
    const std::string& name = res.controllers[i];
    const std::string& state = res.state[i];
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
}  // namespace baxter
}  // namespace pbd
}  // namespace rapid
