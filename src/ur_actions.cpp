#include "rapid_pbd/ur_actions.h"
#include <string>

#include "actionlib/client/simple_action_client.h"
#include "control_msgs/GripperCommandAction.h"
#include "control_msgs/FollowJointTrajectoryAction.h"
#include "rapid_pbd_msgs/Action.h"
#include "rapid_pbd_msgs/ArmControllerState.h"
#include "trajectory_msgs/JointTrajectory.h"

// using robot_controllers_msgs::ControllerState;
// using robot_controllers_msgs::FollowJointTrajectoryGoal;
// using robot_controllers_msgs::FollowJointTrajectoryResult;

using actionlib::SimpleClientGoalState;
using control_msgs::GripperCommandFeedback;
using control_msgs::GripperCommandResult;
typedef actionlib::SimpleActionClient<
    control_msgs::FollowJointTrajectoryAction>
    ControllerClient;

namespace msgs = rapid_pbd_msgs;

namespace rapid {
namespace pbd {
namespace ur {

GripperAction::GripperAction(const std::string& action_name,
                             const std::string& ur_action_name)
    : server_(action_name, boost::bind(&GripperAction::Execute, this, _1),
              false),
      ur_client_(ur_action_name, true) {}

void GripperAction::Start() {
  while (!ur_client_.waitForServer(ros::Duration(5))) {
    ROS_WARN("Waiting for UR gripper server to come up.");
  }
  server_.start();
}

void GripperAction::Execute(
    const control_msgs::GripperCommandGoalConstPtr& goal) {
  control_msgs::GripperCommandGoal ur_goal;
  // Value betwewen 0.0 (open) and 0.8 (closed)
  // TO DO: need to verify that the passed number is correct
  ur_goal.command.position = 0.8-goal->command.position;
  ur_goal.command.max_effort = goal->command.max_effort;
  ROS_INFO("Gripper action forwarding to Gripper client");
  ur_client_.sendGoal(
      ur_goal,
      boost::function<void(const SimpleClientGoalState&,
                           const GripperCommandResult::ConstPtr&)>(),
      boost::function<void()>(),
      boost::bind(&GripperAction::HandleFeedback, this, _1));
  while (!ur_client_.getState().isDone()) {
    if (server_.isPreemptRequested() || !ros::ok()) {
      ur_client_.cancelAllGoals();
      server_.setPreempted();
      return;
    }
    ros::spinOnce();
  }
  if (ur_client_.getState() == SimpleClientGoalState::PREEMPTED) {
    ur_client_.cancelAllGoals();
    server_.setPreempted();
    return;
  } else if (ur_client_.getState() == SimpleClientGoalState::ABORTED) {
    ur_client_.cancelAllGoals();
    server_.setAborted();
    return;
  }

  GripperCommandResult::ConstPtr ur_result = ur_client_.getResult();
  control_msgs::GripperCommandResult result;
  result.effort = ur_result->effort;
  result.position = ur_result->position;
  result.reached_goal = ur_result->reached_goal;
  result.stalled = ur_result->stalled;
  server_.setSucceeded(*ur_result);
}

void GripperAction::HandleFeedback(
    const GripperCommandFeedback::ConstPtr& ur_feedback) {
  control_msgs::GripperCommandFeedback feedback;
  feedback.effort = ur_feedback->effort;
  feedback.position = ur_feedback->position;
  feedback.reached_goal = ur_feedback->reached_goal;
  feedback.stalled = ur_feedback->stalled;
  server_.publishFeedback(*ur_feedback);
}

// to do
// ArmControllerManager::ArmControllerManager(const ros::Publisher& state_pub,
//                                            ControllerClient* client)
//     : state_pub_(state_pub), client_(client), is_arm_active_(true) {}

// void ArmControllerManager::Start() { Update(); }

// bool ArmControllerManager::HandleFreeze(msgs::FreezeArmRequest& request,
//                                         msgs::FreezeArmResponse& response) {
//   FollowJointTrajectoryGoal goal;
//   ControllerState state;
//   state.name = "arm_controller/follow_joint_trajectory";
//   state.state = ControllerState::RUNNING;
//   goal.updates.push_back(state);

//   client_->sendGoal(goal);
//   if (!client_->waitForResult(ros::Duration(5.0))) {
//     response.error = "Failed to freeze arm.";
//     ROS_ERROR("%s", response.error.c_str());
//     return true;
//   }
//   FollowJointTrajectoryResult::ConstPtr result = client_->getResult();
//   if (!result) {
//     response.error = "Got null result when freezing arm.";
//     ROS_ERROR("%s", response.error.c_str());
//     return true;
//   }
//   Update();
//   return true;
// }

// bool ArmControllerManager::HandleRelax(msgs::RelaxArmRequest& request,
//                                        msgs::RelaxArmResponse& response) {
//   FollowJointTrajectoryGoal goal;
//   ControllerState state;
//   state.name = "arm_controller/follow_joint_trajectory";
//   state.state = ControllerState::STOPPED;
//   goal.updates.push_back(state);

//   client_->sendGoal(goal);
//   if (!client_->waitForResult(ros::Duration(5.0))) {
//     response.error = "Failed to relax arm.";
//     ROS_ERROR("%s", response.error.c_str());
//     return true;
//   }
//   FollowJointTrajectoryResult::ConstPtr result = client_->getResult();
//   if (!result) {
//     response.error = "Got null result when relaxing arm.";
//     ROS_ERROR("%s", response.error.c_str());
//     return true;
//   }
//   Update();
//   return true;
// }

// void ArmControllerManager::Update() {
//   FollowJointTrajectoryGoal goal;
//   client_->sendGoal(goal);
//   if (!client_->waitForResult(ros::Duration(5.0))) {
//     ROS_ERROR("Failed to query arm controller state!");
//     return;
//   }
//   FollowJointTrajectoryResult::ConstPtr result = client_->getResult();
//   if (!result) {
//     ROS_ERROR("Got null result when querying arm controller state!");
//     return;
//   }

//   msgs::ArmControllerState msg;
//   for (size_t i = 0; i < result->state.size(); ++i) {
//     const ControllerState& state = result->state[i];
//     if (state.name == "arm_controller/follow_joint_trajectory") {
//       if (state.state == ControllerState::STOPPED) {
//         is_arm_active_ = false;
//         msg.arm_controller = msgs::ArmControllerState::RELAXED;
//         state_pub_.publish(msg);
//         return;
//       } else if (state.state == ControllerState::RUNNING) {
//         is_arm_active_ = true;
//         msg.arm_controller = msgs::ArmControllerState::FROZEN;
//         state_pub_.publish(msg);
//         return;
//       }
//     }
//   }
// }
}  // namespace ur
}  // namespace pbd
}  // namespace rapid
