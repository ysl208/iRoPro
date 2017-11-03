#include "rapid_pbd/motion_planning.h"

#include <map>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include "geometry_msgs/Pose.h"
#include "moveit_goal_builder/builder.h"
#include "moveit_msgs/GetPositionIK.h"
#include "moveit_msgs/MoveGroupAction.h"
#include "rapid_pbd_msgs/Action.h"
#include "rapid_pbd_msgs/Landmark.h"
#include "ros/ros.h"
#include "transform_graph/graph.h"

#include "rapid_pbd/errors.h"
#include "rapid_pbd/landmarks.h"
#include "rapid_pbd/runtime_robot_state.h"
#include "rapid_pbd/world.h"

using moveit_msgs::MoveItErrorCodes;
using std::string;
namespace msgs = rapid_pbd_msgs;
namespace tg = transform_graph;

namespace rapid {
namespace pbd {
MotionPlanning::MotionPlanning(const RuntimeRobotState& robot_state,
                               World* world,
                               const ros::Publisher& planning_scene_pub)
    : robot_state_(robot_state),
      world_(world),
      planning_scene_pub_(planning_scene_pub),
      builder_(robot_state.config.planning_frame(),
               robot_state.config.planning_group()),
      num_goals_(0) {
  builder_.can_replan = true;
  builder_.num_planning_attempts = 10;
  builder_.replan_attempts = 2;
  builder_.planning_time = 10.0;
}

string MotionPlanning::AddPoseGoal(
    const string& actuator_group, const geometry_msgs::Pose& pose,
    const rapid_pbd_msgs::Landmark& landmark,
    const std::vector<std::string>& seed_joint_names,
    const std::vector<double>& seed_joint_positions) {
  const string& base_link(robot_state_.config.base_link());
  const string& ee_link(robot_state_.config.ee_frame_for_group(actuator_group));
  if (ee_link == "") {
    ROS_ERROR("Unable to look up EE link for actuator group \"%s\"",
              actuator_group.c_str());
    return "Invalid actuator in the program.";
  }

  tg::Graph graph;
  graph.Add("ee", tg::RefFrame("landmark"), pose);
  if (landmark.type == msgs::Landmark::TF_FRAME) {
    tf::StampedTransform st;
    try {
      robot_state_.tf_listener.lookupTransform(base_link, landmark.name,
                                               ros::Time(0), st);
    } catch (const tf::TransformException& ex) {
      ROS_ERROR(
          "Unable to get TF transform from \"%s\" to landmark frame \"%s\"",
          base_link.c_str(), landmark.name.c_str());
      return "Unable to get TF transform";
    }
    graph.Add("landmark", tg::RefFrame(base_link), st);
  } else if (landmark.type == msgs::Landmark::SURFACE_BOX) {
    msgs::Landmark match;
    bool success = MatchLandmark(*world_, landmark, &match);
    if (!success) {
      return errors::kNoLandmarksMatch;
    }
    if (match.pose_stamped.header.frame_id != base_link) {
      ROS_ERROR("Landmark not in base frame: \"%s\"",
                match.pose_stamped.header.frame_id.c_str());
      return "Landmark not in base frame.";
    }
    graph.Add("landmark", tg::RefFrame(match.pose_stamped.header.frame_id),
              match.pose_stamped.pose);
  } else {
    ROS_ERROR("Unsupported landmark type \"%s\"", landmark.type.c_str());
    return "Unsupported landmark type.";
  }

  // Transform pose into base frame
  tg::Transform landmark_transform;
  graph.ComputeDescription(tg::LocalFrame("ee"), tg::RefFrame(base_link),
                           &landmark_transform);
  geometry_msgs::Pose pose_in_base;
  landmark_transform.ToPose(&pose_in_base);

  // Compute IK
  // TODO: This is copied from world.cpp. Consider creating a common IK
  // function.
  moveit_msgs::GetPositionIKRequest ik_req;
  if (seed_joint_names.size() > 0) {
    ik_req.ik_request.robot_state.joint_state.name = seed_joint_names;
    ik_req.ik_request.robot_state.joint_state.position = seed_joint_positions;
  }
  if (actuator_group == msgs::Action::ARM) {
    ik_req.ik_request.group_name = "arm";
  } else if (actuator_group == msgs::Action::LEFT_ARM) {
    ik_req.ik_request.group_name = "left_arm";
  } else if (actuator_group == msgs::Action::RIGHT_ARM) {
    ik_req.ik_request.group_name = "right_arm";
  }
  ik_req.ik_request.pose_stamped.header.frame_id = base_link;
  ik_req.ik_request.pose_stamped.pose = pose_in_base;
  ik_req.ik_request.avoid_collisions = true;
  ik_req.ik_request.attempts = 5;
  ik_req.ik_request.timeout = ros::Duration(2);
  robot_state_.config.joints_for_group(actuator_group,
                                       &ik_req.ik_request.ik_link_names);
  moveit_msgs::GetPositionIKResponse ik_res;
  ros::service::call("/compute_ik", ik_req, ik_res);
  bool success =
      ik_res.error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS;
  if (!success) {
    std::string error("No IK solution found");
    ROS_ERROR("%s", error.c_str());
    return error;
  }

  // GetPositionIK returns solution for all joints. Filter out to just joints
  // relevant for this actuator.
  std::set<std::string> joint_set;
  joint_set.insert(ik_req.ik_request.ik_link_names.begin(),
                   ik_req.ik_request.ik_link_names.end());

  std::vector<std::string> joint_names;
  std::vector<double> joint_positions;
  for (size_t j = 0; j < ik_res.solution.joint_state.name.size(); ++j) {
    const std::string& name = ik_res.solution.joint_state.name[j];
    const double value = ik_res.solution.joint_state.position[j];
    if (joint_set.find(name) != joint_set.end()) {
      joint_names.push_back(name);
      joint_positions.push_back(value);
    }
  }

  return AddJointGoal(joint_names, joint_positions);
}

std::string MotionPlanning::AddJointGoal(
    const std::vector<std::string>& joint_names,
    const std::vector<double>& joint_positions) {
  if (joint_names.size() != joint_positions.size()) {
    std::string error("Joint names do not match joint positions!");
    ROS_ERROR_STREAM(error);
    return error;
  }

  for (size_t i = 0; i < joint_names.size(); ++i) {
    current_joint_goal_[joint_names[i]] = joint_positions[i];
  }
  builder_.SetJointGoal(current_joint_goal_);

  ++num_goals_;
  return "";
}

void MotionPlanning::ClearGoals() {
  std::map<string, geometry_msgs::Pose> goals;
  // Overrides joint goals if any and deletes pose goals.
  builder_.SetPoseGoals(goals);
  num_goals_ = 0;

  // If two-arm robot, then save the current joint angles for both arms
  // This is because if you leave the joint goal for an arm unspecified, it can
  // be random.
  int num_arms = robot_state_.config.num_arms();
  if (num_arms == 2) {
    std::vector<std::string> joints;
    robot_state_.config.joints_for_group(msgs::Action::LEFT_ARM, &joints);
    robot_state_.config.joints_for_group(msgs::Action::RIGHT_ARM, &joints);
    std::vector<double> joint_values;
    robot_state_.js_reader.get_positions(joints, &joint_values);
    current_joint_goal_.clear();
    for (size_t i = 0; i < joints.size(); ++i) {
      current_joint_goal_[joints[i]] = joint_values[i];
    }
  }
}

void MotionPlanning::BuildGoal(moveit_msgs::MoveGroupGoal* goal) const {
  builder_.Build(goal);
}

int MotionPlanning::num_goals() const { return num_goals_; }

std::string ErrorCodeToString(const MoveItErrorCodes& code) {
  if (code.val == MoveItErrorCodes::SUCCESS) {
    return "SUCCESS";
  }
  if (code.val == MoveItErrorCodes::FAILURE) {
    return "FAILURE";
  }
  if (code.val == MoveItErrorCodes::PLANNING_FAILED) {
    return "PLANNING_FAILED";
  }
  if (code.val == MoveItErrorCodes::INVALID_MOTION_PLAN) {
    return "INVALID_MOTION_PLAN";
  }
  if (code.val ==
      MoveItErrorCodes::MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE) {
    return "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE";
  }
  if (code.val == MoveItErrorCodes::CONTROL_FAILED) {
    return "CONTROL_FAILED";
  }
  if (code.val == MoveItErrorCodes::UNABLE_TO_AQUIRE_SENSOR_DATA) {
    return "UNABLE_TO_AQUIRE_SENSOR_DATA";
  }
  if (code.val == MoveItErrorCodes::TIMED_OUT) {
    return "TIMED_OUT";
  }
  if (code.val == MoveItErrorCodes::PREEMPTED) {
    return "PREEMPTED";
  }
  if (code.val == MoveItErrorCodes::START_STATE_IN_COLLISION) {
    return "START_STATE_IN_COLLISION";
  }
  if (code.val == MoveItErrorCodes::START_STATE_VIOLATES_PATH_CONSTRAINTS) {
    return "START_STATE_VIOLATES_PATH_CONSTRAINTS";
  }
  if (code.val == MoveItErrorCodes::GOAL_IN_COLLISION) {
    return "GOAL_IN_COLLISION";
  }
  if (code.val == MoveItErrorCodes::GOAL_VIOLATES_PATH_CONSTRAINTS) {
    return "GOAL_VIOLATES_PATH_CONSTRAINTS";
  }
  if (code.val == MoveItErrorCodes::GOAL_CONSTRAINTS_VIOLATED) {
    return "GOAL_CONSTRAINTS_VIOLATED";
  }
  if (code.val == MoveItErrorCodes::INVALID_GROUP_NAME) {
    return "INVALID_GROUP_NAME";
  }
  if (code.val == MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS) {
    return "INVALID_GOAL_CONSTRAINTS";
  }
  if (code.val == MoveItErrorCodes::INVALID_ROBOT_STATE) {
    return "INVALID_ROBOT_STATE";
  }
  if (code.val == MoveItErrorCodes::INVALID_LINK_NAME) {
    return "INVALID_LINK_NAME";
  }
  if (code.val == MoveItErrorCodes::INVALID_OBJECT_NAME) {
    return "INVALID_OBJECT_NAME";
  }
  if (code.val == MoveItErrorCodes::FRAME_TRANSFORM_FAILURE) {
    return "FRAME_TRANSFORM_FAILURE";
  }
  if (code.val == MoveItErrorCodes::COLLISION_CHECKING_UNAVAILABLE) {
    return "COLLISION_CHECKING_UNAVAILABLE";
  }
  if (code.val == MoveItErrorCodes::ROBOT_STATE_STALE) {
    return "ROBOT_STATE_STALE";
  }
  if (code.val == MoveItErrorCodes::SENSOR_INFO_STALE) {
    return "SENSOR_INFO_STALE";
  }
  if (code.val == MoveItErrorCodes::NO_IK_SOLUTION) {
    return "NO_IK_SOLUTION";
  }
  std::stringstream ss;
  ss << "Unknown error code " << code.val;
  return ss.str();
}

void MotionPlanning::PublishCollisionObject(const moveit_msgs::CollisionObject& obj) {
  moveit_msgs::PlanningScene scene;
  scene.world.collision_objects.push_back(obj);
  scene.is_diff = true;
  planning_scene_pub_.publish(scene);
}
}  // namespace pbd
}  // namespace rapid
