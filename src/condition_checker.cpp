#include "rapid_pbd/condition_checker.h"

#include <cmath>
#include <map>
#include <set>
#include <sstream>
#include <string>

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3.h"
#include "moveit_msgs/GetPositionIK.h"
#include "moveit_msgs/MoveGroupAction.h"
#include "rapid_pbd_msgs/Condition.h"
#include "rapid_pbd_msgs/Landmark.h"
#include "ros/ros.h"
#include "transform_graph/graph.h"

#include "rapid_pbd/errors.h"
#include "rapid_pbd/world.h"

using moveit_msgs::MoveItErrorCodes;
using std::string;
namespace msgs = rapid_pbd_msgs;
namespace tg = transform_graph;

namespace rapid {
namespace pbd {
ConditionChecker::ConditionChecker(World* world,
                                   const ros::Publisher& condition_check_pub)
    : world_(world),
      // conditions_(conditions),
      num_goals_(0),
      condition_check_pub_(condition_check_pub) {}

std::string ConditionChecker::CheckConditions(
    const msgs::Condition& condition) {
  // Checks if the given condition holds in a world
  ROS_INFO("CheckConditions: Landmark %s", condition.landmark.name.c_str());

  info_.passed = false;
  // Find matched landmark
  msgs::Landmark match;
  double sizeVariance = 0.075;
  if (condition.sizeRelevant) {
    sizeVariance = condition.sizeVariance.x;
  }

  info_.mainFound =
      MatchLandmark(*world_, condition.landmark, &match, sizeVariance);
  // If there is a landmark that has similar dimensions, check its conditions
  if (info_.mainFound) {
    ROS_INFO("Landmark found in new configuration: %s", match.name.c_str());
    std::string propertyCheck = CheckPropertyConditions(condition, match);
    if (propertyCheck != "") {
      return propertyCheck;
    }
    std::string relativeCheck = CheckRelativeConditions(condition);
    if (relativeCheck != "") {
      return relativeCheck;
    }

  } else {
    ROS_ERROR("No landmark visible for %s", condition.landmark.name.c_str());
    return "No landmark visible for " + condition.landmark.name;
  }
  info_.passed = true;
  ROS_INFO("All conditions passed!");
  return "";
}

msgs::ConditionCheckInfo ConditionChecker::GetConditionCheckMsg() {
  return info_;
}

int ConditionChecker::num_goals() const { return num_goals_; }

bool ConditionChecker::PointDissimilarity(const float& value,
                                          const float& match,
                                          const float& variance) {
  ROS_INFO("Checking fabs(%f-%f) = %f < %f ", match, value,
           fabs(match) - fabs(value), variance);
  return (fabs(match) - fabs(value)) < variance;
}

bool ConditionChecker::VectorDissimilarity(
    const geometry_msgs::Vector3& actual, const geometry_msgs::Vector3& match,
    const geometry_msgs::Vector3& variance) {
  if (!PointDissimilarity(actual.x, match.x, variance.x)) {
    ROS_ERROR("x doesn't meet condition: is %f but should be %f +/- %f ",
              actual.x, match.x, variance.x);
    return false;
  }
  if (!PointDissimilarity(actual.y, match.y, variance.y)) {
    ROS_ERROR("y doesn't meet condition: is %f but should be %f +/- %f ",
              actual.y, match.y, variance.y);
    return false;
  }
  if (!PointDissimilarity(actual.z, match.z, variance.z)) {
    ROS_ERROR("z doesn't meet condition: is %f but should be %f +/- %f ",
              actual.z, match.z, variance.z);
    return false;
  }
  return true;
}

std::string ConditionChecker::CheckPropertyConditions(
    const msgs::Condition& condition, const msgs::Landmark& match) {
  info_.posMatched = true;
  if (condition.positionRelevant) {
    geometry_msgs::Vector3 actualPos;
    PointToVector3(condition.position, &actualPos);
    geometry_msgs::Vector3 matchPos;
    PointToVector3(match.pose_stamped.pose.position, &matchPos);
    geometry_msgs::Vector3 variancePos = condition.positionVariance;
    info_.posMatched = VectorDissimilarity(actualPos, matchPos, variancePos);
  }
  if (!info_.posMatched) {
    ROS_ERROR("Absolute position of %s doesn't match condition",
              match.name.c_str());

    return "Absolute position doesn't match condition: " + match.name;
  }

  info_.oriMatched = true;
  if (condition.orientationRelevant) {
    geometry_msgs::Vector3 actualOri;
    // GetRPY(condition.orientation, &actualOri);
    actualOri = condition.eulerAngles;

    geometry_msgs::Vector3 matchOri;
    GetRPY(match.pose_stamped.pose.orientation, &matchOri);
    geometry_msgs::Vector3 varianceOri = condition.orientationVariance;
    info_.oriMatched = VectorDissimilarity(actualOri, matchOri, varianceOri);
  }
  if (!info_.oriMatched) {
    ROS_ERROR("Absolute orientation of %s doesn't match condition",
              match.name.c_str());

    return "Absolute orientation doesn't match condition: " + match.name;
  }
  ROS_INFO("All absolute conditions: passed");
  return "";
}

std::string ConditionChecker::CheckRelativeConditions(
    const msgs::Condition& condition) {
  info_.referenceFound = true;

  if (condition.reference.name == "") {
    return "";
  } else {
    // condition->referenceRelevant = true;
    msgs::Landmark match;
    double variance = 0.075;
    info_.referenceFound =
        MatchLandmark(*world_, condition.reference, &match, variance);

    if (info_.referenceFound) {
      ROS_INFO("Reference found %s", match.name.c_str());

      info_.contDisMatched = true;
      if (condition.contDisplacementRelevant) {
        geometry_msgs::Point landmark_pose =
            condition.landmark.pose_stamped.pose.position;
        geometry_msgs::Point reference_pose = match.pose_stamped.pose.position;

        geometry_msgs::Vector3 actualDis = condition.contDisplacement;
        geometry_msgs::Vector3 matchDis;
        geometry_msgs::Vector3 varianceDis = condition.contDisplacementVariance;
        matchDis.x = reference_pose.x - landmark_pose.x;
        matchDis.y = reference_pose.y - landmark_pose.y;
        matchDis.z = reference_pose.z - landmark_pose.z;
        info_.contDisMatched =
            VectorDissimilarity(actualDis, matchDis, varianceDis);
      }

      if (!info_.contDisMatched) {
        ROS_ERROR("Relative displacement vector doesn't meet condition");
        return "Relative displacement vector doesn't meet condition";
      }

      info_.contOriMatched = true;
      if (condition.contOrientationRelevant) {
        // TO DO
      }

      if (condition.discDisplacementRelevant) {
        // TO DO
      }
    } else {
      ROS_ERROR("No reference landmark found that matches %s",
                condition.reference.name.c_str());
      return "No matching landmark found for reference";
    }
  }
  // TO DO: Check orientationRelevant

  return "";
}

void ConditionChecker::PublishConditionCheck() {
  ROS_INFO("Publish condition check results..");
  condition_check_pub_.publish(info_);
}

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
}  // namespace pbd
}  // namespace rapid
