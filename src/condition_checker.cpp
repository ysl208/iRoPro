#include "rapid_pbd/condition_checker.h"

#include <map>
#include <set>
#include <sstream>
#include <string>
#include <cmath>

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3.h"
#include "moveit_msgs/GetPositionIK.h"
#include "moveit_msgs/MoveGroupAction.h"
#include "rapid_pbd_msgs/Landmark.h"
#include "rapid_pbd_msgs/Condition.h"
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
ConditionChecker::ConditionChecker(World* world)
    : world_(world),
      //conditions_(conditions),
      num_goals_(0) {}


bool ConditionChecker::CheckConditions(const msgs::Condition& condition) {
    // Checks if the given condition holds in a world
    ROS_INFO("Landmark %s", condition.landmark.name.c_str());
    // Find matched landmark
    msgs::Landmark match;
    double sizeVariance = 0.075;
    if(condition.sizeRelevant) {
       sizeVariance = condition.sizeVariance.x;
    }

    bool visible = MatchLandmark(*world_, condition.landmark, &match, sizeVariance);
    // If there is a landmark that has similar dimensions, check its conditions
    if(visible) {
      ROS_INFO("Landmark found in new configuration: %s", match.name.c_str());
      if (!CheckPropertyConditions(condition, match)) {
        return false;
      } 
      if(condition.referenceRelevant && !CheckRelativeConditions(condition)) {
        return false;
      }
    }
    ROS_INFO("No landmark visible for %s", condition.landmark.name.c_str());
    return false;
  
  return true;
}

int ConditionChecker::num_goals() const { return num_goals_; }

bool ConditionChecker::PointDissimilarity(const float& value, const float& match,
                       const float& variance){
  return fabs(match-value) < variance;
}

bool ConditionChecker::VectorDissimilarity(const geometry_msgs::Vector3& actual,
                                          const geometry_msgs::Vector3& match,
                                          const geometry_msgs::Vector3& variance){
  if(!PointDissimilarity(actual.x, match.x, variance.x)) {
    ROS_INFO("Absolute x doesn't meet condition");
    return false;
  }
  if(!PointDissimilarity(actual.y, match.y, variance.y)) {
    ROS_INFO("Absolute y doesn't meet condition");
    return false;
  }
  if(!PointDissimilarity(actual.x, match.y, variance.z)) {
    ROS_INFO("Absolute z doesn't meet condition");
    return false;
  }
}

bool ConditionChecker::CheckPropertyConditions(const msgs::Condition& condition,
                  const msgs::Landmark& match){
  bool posMatched = true;
  if(condition.positionRelevant){
    geometry_msgs::Vector3 actualPos;
    PointToVector3(condition.position, &actualPos);
    geometry_msgs::Vector3 matchPos;
    PointToVector3(match.pose_stamped.pose.position, &matchPos);
    geometry_msgs::Vector3 variancePos = condition.positionVariance;
    posMatched = VectorDissimilarity(
                  actualPos, 
                  matchPos,
                  variancePos);
  }
  ROS_INFO("All position condition: passed");
  bool oriMatched = true;
  if(condition.orientationRelevant){

    geometry_msgs::Vector3 actualOri;
    GetRPY(condition.orientation, &actualOri);
    geometry_msgs::Vector3 matchOri;
    GetRPY(match.pose_stamped.pose.orientation, &matchOri);
    geometry_msgs::Vector3 varianceOri = condition.orientationVariance;
    oriMatched = VectorDissimilarity(
                  actualOri, 
                  matchOri,
                  varianceOri);
  }
  ROS_INFO("All orientation condition: passed");

      
  ROS_INFO("All absolute conditions: passed");
  return posMatched && oriMatched;
}

bool ConditionChecker::CheckRelativeConditions(const msgs::Condition& condition){
    bool disMatched = true;
  
  if(condition.reference.name != ""){
    msgs::Landmark match;
    double variance = 0.075;
    bool visible = MatchLandmark(*world_, condition.reference, &match, variance);
  
    if(visible){
      ROS_INFO("Reference found %s", match.name.c_str());

      geometry_msgs::Point landmark_pose = condition.landmark.pose_stamped.pose.position;
      geometry_msgs::Point reference_pose = match.pose_stamped.pose.position;
      
      geometry_msgs::Vector3 actualDis = condition.contDisplacement;
      geometry_msgs::Vector3 matchDis;
      geometry_msgs::Vector3 varianceDis = condition.contDisplacementVariance;
      matchDis.x = reference_pose.x - landmark_pose.x;
      matchDis.y = reference_pose.y - landmark_pose.y;
      matchDis.z = reference_pose.z - landmark_pose.z;

      if(condition.contDisplacementRelevant){
        disMatched = VectorDissimilarity(actualDis, 
                            matchDis,
                            varianceDis);
        ROS_INFO("Relative displacement vector doesn't meet condition");
      }
      if(condition.contOrientationRelevant){
        // TO DO
      }
      if(condition.discDisplacementRelevant){
        // TO DO
      }
    } else {
      ROS_INFO("No matching landmark found for reference %s",
        condition.reference.name.c_str());
      return false;
    }
    // Check orientationRelevant
  }
  ROS_INFO("No reference specified");
  return disMatched;
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
