#ifndef _RAPID_PBD_CONDITION_CHECKER_H_
#define _RAPID_PBD_CONDITION_CHECKER_H_

#include <set>
#include <string>
#include <vector>

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3.h"
#include "moveit_msgs/MoveGroupAction.h"
#include "moveit_msgs/MoveItErrorCodes.h"
#include "rapid_pbd_msgs/Condition.h"
#include "rapid_pbd_msgs/ConditionCheckInfo.h"
#include "rapid_pbd_msgs/Landmark.h"
#include "ros/ros.h"
#include "tf/transform_listener.h"

#include "rapid_pbd/world.h"

namespace msgs = rapid_pbd_msgs;
namespace rapid {
namespace pbd {
class ConditionChecker {
 public:
  ConditionChecker(World* world, const ros::Publisher& condition_check_pub);
  // Returns an error message, or empty string if no error.
  // Set seed_joint_names and seed_joint_positions to empty vectors if you do
  // not want to specify an IK seed.
  std::string CheckConditions(const msgs::Condition& condition);

  int num_goals() const;

  std::string CheckPropertyConditions(const msgs::Condition& conditions,
                                      const msgs::Landmark& match);
  std::string CheckRelativeConditions(const msgs::Condition& conditions);
  bool PointDissimilarity(const float& value, const float& match,
                          const float& variance);
  msgs::ConditionCheckInfo GetConditionCheckMsg();
  bool VectorDissimilarity(const geometry_msgs::Vector3& actual,
                           const geometry_msgs::Vector3& match,
                           const geometry_msgs::Vector3& variance);
  void PublishConditionCheck();

 private:
  World* world_;
  int num_goals_;
  ros::Publisher condition_check_pub_;
  msgs::ConditionCheckInfo info_;
};

std::string ErrorCodeToString(const moveit_msgs::MoveItErrorCodes& code);
}  // namespace pbd
}  // namespace rapid

#endif  // _RAPID_PBD_CONDITION_CHECKER_H_
