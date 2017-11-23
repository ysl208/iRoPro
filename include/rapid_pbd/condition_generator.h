//
//  condition.h
//
//
//  Created by YSL on 28/07/2017.
//
//

#ifndef _RAPID_PBD_CONDITION_GENERATOR_H_
#define _RAPID_PBD_CONDITION_GENERATOR_H_
#include <string>
#include <vector>

#include "rapid_pbd_msgs/Condition.h"
#include "rapid_pbd_msgs/Landmark.h"

#include "rapid_pbd/world.h"

namespace rapid {
namespace pbd {
// ConditionGenerator takes landmarks and infers conditions that can be used for
// pre-/postconditions
class ConditionGenerator {
 public:
  ConditionGenerator(const RobotConfig& robot_config);

  void AssignConditions(World* world);
  void AssignLandmarkCondition(const World& world,
                               const std::string& landmark_name,
                               rapid_pbd_msgs::Condition* condition);
  void UpdateReferenceLandmark(const World& world,
                               rapid_pbd_msgs::Condition* condition,
                               const rapid_pbd_msgs::Landmark& reference);
  void GenerateGrid(rapid_pbd_msgs::Condition* condition,
                    std::vector<geometry_msgs::PoseArray>* grid,
                    const int& obj_num);
  void UpdatePosteriors(const World& world,
                        const rapid_pbd_msgs::Landmark& landmark, bool flag1D,
                        std::vector<float>* posteriors);

 private:
  World* world_;
  const RobotConfig& robot_config_;

  bool IsLandmarkName(const rapid_pbd_msgs::Landmark& landmark,
                      const std::string& name);
  void GetPropertyConditions(const rapid_pbd_msgs::Landmark& landmark,
                             const World& world,
                             rapid_pbd_msgs::Condition* condition,
                             const float& defaultVariance);
  void GetRelativeConditions(const rapid_pbd_msgs::Landmark& landmark,
                             const World& world,
                             rapid_pbd_msgs::Condition* condition,
                             const float& defaultVariance);
  geometry_msgs::Vector3 QuaternionToRPY(const geometry_msgs::Quaternion& msg);
  void SetReferenceConditions(rapid_pbd_msgs::Condition* condition,
                              const rapid_pbd_msgs::Landmark& reference,
                              const rapid_pbd_msgs::Landmark& landmark,
                              const geometry_msgs::Vector3& defaultVarVector);
  bool ReferencedLandmark(const rapid_pbd_msgs::Landmark& landmark,
                          const World& world,
                          const double squared_distance_cutoff,
                          rapid_pbd_msgs::Landmark* reference);
  void GetDisplacementVector(const rapid_pbd_msgs::Landmark& landmark,
                             const rapid_pbd_msgs::Landmark& reference,
                             rapid_pbd_msgs::Condition* condition);
  void GetSpatialRelation(rapid_pbd_msgs::Condition* condition);
  void GetRelativeAlignment(rapid_pbd_msgs::Condition* condition);
  void CheckRelevantProperties(const World& world,
                               rapid_pbd_msgs::Condition* condition);
  void GetPositions(const geometry_msgs::Vector3& min_pos,
                    const geometry_msgs::Vector3& max_pos,
                    const geometry_msgs::Vector3& dimensions,
                    const geometry_msgs::Vector3& obj_distance,
                    std::vector<geometry_msgs::Pose>* positions,
                    const int& obj_num);
  void GetPatternPositions(const int& s,
                           std::vector<geometry_msgs::Pose>* positions,
                           const rapid_pbd_msgs::Landmark& landmark,
                           geometry_msgs::Vector3 obj_distance,
                           const int& obj_num);
  int GetPatternIndex(const std::string& s);
  void UpdatePriors(const std::vector<float>& priors,
                    const std::vector<float>& pOfD,
                    std::vector<float>* posteriors);
  // void GetPositionsAroundObject(
  //     msgs::Condition* condition, const geometry_msgs::Vector3& dimensions,
  //     const geometry_msgs::Vector3& obj_distance,
  //     std::vector<geometry_msgs::Pose>* positions);
};
}  // namespace pbd
}  // namespace rapid

#endif  // _RAPID_PBD_CONDITION_GENERATOR_H_
