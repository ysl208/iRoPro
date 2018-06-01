//  Created by YSL on 28/11/2017.
//
//

#ifndef _RAPID_PBD_SPEC_INFERENCE_H_
#define _RAPID_PBD_SPEC_INFERENCE_H_
#include <string>
#include <vector>

#include "rapid_pbd_msgs/Condition.h"
#include "rapid_pbd_msgs/Landmark.h"
#include "rapid_pbd_msgs/Specification.h"
#include "rapid_pbd_msgs/Surface.h"

#include "rapid_pbd/world.h"

namespace msgs = rapid_pbd_msgs;
namespace rapid {
namespace pbd {
// ConditionGenerator takes landmarks and infers conditions that can be used for
// pre-/postconditions
class SpecInference {
 public:
  std::vector<float> priors_, posteriors_;
  std::vector<msgs::Specification> specs_;
  SpecInference(const RobotConfig& robot_config);
  void Init();
  void InitSpec(msgs::Specification* spec);
  void InitSpecs(std::vector<msgs::Specification>* specs,
                 const msgs::Landmark& landmark);
  void UpdatePosteriors(const World& world, const msgs::Landmark& landmark,
                        std::vector<float>* posteriors,
                        msgs::Specification* spec);
  void GenerateGrid(const msgs::Specification& spec,
                    const msgs::Surface& surface,
                    std::vector<geometry_msgs::PoseArray>* grid);
  void GenerateStacks(const msgs::Specification& spec,
                      const msgs::Surface& surface,
                      std::vector<geometry_msgs::PoseArray>* grid);
  void GetOffset(const msgs::Specification& spec,
                 geometry_msgs::Vector3* offset);

 private:
  World* world_;
  const RobotConfig& robot_config_;
  double distance_cutoff;
  float allowedVariance;
  bool flag1D;
  float avg_dx, avg_dy;
  int max_rows_, min_rows_;

  bool ReferencedLandmark(const msgs::Landmark& landmark, const World& world,
                          const double squared_distance_cutoff,
                          msgs::Landmark* reference);

  void GetPatternPositions(const int& s,
                           std::vector<geometry_msgs::Pose>* positions,
                           const msgs::Landmark& landmark,
                           geometry_msgs::Vector3 obj_distance,
                           const int& obj_num);
  int GetPatternIndex(const std::string& s);
  void UpdatePriors(const std::vector<float>& pOfD,
                    const std::vector<float>& priors,
                    std::vector<float>* posteriors);
  void GetPositions(const geometry_msgs::Point& min_pos,
                    const geometry_msgs::Point& max_pos,
                    const geometry_msgs::Vector3& offset,
                    const geometry_msgs::Quaternion& orientation,
                    const geometry_msgs::Vector3& obj_distance,
                    std::vector<geometry_msgs::Pose>* positions,
                    const int& obj_num);

  bool SimilarSized(const msgs::Landmark& landmark1,
                    const msgs::Landmark& landmark2);
};
}  // namespace pbd
}  // namespace rapid

#endif  // _RAPID_PBD_SPEC_INFERENCE_H_
