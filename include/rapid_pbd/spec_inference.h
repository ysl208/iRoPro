//  Created by YSL on 28/11/2017.
//
//

#ifndef _RAPID_PBD_SPEC_INFERENCE_H_
#define _RAPID_PBD_SPEC_INFERENCE_H_
#include <string>
#include <vector>

#include "rapid_pbd_msgs/Condition.h"
#include "rapid_pbd_msgs/Landmark.h"

#include "rapid_pbd/world.h"

namespace rapid {
namespace pbd {
// ConditionGenerator takes landmarks and infers conditions that can be used for
// pre-/postconditions
class SpecInference {
 public:
  SpecInference(const RobotConfig& robot_config);
  void Init();
  void UpdatePosteriors(const World& world,
                        const rapid_pbd_msgs::Landmark& landmark);

 private:
  World* world_;
  const RobotConfig& robot_config_;
   double distance_cutoff;
   float allowedVariance;
  std::vector<float> priors_, posteriors_;
  bool flag1D;
  float avg_dx, avg_dy;


  bool ReferencedLandmark(const rapid_pbd_msgs::Landmark& landmark,
                          const World& world,
                          const double squared_distance_cutoff,
                          rapid_pbd_msgs::Landmark* reference);

  void GetPatternPositions(const int& s,
                           std::vector<geometry_msgs::Pose>* positions,
                           const rapid_pbd_msgs::Landmark& landmark,
                           geometry_msgs::Vector3 obj_distance,
                           const int& obj_num);
  int GetPatternIndex(const std::string& s);
  void UpdatePriors(const std::vector<float>& pOfD);
};
}  // namespace pbd
}  // namespace rapid

#endif  // _RAPID_PBD_SPEC_INFERENCE_H_
