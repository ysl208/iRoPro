#ifndef _RAPID_PBD_PDDL_DOMAIN_H_
#define _RAPID_PBD_PDDL_DOMAIN_H_
#include <string>
#include <vector>

#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "rapid_pbd_msgs/Condition.h"
#include "rapid_pbd_msgs/Landmark.h"
#include "rapid_pbd_msgs/Program.h"
#include "rapid_pbd_msgs/Surface.h"

#include "rapid_pbd/joint_state.h"
#include "rapid_pbd/robot_config.h"

namespace rapid {
namespace pbd {
struct PDDLDomain {
 public:
  std::string scene_id;
  JointState joint_state;
  std::vector<rapid_pbd_msgs::Landmark> surface_box_landmarks;
  rapid_pbd_msgs::Surface surface;

  std::vector<rapid_pbd_msgs::Condition> domain_conditions;
  // std::vector<std::vector<std::string> > grid;
  std::vector<geometry_msgs::PoseArray> grid;
};

void GetPDDLDomain(const RobotConfig& robot_config,
               const rapid_pbd_msgs::Program& program, size_t step_id,
               PDDLDomain* domain);

}  // namespace pbd
}  // namespace rapid

#endif  // _RAPID_PBD_WORLD_H_
