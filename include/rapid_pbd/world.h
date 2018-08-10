#ifndef _RAPID_PBD_WORLD_H_
#define _RAPID_PBD_WORLD_H_
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

namespace msgs = rapid_pbd_msgs;
namespace rapid {
namespace pbd {
struct World {
 public:
  std::string scene_id;
  JointState joint_state;
  std::vector<msgs::Landmark> surface_box_landmarks;
  msgs::Surface surface;

  std::vector<msgs::Condition> world_conditions;
  // std::vector<std::vector<std::string> > grid;
  std::vector<geometry_msgs::PoseArray> grid;
  std::vector<msgs::WorldState> world_state;
};

void GetWorld(const RobotConfig& robot_config, const msgs::Program& program,
              size_t step_id, World* world);

bool MatchLandmark(const World& world, const msgs::Landmark& landmark,
                   msgs::Landmark* match, const double& variance);

void GetRPY(const geometry_msgs::Quaternion& q, geometry_msgs::Vector3* rpy);
void PointToVector3(const geometry_msgs::Point& p, geometry_msgs::Vector3* v);

void UpdateGrid(const msgs::Landmark& landmark,
                std::vector<geometry_msgs::PoseArray>* grid);

}  // namespace pbd
}  // namespace rapid

#endif  // _RAPID_PBD_WORLD_H_
