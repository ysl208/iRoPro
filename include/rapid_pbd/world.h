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

namespace rapid {
namespace pbd {
struct World {
 public:
  std::string scene_id;
  JointState joint_state;
  std::vector<rapid_pbd_msgs::Landmark> surface_box_landmarks;
  rapid_pbd_msgs::Surface surface;

  std::vector<rapid_pbd_msgs::Condition> world_conditions;
  std::vector<std::vector<std::string> > grid;
  std::vector<std::vector<geometry_msgs::Point> > points;
};

void GetWorld(const RobotConfig& robot_config,
              const rapid_pbd_msgs::Program& program, size_t step_id,
              World* world);

bool MatchLandmark(const World& world, const rapid_pbd_msgs::Landmark& landmark,
                   rapid_pbd_msgs::Landmark* match, const double& variance);

void GetRPY(const geometry_msgs::Quaternion& q, geometry_msgs::Vector3* rpy);
void PointToVector3(const geometry_msgs::Point& p, geometry_msgs::Vector3* v);

void UpdateGrid(const rapid_pbd_msgs::Landmark& landmark,
                std::vector<std::vector<std::string> >* grid);

}  // namespace pbd
}  // namespace rapid

#endif  // _RAPID_PBD_WORLD_H_
