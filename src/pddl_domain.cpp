#include "rapid_pbd/pddl_domain.h"

#include <math.h>
#include <set>
#include <string>
#include <vector>

#include "geometry_msgs/Pose.h"
#include "moveit_msgs/GetPositionIK.h"
#include "rapid_pbd_msgs/Action.h"
#include "rapid_pbd_msgs/Landmark.h"
#include "rapid_pbd_msgs/Program.h"
#include "rapid_pbd_msgs/Step.h"
#include "transform_graph/graph.h"

#include "rapid_pbd/action_utils.h"
#include "rapid_pbd/joint_state.h"
#include "rapid_pbd/landmarks.h"
#include "rapid_pbd/robot_config.h"

#include "ros/ros.h"

namespace msgs = rapid_pbd_msgs;

namespace rapid {
namespace pbd {
void InitDomain(Domain* domain) {}

void GetWorldState(const World& world, WorldState* world_state) {
  // Given: World with landmarks
  // Create World state of objects
  world_state->objects_.clear();
  world_state->predicates_.clear();

  for (size_t i = 0; i < world.surface_box_landmarks.size(); ++i) {
    const msgs::Landmark& world_landmark = world.surface_box_landmarks[i];

    msgs::PDDLObject obj;
    obj.name = "Object " + i;
    world_state->objects_.push_back(obj);
    // Generate Predicates for type == OBJECT
    // IS_ON predicates

    // IS_CLEAR predicates

    // Generate Predicates for type == POSITION
    // based on IS_ON predicates, we can infer IS_CLEAR
  }

  void GetTypeFromDims(const geometry_msgs::Vector3& dims, std::string type) {
    type = "";
  }

  void GetFixedPositions(std::vector<msgs::PDDLObject> * objects) {
    msgs::PDDLObject obj;
    msgs::PDDLType obj_type;
    obj_type.type = msgs::PDDLType::POSITION;
    obj_type.pose = obj.type = obj_type;

    // Positions A,B,C,D
    obj.name = "Position A";
    double a_center_x = 0, a_center_y = 0, a_center_z = 0;
    std::vector<std::pair<std::string, std::string>> world_positions;
    ros::param::param<std::vector<std::pair<std::string, std::string>>>(
        "world_positions", world_positions);
  }

}  // namespace pbd
}  // namespace pbd
