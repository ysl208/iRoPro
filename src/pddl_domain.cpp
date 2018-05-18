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

    // Generate Predicates for type == OBJECT
    msgs::PDDLObject obj;
    obj.name = "Object " + i;
    msgs::PDDLType obj_type;
    GetTypeFromDims(world_landmark.surface_box_dims, &obj_type);

    obj.type = obj_type;
    world_state->objects_.push_back(obj);

    // IS_ON predicates
    msgs::PDDLPredicate pred;
    pred.predicate = msgs::PDDLPredicate::IS_ON;
    for (size_t j = 0; j < world_state->positions_.size(); ++j) {
      // Check which position the object is on
      const msgs::PDDLObject& position = world_state->positions_[j];
      GetObjectTablePosition(position->type.pose)
    }

    world_state->predicates_.push_back(pred);

    // IS_CLEAR predicates

    // Generate Predicates for type == POSITION
    // based on IS_ON predicates, we can infer IS_CLEAR
  }
}
bool GetObjectTablePosition(const geometry_msgs::Pose& obj_pose,
                            const msgs::PDDLObject& position) {
  // Check if object center is inside the position area
  const double squared_distance_cutoff = 0.05;
  bool success = false;
  double closest_distance = std::numeric_limits<double>::max();

  for (size_t i = 0; i < world.surface_box_landmarks.size(); ++i) {
    const msgs::Landmark& world_landmark = world.surface_box_landmarks[i];
    if (landmark.name != world_landmark.name) {
      geometry_msgs::Point world_pose;
      world_pose.x = world_landmark.pose_stamped.pose.position.x;
      world_pose.y = world_landmark.pose_stamped.pose.position.y;
      world_pose.z = world_landmark.pose_stamped.pose.position.z;
      double dx = world_pose.x - landmark.pose_stamped.pose.position.x;
      double dy = world_pose.y - landmark.pose_stamped.pose.position.y;
      double dz = world_pose.z - landmark.pose_stamped.pose.position.z;
      double squared_distance = dx * dx + dy * dy + dz * dz;
      if (squared_distance < closest_distance &&
          squared_distance <= squared_distance_cutoff) {
        *reference = world_landmark;
        closest_distance = squared_distance;
        success = true;
      }
    }
  }
  return success;
}

void GetTypeFromDims(const geometry_msgs::Vector3& dims,
                     msgs::PDDLType* obj_type) {
  // TO DO: check yaml file to decide object type
  obj_type.type = msgs::PDDLType::OBJECT;
}

void GetFixedPositions(std::vector<msgs::PDDLObject>* objects) {
  msgs::PDDLObject obj;
  msgs::PDDLType obj_type;
  obj_type.type = msgs::PDDLType::POSITION;
  //   obj_type.pose =
  obj.type = obj_type;

  // Positions A,B,C,D
  obj.name = "Position A";
  double a_center_x = 0, a_center_y = 0, a_center_z = 0;
  // std::vector<std::pair<std::string, std::string> > world_positions;
  //   ros::param::param<std::vector<std::pair<std::string, std::string> > >(
  //       "world_positions", world_positions);
}

}  // namespace pbd
}  // namespace rapid
