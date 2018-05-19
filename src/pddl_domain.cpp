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
    obj_type.type = msgs::PDDLType::OBJECT;
    GetTypeFromDims(world_landmark.surface_box_dims, &obj_type);

    obj.type = obj_type;
    world_state->objects_.push_back(obj);

    // IS_ON predicates
    // Check which position the object is on
    double position_radius = 0.05;
    std::vector<msgs::PDDLObject> args;
    std::string predicate;
    msgs::PDDLObject position;
    if (GetObjectTablePosition(obj_type, world_state, position_radius,
                               &position)) {
      predicate = msgs::PDDLPredicate::IS_ON;
      args.push_back(obj);
      args.push_back(position);
      AddPredicate(world_state->predicates_, predicate, args);

      predicate = msgs::PDDLPredicate::IS_STACKABLE;
      AddPredicate(world_state->predicates_, predicate, args);

      predicate = msgs::PDDLPredicate::IS_CLEAR;
      args.clear();
      args.push_back(obj);
      AddPredicate(world_state->predicates_, predicate, args);

    } else {
      ROS_ERROR("Object not on any predefined position");
    }

    // Generate Predicates for type == POSITION
    // based on IS_ON predicates, we can infer IS_CLEAR

    for (size_t i = 0; i < world_state->positions_.size(); ++i) {
      const msgs::PDDLObject& pos_object = world_state->positions_[i];
      predicate = msgs::PDDLPredicate::IS_CLEAR;
      args.clear();
      args.push_back(pos_object);
      AddPredicate(world_state->predicates_, predicate, args);
    }
  }
}
bool PredicateExists(std::vector<msgs::PDDLPredicate> predicates,
                     const std::string& predicate,
                     const std::vector<msgs::PDDLObject>& args) {
  for (size_t i = 0; i < predicates.size(); ++i) {
    const msgs::PDDLPredicate& pred = predicates[i];
    if (pred.name == predicate && pred.arg1.name == args[0].name) {
      if (args.size() > 1) {
        return pred.arg1.name == args[1].name;
      }
      return true;
    }
  }
  return false;
}
void AddPredicate(std::vector<msgs::PDDLPredicate> predicates,
                  const std::string& predicate,
                  const std::vector<msgs::PDDLObject>& args) {
  msgs::PDDLPredicate new_pred;
  new_pred.name = predicate;

  if (predicate == msgs::PDDLPredicate::IS_ON && args.size() == 2) {
    new_pred.arg1 = args[0];
    new_pred.arg2 = args[1];
  } else if (predicate == msgs::PDDLPredicate::IS_CLEAR && args.size() == 1) {
    new_pred.arg1 = args[0];
  } else if (predicate == msgs::PDDLPredicate::IS_STACKABLE &&
             args.size() == 2) {
    new_pred.arg1 = args[0];
    new_pred.arg2 = args[1];
  }
  if (!PredicateExists(predicates, predicate, args)) {
    predicates.push_back(new_pred);
  }
}
bool GetObjectTablePosition(const msgs::PDDLType& obj, WorldState* world_state,
                            const double squared_distance_cutoff,
                            msgs::PDDLObject* found_position) {
  // return closest position
  // TO DO: get squared_distance_cutoff from yaml file
  bool success = false;
  double closest_distance = std::numeric_limits<double>::max();
  if (obj.type == msgs::PDDLType::OBJECT) {
    for (size_t i = 0; i < world_state->positions_.size(); ++i) {
      const msgs::PDDLObject& pos_object = world_state->positions_[i];
      if (pos_object.type.type == msgs::PDDLType::POSITION) {
        geometry_msgs::Point pose;
        pose.x = pos_object.type.pose_stamped.pose.position.x;
        pose.y = pos_object.type.pose_stamped.pose.position.y;
        pose.z = pos_object.type.pose_stamped.pose.position.z;
        double dx = pose.x - obj.pose_stamped.pose.position.x;
        double dy = pose.y - obj.pose_stamped.pose.position.y;
        double dz = pose.z - obj.pose_stamped.pose.position.z;
        double squared_distance = dx * dx + dy * dy + dz * dz;
        if (squared_distance < closest_distance &&
            squared_distance <= squared_distance_cutoff) {
          *found_position = pos_object;
          closest_distance = squared_distance;
          success = true;
        }
      }
    }

    return success;
  } else {
    return false;
  }
}

void GetTypeFromDims(const geometry_msgs::Vector3& dims,
                     msgs::PDDLType* obj_type) {
  // TO DO: check yaml file to decide object type
  // obj_type->type = msgs::PDDLType::OBJECT;
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
