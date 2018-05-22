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
  ROS_INFO("Getting World State with %zd landmarks...",
           world.surface_box_landmarks.size());

  world_state->objects_.clear();
  world_state->predicates_.clear();
  world_state->positions_.clear();
  std::vector<msgs::PDDLObject> args;
  std::string predicate;
  double position_radius = 0.9;
  msgs::PDDLObject position;
  bool negate = false;
  msgs::PDDLObject obj;
  msgs::PDDLType obj_type;

  // Add positions to world state objects
  std::vector<msgs::PDDLObject> fixed_positions;
  GetFixedPositions(&fixed_positions);
  world_state->objects_ = fixed_positions;
  world_state->positions_ = fixed_positions;

  for (size_t i = 0; i < world.surface_box_landmarks.size(); ++i) {
    const msgs::Landmark& world_landmark = world.surface_box_landmarks[i];
    ROS_INFO("Generating predicates for %s at (%f,%f,%f)",
             world_landmark.name.c_str(),
             world_landmark.pose_stamped.pose.position.x,
             world_landmark.pose_stamped.pose.position.y,
             world_landmark.pose_stamped.pose.position.z);
    // Generate Predicates for type == OBJECT
    std::stringstream ss;
    ss << "Obj " << i + 1;
    obj.name = ss.str();
    msgs::PDDLType obj_type;
    obj_type.name = msgs::PDDLType::OBJECT;
    GetTypeFromDims(world_landmark.surface_box_dims, &obj_type);
    obj.type = obj_type;
    AddObject(&world_state->objects_, obj);

    // IS_ON predicates
    // Check which position the object is on
    if (GetObjectTablePosition(obj_type, world_state, position_radius,
                               &position)) {
      negate = false;
      predicate = msgs::PDDLPredicate::IS_ON;
      args.clear();
      args.push_back(obj);
      args.push_back(position);
      AddPredicate(&world_state->predicates_, predicate, args, negate);

      predicate = msgs::PDDLPredicate::IS_STACKABLE;
      AddPredicate(&world_state->predicates_, predicate, args, negate);

      predicate = msgs::PDDLPredicate::IS_CLEAR;
      args.clear();
      args.push_back(obj);
      AddPredicate(&world_state->predicates_, predicate, args, negate);

      predicate = msgs::PDDLPredicate::IS_CLEAR;
      args.clear();
      args.push_back(position);
      negate = true;
      AddPredicate(&world_state->predicates_, predicate, args, negate);
      ROS_INFO("Predicates now: %zd", world_state->predicates_.size());
    } else {
      ROS_ERROR("Object not on any predefined position");
    }
  }
  // Generate Predicates for type == POSITION
  // based on IS_ON predicates, we can infer IS_CLEAR
  for (size_t i = 0; i < world_state->positions_.size(); ++i) {
    const msgs::PDDLObject& pos_object = world_state->positions_[i];
    predicate = msgs::PDDLPredicate::IS_CLEAR;
    args.clear();
    args.push_back(pos_object);
    negate = false;
    AddPredicate(&world_state->predicates_, predicate, args, negate);
  }
  ROS_INFO("Number of Objects: %zd", world_state->objects_.size());
  ROS_INFO("Number of Positions: %zd", world_state->positions_.size());
  ROS_INFO("Number of Predicates: %zd", world_state->predicates_.size());
}

void AddObject(std::vector<msgs::PDDLObject>* objects,
               const msgs::PDDLObject& new_obj) {
  if (!ObjectExists(objects, new_obj.name)) {
    objects->push_back(new_obj);
    ROS_INFO("Added object #%zd: %s", objects->size(), new_obj.name.c_str());
  } else {
    ROS_INFO("%s already exists", new_obj.name.c_str());
  }
}

bool ObjectExists(std::vector<msgs::PDDLObject>* objects,
                  const std::string& name) {
  for (size_t i = 0; i < objects->size(); ++i) {
    const msgs::PDDLObject& obj = objects->at(i);
    if (obj.name == name) {
      return true;
    }
  }
  return false;
}

void AddPredicate(std::vector<msgs::PDDLPredicate>* predicates,
                  const std::string& name,
                  const std::vector<msgs::PDDLObject>& args, bool negate) {
  msgs::PDDLPredicate new_pred;
  new_pred.name = name;
  new_pred.negate = negate;
  if (name == msgs::PDDLPredicate::IS_ON && args.size() == 2) {
    new_pred.arg1 = args[0];
    new_pred.arg2 = args[1];
  } else if (name == msgs::PDDLPredicate::IS_CLEAR && args.size() == 1) {
    new_pred.arg1 = args[0];
  } else if (name == msgs::PDDLPredicate::IS_STACKABLE && args.size() == 2) {
    new_pred.arg1 = args[0];
    new_pred.arg2 = args[1];
  } else {
    ROS_ERROR("Predicate %s is incorrect with %zd args: arg1 = %s",
              name.c_str(), args.size(), args[0].name.c_str());
  }
  if (!PredicateExists(predicates, name, args)) {
    predicates->push_back(new_pred);
    ROS_INFO("Added predicate #%zd: %s", predicates->size(), name.c_str());
  } else {
    ROS_INFO("Predicate %s(%s) already exists", name.c_str(),
             args[0].name.c_str());
  }
}

bool PredicateExists(std::vector<msgs::PDDLPredicate>* predicates,
                     const std::string& predicate,
                     const std::vector<msgs::PDDLObject>& args) {
  for (size_t i = 0; i < predicates->size(); ++i) {
    const msgs::PDDLPredicate& pred = predicates->at(i);
    if (pred.name == predicate && pred.arg1.name == args[0].name) {
      if (args.size() > 1) {
        return pred.arg1.name == args[1].name;
      }
      return true;
    }
  }
  return false;
}

bool GetObjectTablePosition(const msgs::PDDLType& obj, WorldState* world_state,
                            const double distance,
                            msgs::PDDLObject* found_position) {
  // return closest position
  // TO DO: get squared_distance_cutoff from yaml file
  double squ_dist_cutoff = distance * distance;
  bool success = false;
  double closest_distance = std::numeric_limits<double>::max();
  if (obj.name == msgs::PDDLType::OBJECT) {
    for (size_t i = 0; i < world_state->positions_.size(); ++i) {
      const msgs::PDDLObject& pos_object = world_state->positions_[i];
      if (pos_object.type.name == msgs::PDDLType::POSITION) {
        geometry_msgs::Point pose;
        pose.x = pos_object.type.pose.position.x;
        pose.y = pos_object.type.pose.position.y;
        pose.z = pos_object.type.pose.position.z;
        double dx = pose.x - obj.pose.position.x;
        double dy = pose.y - obj.pose.position.y;
        double dz = pose.z - obj.pose.position.z;
        double squared_distance = dx * dx + dy * dy + dz * dz;
        ROS_INFO("Dist = %f < cutoff %f", squared_distance, squ_dist_cutoff);

        if (squared_distance < closest_distance &&
            squared_distance <= squ_dist_cutoff) {
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
  obj_type.name = msgs::PDDLType::POSITION;
  // std::vector<double, double> poses = {(1.0, -0.5), (1.0, 0.5), (1.15, -0.5),
  //                                     (1.15, 0.5)};
  for (size_t i = 0; i < 4; ++i) {
    std::stringstream ss;
    ss << "Position " << i + 1;
    obj.name = ss.str();
    geometry_msgs::Pose pose;
    pose.position.x = 0.8 + 0.2 * (i % 2);
    pose.position.y = -0.25 + 0.5 * (i % 2);
    pose.position.z = -0.05;
    pose.orientation.w = 1;
    pose.orientation.x = 0;
    pose.orientation.y = 0;
    pose.orientation.z = 0;
    obj_type.name = msgs::PDDLType::POSITION;
    obj_type.pose = pose;
    obj.type = obj_type;
    ROS_INFO("%s at %f,%f,%f", obj.name.c_str(), pose.position.x,
             pose.position.y, pose.position.z);
    AddObject(objects, obj);
  }
  double a_center_x = 0, a_center_y = 0, a_center_z = 0;
  // std::vector<std::pair<std::string, std::string> > world_positions;
  //   ros::param::param<std::vector<std::pair<std::string, std::string> > >(
  //       "world_positions", world_positions);
}

void PrintAllPredicates(std::vector<msgs::PDDLPredicate> predicates,
                        std::string type) {
  if (type == "PDDL") {
    std::cout << "and ";
    for (size_t i = 0; i < predicates.size(); ++i) {
      msgs::PDDLPredicate predicate = predicates[i];
      std::cout << "(" + PrintPDDLPredicate(predicate) + ")" << std::endl;
    }
  } else {
    for (size_t i = 0; i < predicates.size(); ++i) {
      msgs::PDDLPredicate predicate = predicates[i];
      std::cout << PrintPredicate(predicate) << std::endl;
    }
  }
}

std::string PrintPDDLPredicate(msgs::PDDLPredicate predicate) {
  std::string print_out = "";
  print_out += predicate.name + " ";
  print_out += predicate.arg1.name;
  if (predicate.arg2.name != "") {
    print_out += " " + predicate.arg2.name;
  }
  if (predicate.negate) {
    print_out = "not(" + print_out + ")";
  }
  return print_out;
}

std::string PrintPredicate(msgs::PDDLPredicate predicate) {
  std::string print_out = "";
  print_out += predicate.arg1.name + " is ";
  if (predicate.negate) {
    print_out += "not ";
  }
  print_out += predicate.name;
  if (predicate.arg2.name != "") {
    print_out += " " + predicate.arg2.name;
  }
  return print_out;
}

}  // namespace pbd
}  // namespace rapid
