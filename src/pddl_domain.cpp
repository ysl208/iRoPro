#include "rapid_pbd/pddl_domain.h"

#include <math.h>
#include <algorithm>  // std::find
#include <set>
#include <string>
#include <vector>

#include "geometry_msgs/Pose.h"
#include "rapid_pbd_msgs/Action.h"
#include "rapid_pbd_msgs/Landmark.h"
#include "rapid_pbd_msgs/Program.h"
#include "rapid_pbd_msgs/Step.h"
#include "transform_graph/graph.h"

#include "rapid_pbd/landmarks.h"
#include "rapid_pbd/robot_config.h"

#include "ros/ros.h"

namespace msgs = rapid_pbd_msgs;

namespace rapid {
namespace pbd {
PDDLDomain::PDDLDomain(const ros::Publisher& pub) : pddl_domain_pub_(pub) {}

void PDDLDomain::Init(msgs::PDDLDomain* domain, const std::string& name) {
  domain->name = name;
  domain->requirements = ":typing :strips";
  msgs::PDDLType type;

  // get names of different objects from yaml file
  std::vector<std::string> name_list;
  std::vector<std::string> parent_list;
  std::vector<double> obj_dims;
  ros::param::param<std::vector<std::string> >("world_objects/names", name_list,
                                               name_list);
  ros::param::param<std::vector<std::string> >("world_objects/parents",
                                               parent_list, parent_list);

  for (size_t i = 0; i < name_list.size(); ++i) {
    std::stringstream ss;
    ss << name_list[i];
    type.name = ss.str();
    if (parent_list.size() > i) type.parent = parent_list[i];
    std::string name = "world_objects/" + type.name;

    ros::param::param<std::vector<double> >(name, obj_dims, obj_dims);
    type.dimensions.x = obj_dims[0];
    type.dimensions.y = obj_dims[1];
    type.dimensions.z = obj_dims[2];

    ROS_INFO("Added type %s with parent %s", type.name.c_str(),
             type.parent.c_str());
    AddType(&domain->types, type);
  }

  msgs::PDDLPredicate predicate;
  // Predicate has at least 1 arg of type table_entity
  msgs::PDDLObject obj;
  obj.name = "obj1";
  type.name = msgs::PDDLType::ENTITY;
  obj.type = type;
  predicate.arg1 = obj;

  predicate.name = msgs::PDDLPredicate::IS_CLEAR;
  domain->predicates.push_back(predicate);

  // Predicates with 2 args, 1st arg needs to be an object
  predicate.name = msgs::PDDLPredicate::IS_ON;
  predicate.arg2 = obj;

  msgs::PDDLObject obj2;
  obj2.name = "obj2";
  type.name = msgs::PDDLType::OBJECT;
  obj2.type = type;
  predicate.arg1 = obj2;
  domain->predicates.push_back(predicate);

  predicate.name = msgs::PDDLPredicate::IS_STACKABLE;
  domain->predicates.push_back(predicate);
  ROS_INFO("# Predicates now: %zd", domain->predicates.size());
}

void PDDLDomain::PublishPDDLDomain(const msgs::PDDLDomain& domain) {
  ROS_INFO("Publish PDDL Domain..");
  domain_ = domain;
  pddl_domain_pub_.publish(domain);
}
void AddType(std::vector<msgs::PDDLType>* types,
             const msgs::PDDLType& new_type) {
  // if (find(types->begin(), types->end(), new_type) != types->end()) {
  types->push_back(new_type);
  ROS_INFO("Added type #%zd: %s", types->size(), new_type.name.c_str());
  // } else {
  //  ROS_INFO("%s already exists", new_type.name.c_str());
  //}
}

void GetWorldState(const std::vector<msgs::Landmark>& world_landmarks,
                   WorldState* world_state) {
  // Given: World_landmarks
  // Create World state of objects
  ROS_INFO("Getting World State with %zd landmarks...", world_landmarks.size());

  world_state->objects_.clear();
  world_state->predicates_.clear();
  world_state->positions_.clear();
  std::vector<msgs::PDDLObject> args;
  std::string predicate;
  msgs::PDDLObject position;
  bool negate = false;
  double position_radius = 0.5;
  msgs::PDDLObject obj;
  msgs::PDDLType obj_type;

  for (size_t i = 0; i < world_landmarks.size(); ++i) {
    const msgs::Landmark& world_landmark = world_landmarks[i];
    ROS_INFO("Generating predicates for %s at (%f,%f,%f)",
             world_landmark.name.c_str(),
             world_landmark.pose_stamped.pose.position.x,
             world_landmark.pose_stamped.pose.position.y,
             world_landmark.pose_stamped.pose.position.z);
    ROS_INFO("and size (%f,%f,%f)", world_landmark.surface_box_dims.x,
             world_landmark.surface_box_dims.y,
             world_landmark.surface_box_dims.z);

    msgs::PDDLType obj_type;
    if (world_landmark.name.find("obj") != std::string::npos) {
      // Generate Predicates for type == OBJECT
      obj_type.name = msgs::PDDLType::OBJECT;
      GetTypeFromDims(world_landmark.surface_box_dims, &obj_type);
    } else if (world_landmark.name.find("pos") != std::string::npos) {
      // Generate Predicates for type == POSITION
      obj_type.name = msgs::PDDLType::POSITION;
    } else {
      obj_type.name = msgs::PDDLType::ENTITY;
    }
    obj_type.parent = msgs::PDDLType::ENTITY;

    obj_type.pose = world_landmark.pose_stamped.pose;
    obj_type.dimensions = world_landmark.surface_box_dims;
    obj.type = obj_type;

    obj.name = world_landmark.name;
    obj.surface_box_dims = world_landmark.surface_box_dims;
    AddObject(&world_state->objects_, obj);
    ros::param::param<double>("world_positions/radius", position_radius,
                              position_radius);
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

      // // Due to closed world assumption not needed (implied by having IS_ON)
      // predicate = msgs::PDDLPredicate::IS_STACKABLE;
      // AddPredicate(&world_state->predicates_, predicate, args, negate);

      // // Object is clear
      // predicate = msgs::PDDLPredicate::IS_CLEAR;
      // args.clear();
      // args.push_back(obj);
      // AddPredicate(&world_state->predicates_, predicate, args, negate);

      // // Position is not clear because occupied by object
      // predicate = msgs::PDDLPredicate::IS_CLEAR;
      // args.clear();
      // args.push_back(position);
      // negate = true;
      // AddPredicate(&world_state->predicates_, predicate, args, negate);
      // ROS_INFO("Predicates now: %zd", world_state->predicates_.size());
    } else {
      ROS_ERROR("Object not on any predefined position");
    }
  }
  // Generate Predicates for type == POSITION
  // based on IS_ON predicates, we can infer IS_CLEAR
  for (size_t i = 0; i < world_state->positions_.size(); ++i) {
    const msgs::PDDLObject& pos_object = world_state->positions_[i];

    args.clear();
    obj.name = "";
    args.push_back(obj);
    args.push_back(pos_object);
    if (!PredicateExists(&world_state->predicates_, msgs::PDDLPredicate::IS_ON,
                         args)) {
      predicate = msgs::PDDLPredicate::IS_CLEAR;
      args.clear();
      args.push_back(pos_object);
      negate = false;
      AddPredicate(&world_state->predicates_, predicate, args, negate);
    }
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
    ROS_INFO("Added predicate #%zd: %s(%s)", predicates->size(), name.c_str(),
             args[0].name.c_str());
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
    if (pred.name == predicate) {
      if (predicate == msgs::PDDLPredicate::IS_CLEAR) {
        // only check first argument if predicate is 'is clear'
        return pred.arg1.name == args[0].name;
      }

      if (pred.name == msgs::PDDLPredicate::IS_ON) {
        if (args.size() < 2) {
          ROS_ERROR("Predicate IS_ON does not have 2 arguments");
        }
        if (pred.arg1.name == args[0].name || pred.arg2.name == args[1].name) {
          ROS_INFO(
              "PredicateExists: Trying to add %s(%s, %s), but %s(%s, %s) "
              "already exists ",
              predicate.c_str(), args[0].name.c_str(), args[1].name.c_str(),
              predicate.c_str(), pred.arg1.name.c_str(),
              pred.arg2.name.c_str());
          return true;
        }
      }
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
  if (obj.name == msgs::PDDLType::OBJECT ||
      obj.parent == msgs::PDDLType::OBJECT) {
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
        double squared_distance = dx * dx + dy * dy;  // + dz * dz;
        // ROS_INFO("Dist = %f < cutoff %f", squared_distance, squ_dist_cutoff);

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
  obj_type->parent = msgs::PDDLType::OBJECT;

  // get names of different objects, then iterate through them to get their
  // dimensions and compare to given dims
  std::vector<std::string> name_list;
  std::vector<double> obj_dims;
  std::vector<double> variance;

  ros::param::param<std::vector<std::string> >("world_objects/names", name_list,
                                               name_list);

  ros::param::param<std::vector<double> >("world_objects/variance", variance,
                                          variance);

  double closest_distance = std::numeric_limits<double>::max();
  double squ_dist_cutoff = variance[0] * variance[0];
  for (size_t i = 0; i < name_list.size(); ++i) {
    std::stringstream ss;
    ss << name_list[i];
    obj_type->name = ss.str();
    ros::param::param<std::vector<double> >("world_objects/" + obj_type->name,
                                            obj_dims, obj_dims);
    obj_type->dimensions.x = obj_dims[0];
    obj_type->dimensions.y = obj_dims[1];
    obj_type->dimensions.z = obj_dims[2];
    double dx = obj_dims[0] - dims.x;
    double dy = obj_dims[0] - dims.y;
    double dz = obj_dims[0] - dims.z;
    double squared_distance = dx * dx + dy * dy + dz * dz;
    if (dims.z > 0.005 && squared_distance < closest_distance &&
        squared_distance <= squ_dist_cutoff) {
      // closest_distance = squared_distance;
      ROS_INFO("Assigned type %s with dims (%f,%f,%f)", obj_type->name.c_str(),
               obj_type->dimensions.x, obj_type->dimensions.y,
               obj_type->dimensions.z);
      return;
    }
  }
  obj_type->name = "object";
  obj_type->dimensions.x = 1.0;
  obj_type->dimensions.y = 1.0;
  obj_type->dimensions.z = 1.0;
}

std::string PrintAllPredicates(std::vector<msgs::PDDLPredicate> predicates,
                               std::string type) {
  std::string ss = "";
  if (type != "") {
    for (size_t i = 0; i < predicates.size(); ++i) {
      msgs::PDDLPredicate predicate = predicates[i];
      ss += PrintPDDLPredicate(predicate, type) + " ";
    }
    if (predicates.size() > 1) {
      ss = "(and " + ss + ")";
    }
  } else {
    for (size_t i = 0; i < predicates.size(); ++i) {
      msgs::PDDLPredicate predicate = predicates[i];
      ss += PrintPredicate(predicate) + "\n";
    }
  }
  return ss;
}

std::string PrintPDDLPredicate(msgs::PDDLPredicate predicate,
                               std::string pred_type) {
  // pred_type is used for different PDDL output
  // can be either predicate, precondition/effect, or init/goal
  std::string print_out = "";
  print_out += predicate.name + " ";

  if (pred_type != "init" && pred_type != "goal") {  // include '?' if parameter
    print_out += "?";
  }
  std::string str = predicate.arg1.name;
  str.erase(remove_if(str.begin(), str.end(), isspace), str.end());
  print_out += str;
  if (pred_type == "predicate") {  // need to include object type
    print_out += " - " + predicate.arg1.type.name;
  }

  if (predicate.arg2.name != "") {
    print_out += " ";
    if (pred_type != "init" &&
        pred_type != "goal") {  // include '?' if parameter
      print_out += "?";
    }
    str = predicate.arg2.name;
    str.erase(remove_if(str.begin(), str.end(), isspace), str.end());
    print_out += str;
    if (pred_type == "predicate") {  // need to include object type
      print_out += " - " + predicate.arg2.type.name;
    }
  }
  if (predicate.negate) {
    print_out = "not(" + print_out + ")";
  }
  return "(" + print_out + ")";
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
