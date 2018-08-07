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
  domain->requirements = ":typing";
  msgs::PDDLType type;
  geometry_msgs::Vector3 obj_dims;
  // TO DO: get obj_dims from yaml file
  obj_dims.x = 1.0;
  obj_dims.x = 1.0;
  obj_dims.x = 1.0;

  type.name = msgs::PDDLType::TABLE_ENTITY;
  domain->types.push_back(type);

  type.name = msgs::PDDLType::OBJECT;
  type.parent = msgs::PDDLType::TABLE_ENTITY;
  domain->types.push_back(type);
  type.name = msgs::PDDLType::POSITION;
  AddType(&domain->types, type);
  type.name = msgs::PDDLType::CUBE_OBJECT;
  type.parent = msgs::PDDLType::OBJECT;
  AddType(&domain->types, type);
  type.name = msgs::PDDLType::TOWER_OBJECT;
  AddType(&domain->types, type);
  type.name = msgs::PDDLType::PLATE_OBJECT;
  AddType(&domain->types, type);

  msgs::PDDLPredicate predicate;
  // Predicate has at least 1 arg of type table_entity
  msgs::PDDLObject obj;
  obj.name = "Obj1";
  type.name = msgs::PDDLType::TABLE_ENTITY;
  obj.type = type;
  predicate.arg1 = obj;

  predicate.name = msgs::PDDLPredicate::IS_CLEAR;
  domain->predicates.push_back(predicate);

  // Predicates with 2 args, 1st arg needs to be an object
  predicate.name = msgs::PDDLPredicate::IS_ON;
  predicate.arg2 = obj;

  msgs::PDDLObject obj2;
  obj2.name = "Obj2";
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

  // Add positions to world state objects
  std::vector<msgs::PDDLObject> fixed_positions;
  GetFixedPositions(&fixed_positions);
  world_state->objects_ = fixed_positions;
  world_state->positions_ = fixed_positions;

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
    // Generate Predicates for type == OBJECT
    std::stringstream ss;
    ss << "Obj" << i + 1;
    obj.name = ss.str();
    msgs::PDDLType obj_type;
    obj_type.name = msgs::PDDLType::OBJECT;
    GetTypeFromDims(world_landmark.surface_box_dims, &obj_type);
    obj_type.pose = world_landmark.pose_stamped.pose;
    obj_type.dimensions = world_landmark.surface_box_dims;
    obj.type = obj_type;
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

      // Due to closed world assumption not needed (implied by having IS_ON)
      // predicate = msgs::PDDLPredicate::IS_STACKABLE;
      // AddPredicate(&world_state->predicates_, predicate, args, negate);

      // predicate = msgs::PDDLPredicate::IS_CLEAR;
      // args.clear();
      // args.push_back(obj);
      // AddPredicate(&world_state->predicates_, predicate, args, negate);

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
  obj_type->name = msgs::PDDLType::RECTANGLE_OBJECT;
  obj_type->parent = msgs::PDDLType::OBJECT;

  // vector <map<std::string,std::string> > objects;
  // ros::param::param< vector <map<std::string,std::string> >
  // >("world_objects",objects, objects);

  /*double x_y_ratio = dims.x/dims.y;
  double x_z_ratio = dims.x/dims.z;
  double y_z_ratio = dims.y/dims.z;
  */
  double x = dims.x;
  double y = dims.y;
  double z = dims.z;
  std::vector<std::string> types;
  // types.push_back = {"cube", "rectangle", "tower", "plate"};
  double x_y_ratio;
  double x_z_ratio;
  double y_z_ratio;
  // for (size_t i = 0; i < types.size(); ++i) {
  std::string type = "rectangle";
  /* std::map<std::string, double> cube;
  std::string param_name = "world_objects/" + types[i];
  ros::param::param<std::map<std::string, double> >(param_name, cube, cube); */
  /* std::cout << cube.find("x") << std::endl;

  x_z_ratio = cube.find("x") / cube.find("z"); */
  if (x / y > 0.85) {
    // is one of three shapes
    std::cout << x / z << std::endl;
    std::cout << y / z << std::endl;
    if (x / z > 3.5 || y / z > 3.5) {
      type = "plate";
      obj_type->name = msgs::PDDLType::PLATE_OBJECT;
    } else if ((x / z > 0.85 || y / z > 0.85) && (x / z < 1.3 || y / z < 1.3)) {
      type = "cube";
      obj_type->name = msgs::PDDLType::CUBE_OBJECT;
    } else if (x / z < 0.5 || y / z < 0.5) {
      type = "tower";
      obj_type->name = msgs::PDDLType::TOWER_OBJECT;
    }
  }
  //}
  /*for (size_t i = 0; i < objects.size(); ++i) {
  std::cout << objects.at('name').c_str() << std::endl;

  std::cout << objects.at('dims_ratio').c_str() << std::endl;
}*/
}

void GetFixedPositions(std::vector<msgs::PDDLObject>* objects) {
  msgs::PDDLObject obj;
  msgs::PDDLType obj_type;
  obj_type.name = msgs::PDDLType::POSITION;

  std::vector<double> pos_x_list, pos_y_list, pos_z_list;
  std::vector<std::string> name_list;

  ros::param::param<std::vector<std::string> >("world_positions/names",
                                               name_list, name_list);
  ros::param::param<std::vector<double> >("world_positions/pos_x", pos_x_list,
                                          pos_x_list);
  ros::param::param<std::vector<double> >("world_positions/pos_y", pos_y_list,
                                          pos_x_list);
  ros::param::param<std::vector<double> >("world_positions/pos_z", pos_z_list,
                                          pos_x_list);

  for (size_t i = 0; i < name_list.size(); ++i) {
    std::stringstream ss;
    ss << "Position" << name_list[i];
    obj.name = ss.str();
    geometry_msgs::Pose pose;
    pose.position.x = pos_x_list[i];
    pose.position.y = pos_y_list[i];
    pose.position.z = pos_z_list[i];
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
