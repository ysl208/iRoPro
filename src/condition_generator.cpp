//
//  condition->cpp
//
//
//  Created by Ying Siu Sheryl Liang on 28/07/2017.
//
//

#include "rapid_pbd/condition_generator.h"

#include <tf/transform_datatypes.h>
#include <algorithm>
#include <cmath>
#include <exception>
#include <vector>

#include "rapid_pbd/robot_config.h"
#include "rapid_pbd/spatial_relations.h"
#include "transform_graph/graph.h"

#include "ros/ros.h"

namespace msgs = rapid_pbd_msgs;

namespace rapid {
namespace pbd {
ConditionGenerator::ConditionGenerator(const RobotConfig& robot_config)
    : robot_config_(robot_config) {}

void ConditionGenerator::AssignLandmarkCondition(
    const World& world, const std::string& landmark_name,
    msgs::Condition* condition) {
  World currentWorld = world;
  for (size_t i = 0; i < world.surface_box_landmarks.size(); ++i) {
    msgs::Landmark world_landmark = world.surface_box_landmarks[i];

    msgs::Surface surface = world.surface;
    if (surface.dimensions.x <= 0) {
      ROS_ERROR("No surface found: (%f,%f,%f)", surface.dimensions.x,
                surface.dimensions.y, surface.dimensions.z);
    }

    if (IsLandmarkName(world_landmark, landmark_name)) {
      condition->relevant = true;
      condition->landmark = world_landmark;

      float defaultVariance = 0.075;
      GetPropertyConditions(world_landmark, currentWorld, condition,
                            defaultVariance);
      CheckRelevantProperties(currentWorld, condition);
      // GenerateGrid(currentWorld, condition, currentWorld.points);
      // Add relation condition for each landmark
      GetRelativeConditions(world_landmark, currentWorld, condition,
                            defaultVariance);
      break;
    }
  }
}

void ConditionGenerator::AssignConditions(World* world) {
  for (size_t i = 0; i < world->surface_box_landmarks.size(); ++i) {
    msgs::Condition condition;
    msgs::Landmark world_landmark = world->surface_box_landmarks[i];
    std::string landmark_name = world_landmark.name;
    condition.surface = world->surface;
    AssignLandmarkCondition(*world, landmark_name, &condition);
  }
}

void ConditionGenerator::UpdateReferenceLandmark(
    const World& world, msgs::Condition* condition,
    const msgs::Landmark& reference) {
  float defaultVariance = 0.075;
  geometry_msgs::Vector3 defaultVarVector;
  defaultVarVector.x = defaultVariance;
  defaultVarVector.y = defaultVariance;
  defaultVarVector.z = defaultVariance;
  SetReferenceConditions(condition, reference, condition->landmark,
                         defaultVarVector);
}

void ConditionGenerator::GenerateGrid(
    msgs::Condition* condition, std::vector<geometry_msgs::PoseArray>* grid) {
  // Given a range [min,max], a point in the range x_0, a subrange around x with
  // width dim.x, (a distance d,) return an array of i points x_i in [min,max]
  // such that all x_i are spread out evenly
  // Note: min/max are the borders of the table

  // start at x_0, move left until border, move right
  // if (condition->positionRelevant) {
  geometry_msgs::PoseArray pose_array;
  std::vector<geometry_msgs::Pose> positions;

  geometry_msgs::Vector3 obj_distance;
  obj_distance.x =
      fmax(condition->surface_box_dims.x, fabs(condition->contDisplacement.x));
  obj_distance.y =
      fmax(condition->surface_box_dims.y, fabs(condition->contDisplacement.y));
  obj_distance.z =
      fmax(condition->surface_box_dims.z, fabs(condition->contDisplacement.z));

  // set corners of the allowed area
  geometry_msgs::Vector3 min_pos = condition->min_pos;
  geometry_msgs::Vector3 max_pos = condition->max_pos;
  GetPositions(min_pos, max_pos, condition->surface_box_dims, obj_distance,
               &positions);
  //}
  pose_array.poses = positions;
  grid->push_back(pose_array);
}

void ConditionGenerator::GetPositions(
    const geometry_msgs::Vector3& min_pos,
    const geometry_msgs::Vector3& max_pos,
    const geometry_msgs::Vector3& dimensions,
    const geometry_msgs::Vector3& obj_distance,
    std::vector<geometry_msgs::Pose>* positions) {
  // Takes the corner values of the area which should be filled with positions

  geometry_msgs::Pose pose;
  pose.position.z = min_pos.z;
  geometry_msgs::Vector3 local_min = min_pos;
  geometry_msgs::Vector3 local_max = max_pos;

  local_min.x += dimensions.x;
  local_min.y += dimensions.y;

  while (local_min.x < max_pos.x) {
    pose.position.x = local_min.x - (dimensions.x * 0.5);
    while (local_min.y < max_pos.y) {
      pose.position.y = local_min.y - (dimensions.y * 0.5);
      std::cout << "x y : " << pose.position.x << " , " << pose.position.y
                << "\n";
      positions->push_back(pose);
      local_min.y += +obj_distance.y;
    }
    local_min.y = min_pos.y + dimensions.y;  // reset y
    local_min.x += obj_distance.x;
    std::cout << "No. of positions: " << positions->size() << "\n";
  }
}

// void ConditionGenerator::GetPositionsAroundObject(
//     msgs::Condition* condition, const geometry_msgs::Vector3& dimensions,
//     const geometry_msgs::Vector3& obj_distance,
//     std::vector<geometry_msgs::Pose>* positions) {
//   geometry_msgs::Pose pose;
//   // 1. Add the actual position of the object x_0
//   pose.position = condition->position;
//   pose.orientation = condition->orientation;
//   // positions.push_back(pose);
//   ROS_INFO("Added first point: (%f,%f) ", pose.position.x, pose.position.y);
//   ROS_INFO("No. of positions: %d", positions.size());

//   // 2. Find spaces for x-coordinate
//   // 2.1 move x < x_0
//   float l_min = condition->min_pos.x;
//   float l_max = pose.position.x - (dimensions.x * 0.5);
//   ROS_INFO("l_min/l_max for x: (%f,%f) ", l_min, l_max);

//   // generate pose closest to x_0 (i.e. closest to l_max)
//   // geometry_msgs::Point new_point = condition->position;
//   while (l_max - obj_distance.x > l_min) {
//     pose.position.x = l_max - (obj_distance.x * 0.5);
//     l_max = pose.position.x - (obj_distance.x * 0.5);
//     positions.push_back(pose);  // add new point
//     ROS_INFO("Added point: (%f,%f) ", pose.position.x, pose.position.y);
//     ROS_INFO("No. of positions: %d", positions.size());
//   }
//   // 2.2 move x > x_0
//   float r_min = pose.position.x + (dimensions.x * 0.5);
//   float r_max = condition->max_pos.x;
//   ROS_INFO("r_min/r_max for x: (%f,%f) ", r_min, r_max);

//   while (r_min + obj_distance.x < r_max) {
//     pose.position.x = r_min + (obj_distance.x * 0.5);
//     r_min = pose.position.x + (obj_distance.x * 0.5);
//     positions.push_back(pose);  // add new point
//     ROS_INFO("Added point: (%f,%f) ", pose.position.x, pose.position.y);
//     ROS_INFO("No. of positions: %d", positions.size());
//   }
//   // 3. find spaces for y-coordinate
//   // 3.1 move before y_0
//   l_min = condition->min_pos.y;
//   l_max = pose.position.y - (dimensions.y * 0.5);
//   ROS_INFO("spaces for y-position: l_min/l_max: (%f,%f) ", l_min, l_max);

//   // place new y closest to y_0 (i.e. closest to l_max)
//   pose.position = condition->position;  // reset pose.position to x_0
//   while (l_max - obj_distance.y > l_min) {
//     pose.position.y = l_max - (obj_distance.y * 0.5);
//     l_max = pose.position.y - (obj_distance.y * 0.5);
//     positions.push_back(pose);  // add new point
//     ROS_INFO("Added point: (%f,%f) ", pose.position.x, pose.position.y);
//     ROS_INFO("No. of positions: %d", positions.size());
//   }
//   // 3.2 move after y_0
//   r_min = pose.position.y + (dimensions.y * 0.5);
//   r_max = condition->max_pos.y;
//   ROS_INFO("r_min/r_max: (%f,%f) ", r_min, r_max);

//   while (r_min + obj_distance.y < r_max) {
//     pose.position.y = r_min + (obj_distance.y * 0.5);
//     r_min = pose.position.y + (obj_distance.y * 0.5);
//     positions.push_back(pose);  // add new point
//     ROS_INFO("Added point: (%f,%f) ", pose.position.x, pose.position.y);
//     ROS_INFO("No. of positions: %d", positions.size());
//   }
// }

void ConditionGenerator::CheckRelevantProperties(const World& world,
                                                 msgs::Condition* condition) {
  // Given a condition for a landmark, check alignment properties and set to
  // relevant
  // TO DO: Check if object is cylinder, then no orientation

  msgs::Surface surface = world.surface;

  geometry_msgs::Vector3 surface_orientation =
      QuaternionToRPY(surface.pose_stamped.pose.orientation);
  // check if orientation matches table
  ROS_INFO("Surface orientation: %f", surface_orientation.z);
  ROS_INFO("Landmark orientation: %f", condition->eulerAngles.z);
  if (fabs(fmod(surface_orientation.z - condition->eulerAngles.z, 90)) < 5) {
    int multiple = (int)condition->eulerAngles.z / 90;
    condition->eulerAngles.z = multiple * 90.0;
    condition->orientationRelevant = true;
    ROS_INFO("new Landmark orientation: %f", condition->eulerAngles.z);
  }
  geometry_msgs::Vector3 object_dims = condition->surface_box_dims;
  if (fabs(fmod(condition->eulerAngles.z, 180)) > 85) {
    float temp = object_dims.x;
    object_dims.x = object_dims.y;
    object_dims.y = temp;
  }

  ROS_INFO("Surface dimensions: (%f,%f,%f)", surface.dimensions.x,
           surface.dimensions.y, surface.dimensions.z);
  ROS_INFO("Surface position: (%f,%f,%f)", surface.pose_stamped.pose.position.x,
           surface.pose_stamped.pose.position.y,
           surface.pose_stamped.pose.position.z);
  ROS_INFO("Surface orientation: (%f,%f,%f,%f)",
           surface.pose_stamped.pose.orientation.w,
           surface.pose_stamped.pose.orientation.x,
           surface.pose_stamped.pose.orientation.y,
           surface.pose_stamped.pose.orientation.z);
  ROS_INFO("Object dimensions: (%f,%f,%f)", object_dims.x, object_dims.y,
           object_dims.z);
  ROS_INFO("Object position: (%f,%f,%f)", condition->position.x,
           condition->position.y, condition->position.z);
  ROS_INFO("Object euler: (%f,%f,%f)", condition->eulerAngles.x,
           condition->eulerAngles.y, condition->eulerAngles.z);
  ROS_INFO("%s quaternion: (%f,%f,%f,%f)", condition->landmark.name.c_str(),
           condition->orientation.w, condition->orientation.x,
           condition->orientation.y, condition->orientation.z);

  // check if position on table is relevant for zones (borders and center)
  // TO DO: Double-check how to detect the edge of the whole surface
  // vertical zones (from the robot's perspective)
  geometry_msgs::Point surface_position = surface.pose_stamped.pose.position;
  geometry_msgs::Vector3 surface_dims = surface.dimensions;
  float dx = fabs(condition->position.x - surface.pose_stamped.pose.position.x);
  float t_edge = surface_position.x + surface_dims.x * 0.5;
  float b_edge = surface_position.x - surface_dims.x * 0.5;
  ROS_INFO("bottom edge 1 %f < %f", condition->position.x, surface_position.x);
  ROS_INFO("-- 2 %f < %f", fabs(b_edge - condition->position.x), object_dims.x);
  if ((condition->position.x < (b_edge + object_dims.x)) ||
      (dx < object_dims.x) ||
      (condition->position.x > (t_edge - object_dims.x))) {
    condition->positionVariance.x = condition->surface_box_dims.x * 0.5 + 0.01;
    condition->min_pos.x =
        condition->position.x - condition->positionVariance.x;
    condition->max_pos.x =
        condition->position.x + condition->positionVariance.x;
    condition->positionRelevant = true;
  } else {
    ROS_INFO(
        "x-coordinate does not seem relevant, keep range at table width: %f",
        surface_dims.x);
    object_dims.x = fabs(surface_dims.x);
  }
  ROS_INFO("set min_ max_ to %f , %f", condition->min_pos.x,
           condition->max_pos.x);

  // horizontal zones (from the robot's perspective)
  float l_edge = surface_position.y + surface_dims.y * 0.5;
  float r_edge = surface_position.y - surface_dims.y * 0.5;
  float dy = fabs(condition->position.y - surface.pose_stamped.pose.position.y);
  ROS_INFO("half length %f < %f", dy, (object_dims.x));
  ROS_INFO("left edge = %f - %f = %f", surface_position.y, surface_dims.y,
           l_edge);
  ROS_INFO("left edge 1 %f < %f", condition->position.y, surface_position.y);
  ROS_INFO("-- 2 %f < %f", fabs(l_edge - condition->position.y), object_dims.y);
  ROS_INFO("right edge  %f < %f", fabs(r_edge - condition->position.y),
           object_dims.y);
  if ((condition->position.y < (r_edge + object_dims.y)) ||
      (dy < object_dims.y) ||
      (condition->position.y > (l_edge - object_dims.y))) {
    // half length
    condition->positionVariance.y = condition->surface_box_dims.y * 0.5 + 0.01;
    condition->min_pos.y =
        condition->position.y - condition->positionVariance.y;
    condition->max_pos.y =
        condition->position.y + condition->positionVariance.y;
    condition->positionRelevant = true;
  } else {
    ROS_INFO(
        "y-coordinate does not seem relevant, keep range at table width: %f",
        surface_dims.y);
    object_dims.y = fabs(surface_dims.y);
  }
}

bool ConditionGenerator::IsLandmarkName(const msgs::Landmark& landmark,
                                        const std::string& landmark_name) {
  return landmark.name == landmark_name;
}

void ConditionGenerator::GetPropertyConditions(const msgs::Landmark& landmark,
                                               const World& world,
                                               msgs::Condition* condition,
                                               const float& defaultVariance) {
  // default values
  geometry_msgs::Vector3 defaultVarVector;
  defaultVarVector.x = defaultVariance;
  defaultVarVector.y = defaultVariance;
  defaultVarVector.z = defaultVariance;
  // size
  condition->sizeRelevant = true;
  condition->sizeVariance = defaultVarVector;
  condition->surface_box_dims = landmark.surface_box_dims;
  // position
  condition->positionRelevant = false;
  condition->positionVariance = defaultVarVector;
  geometry_msgs::Point surface_position =
      world.surface.pose_stamped.pose.position;
  geometry_msgs::Vector3 surface_dims = world.surface.dimensions;

  condition->min_pos.x = surface_position.x - surface_dims.x * 0.5;
  condition->min_pos.y = surface_position.y - surface_dims.y * 0.5;
  condition->min_pos.z = surface_position.z + condition->surface_box_dims.z * 0.5;
  condition->max_pos.x = surface_position.x + surface_dims.x * 0.5;
  condition->max_pos.y = surface_position.y + surface_dims.y * 0.5;
  condition->max_pos.z = surface_position.z + condition->surface_box_dims.z * 0.5;
  ROS_INFO("set min_ max_ to %f , %f", condition->min_pos.x,
           condition->max_pos.x);
  ROS_INFO("surface pos and dims %f , %f * 0.5 = %f", surface_position.x,
           surface_dims.x, surface_dims.x * 0.5);
  condition->position = landmark.pose_stamped.pose.position;
  // orientation
  condition->orientationRelevant = false;
  condition->orientationVariance = defaultVarVector;
  condition->orientation = landmark.pose_stamped.pose.orientation;

  condition->eulerAngles = QuaternionToRPY(condition->orientation);
  ROS_INFO("GetPropertyConditions: Object quaternion: (%f,%f,%f,%f)",
           condition->orientation.x, condition->orientation.y,
           condition->orientation.z, condition->orientation.w);
  ROS_INFO("-- Object euler: (%f,%f,%f)", condition->eulerAngles.x,
           condition->eulerAngles.y, condition->eulerAngles.z);
}

geometry_msgs::Vector3 ConditionGenerator::QuaternionToRPY(
    const geometry_msgs::Quaternion& msg) {
  tf::Quaternion quat(msg.x, msg.y, msg.z, msg.w);
  // the tf::Quaternion has a method to access roll pitch and yaw
  double roll, pitch, yaw;
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  ROS_INFO("quaternion: %f,%f,%f,%f", msg.w, msg.x, msg.y, msg.z);
  // the found angles are written in a geometry_msgs::Vector3
  geometry_msgs::Vector3 rpy;
  rpy.x = roll * 180.0 / M_PI;
  rpy.y = pitch * 180.0 / M_PI;
  rpy.z = yaw * 180.0 / M_PI;

  ROS_INFO("euler: %f,%f,%f", rpy.x, rpy.y, rpy.z);
  return rpy;
}

void ConditionGenerator::GetRelativeConditions(const msgs::Landmark& landmark,
                                               const World& world,
                                               msgs::Condition* condition,
                                               const float& defaultVariance) {
  // Find closest landmark and calculate displacement vector
  // msgs::Condition condition;
  // condition->landmark = landmark;

  // default values
  geometry_msgs::Vector3 defaultVarVector;
  defaultVarVector.x = defaultVariance;
  defaultVarVector.y = defaultVariance;
  defaultVarVector.z = defaultVariance;
  double distance_cutoff = 0.2;
  double squared_cutoff = distance_cutoff * distance_cutoff;

  msgs::Landmark reference;

  condition->referenceRelevant = false;
  if (ReferencedLandmark(landmark, world, squared_cutoff, &reference)) {
    SetReferenceConditions(condition, reference, landmark, defaultVarVector);
  }
}

void ConditionGenerator::SetReferenceConditions(
    msgs::Condition* condition, const msgs::Landmark& reference,
    const msgs::Landmark& landmark,
    const geometry_msgs::Vector3& defaultVarVector) {
  condition->contDisplacementRelevant = true;
  condition->referenceRelevant = true;
  condition->reference = reference;
  GetDisplacementVector(landmark, reference, condition);
  condition->contDisplacementVariance = defaultVarVector;

  condition->discDisplacementRelevant = false;
  GetSpatialRelation(condition);

  condition->contOrientationRelevant = true;
  condition->contOrientationVariance = defaultVarVector;
  condition->contEulerAngles = QuaternionToRPY(condition->contOrientation);

  condition->discOrientationRelevant = false;
  GetRelativeAlignment(condition);
}

bool ConditionGenerator::ReferencedLandmark(
    const msgs::Landmark& landmark, const World& world,
    const double squared_distance_cutoff, msgs::Landmark* reference) {
  // This is copied from editor.cpp ClosestLandmark but excludes the given
  // landmark
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

// Gets the current pose of the given landmark relative to the given reference
// landmark.
void ConditionGenerator::GetDisplacementVector(const msgs::Landmark& landmark,
                                               const msgs::Landmark& reference,
                                               msgs::Condition* condition) {
  // Get transform from landmark to reference
  transform_graph::Graph graph;

  // Get transform of given landmark relative to base
  tf::StampedTransform transform;
  std::string landmark_frame(landmark.pose_stamped.header.frame_id);
  if (landmark_frame != robot_config_.base_link()) {
    ROS_WARN("Given landmark not in base frame.");
  }
  graph.Add("landmark", transform_graph::RefFrame(landmark_frame),
            landmark.pose_stamped.pose);

  // Add frame for referenced landmark
  std::string reference_frame(reference.pose_stamped.header.frame_id);
  if (reference_frame != robot_config_.base_link()) {
    ROS_WARN("Referenced landmark not in base frame.");
  }
  graph.Add("reference", transform_graph::RefFrame(reference_frame),
            reference.pose_stamped.pose);

  // Get 6DOF displacement vector (x, y, z, θx, θy, θz) from transform
  transform_graph::Transform landmark_in_reference;
  bool success = graph.ComputeDescription(
      transform_graph::LocalFrame("reference"),
      transform_graph::RefFrame("landmark"), &landmark_in_reference);
  if (!success) {
    ROS_ERROR(
        "Unable to transform given landmark pose into referenced landmark!");
  }
  // Get rotation only
  //  landmark_in_reference.ToPose(&condition->contDisplacement);

  // Find 6DOF displacement vector (x, y, z, θx, θy, θz)
  geometry_msgs::Point landmark_pose = landmark.pose_stamped.pose.position;
  geometry_msgs::Point reference_pose = reference.pose_stamped.pose.position;

  geometry_msgs::Vector3 displacement;
  // Get Translation
  condition->contDisplacement.x = reference_pose.x - landmark_pose.x;
  condition->contDisplacement.y = reference_pose.y - landmark_pose.y;
  condition->contDisplacement.z = reference_pose.z - landmark_pose.z;
  // Get orientation
  condition->contOrientation.x = reference_pose.x - landmark_pose.x;
  condition->contOrientation.y = reference_pose.y - landmark_pose.y;
  condition->contOrientation.z = reference_pose.z - landmark_pose.z;
}

void ConditionGenerator::GetRelativeAlignment(msgs::Condition* condition) {
  // if theta_z + 90 = 90 mod 180 then orthogonal
  // if              = 0 mod 180 then parallel
  // theta_x, _y do not need to be taken into account
  std::string alignment;
  alignment = "None";
  int theta_z = static_cast<int>(condition->contEulerAngles.z);
  // float orient = ( theta_z ) % 180;
  float defaultVariance = 0.075;

  if ((theta_z % 180) < defaultVariance * 100) {
    alignment = "parallel";
    condition->discOrientationRelevant = true;
  } else if ((theta_z % 90) < defaultVariance * 100) {
    alignment = "orthogonal";
    condition->discOrientationRelevant = true;
  }
  condition->alignment = alignment;
  ROS_INFO("alignment is %s ", alignment.c_str());
}

void ConditionGenerator::GetSpatialRelation(msgs::Condition* condition) {
  // Note: x-axis points forward from the robot base
  //       y-axis points to the left, z-axis points up
  // use x,y,z coordinates to get preposition
  std::string spatial_relation;
  float dx = condition->contDisplacement.x;
  float dy = condition->contDisplacement.y;
  float dz = condition->contDisplacement.z;

  // Get given landmark's bounding box size
  // with x always <= y (see world.cpp)
  geometry_msgs::Vector3 dims = condition->landmark.surface_box_dims;
  float min_dim = std::min(dims.x, dims.y);
  float max_dim = std::max(dims.x, dims.y);
  float width = std::min(fabs(dx), fabs(dy));
  float length = std::max(fabs(dx), fabs(dy));
  // Check if object is above/inside
  if ((width < min_dim * 0.5) && (length < max_dim * 0.5)) {
    // above or inside
    if (fabs(dz) < dims.z * 0.5) {
      spatial_relation = aboveSpatialRelation;
    } else {
      spatial_relation = insideSpatialRelation;
    }
  } else {
    // Object is 'around' (left/right/front/behind)
    if (fabs(dx) < fabs(dy)) {
      // left or right
      if (dy < 0) {
        spatial_relation = leftSpatialRelation;
      } else {
        spatial_relation = rightSpatialRelation;
      }
    } else {
      // behind/front - assuming y-axis is towards the robot
      if (dx < 0) {
        spatial_relation = behindSpatialRelation;
      } else {
        spatial_relation = frontSpatialRelation;
      }
    }
  }
  condition->spatial_relation = spatial_relation;
}

}  // namespace pbd
}  // namespace rapid
