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
  ROS_INFO("Assign landmarks: (%d) ", world.surface_box_landmarks.size());

  for (size_t i = 0; i < world.surface_box_landmarks.size(); ++i) {
    msgs::Landmark world_landmark = world.surface_box_landmarks[i];
    ROS_INFO("landmark: (%s) ", world_landmark.name.c_str());
    if (world.surface.size() > 0) {
      // ROS_INFO("- : Surface orientation: %f for 1/%d surfaces",
      //          world.surface[0].pose_stamped.pose.orientation.z,
      //          world.surface.size());
    }
    if (IsLandmarkName(world_landmark, landmark_name)) {
      condition->relevant = true;
      condition->landmark = world_landmark;

      float defaultVariance = 0.075;
      GetPropertyConditions(world_landmark, world, condition, defaultVariance);
      CheckRelevantProperties(world, condition);

      // Add relation condition for each landmark
      GetRelativeConditions(world_landmark, world, condition, defaultVariance);
      break;
    }
  }
}

void ConditionGenerator::AssignConditions(World* world) {
  for (size_t i = 0; i < world->surface_box_landmarks.size(); ++i) {
    msgs::Condition condition;
    msgs::Landmark world_landmark = world->surface_box_landmarks[i];
    std::string landmark_name = world_landmark.name;
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

void ConditionGenerator::GenerateGrid(World* world,
                                      const msgs::Condition& condition) {
  msgs::Surface surface = world->surface[0];
  if (surface.pose_stamped.pose.position.x <= 0) {
    GetDefaultSurface(&surface);
  }
  std::vector<std::vector<std::string> > grid;
  std::vector<geometry_msgs::Point> positions;
  positions.push_back(condition.position);

  // divide the surface into cells, each cell has the size of the landmark
  // dimension

  if (condition.positionRelevant) {
  }
}

void ConditionGenerator::GetDefaultSurface(msgs::Surface* surface) {
  surface->dimensions.x = 0.426411;
  surface->dimensions.y = 0.851767;
  surface->dimensions.z = 0.009999;

  surface->pose_stamped.pose.position.x = 0.686756;
  surface->pose_stamped.pose.position.y = 0.066172;
  surface->pose_stamped.pose.position.z = 0.717414;
  // converted to zero degrees
  surface->pose_stamped.pose.orientation.x = -0.007211;
  surface->pose_stamped.pose.orientation.y = -0.007610;
  surface->pose_stamped.pose.orientation.z = -0.005776;
  surface->pose_stamped.pose.orientation.w = 0.999928;
  ROS_INFO(
      "Using default surface values - might not align with actual surface!");
}

void ConditionGenerator::CheckRelevantProperties(const World& world,
                                                 msgs::Condition* condition) {
  // Given a condition for a landmark, check alignment properties and set to
  // relevant
  // TO DO: Check if object is cylinder, then no orientation

  msgs::Surface surface = world.surface[0];
  if (surface.pose_stamped.pose.position.x <= 0) {
    GetDefaultSurface(&surface);
  }
  ROS_INFO("Surface dimensions: (%f,%f,%f)", surface.dimensions.x,
           surface.dimensions.y, surface.dimensions.z);
  ROS_INFO("Surface position: (%f,%f,%f)", surface.pose_stamped.pose.position.x,
           surface.pose_stamped.pose.position.y,
           surface.pose_stamped.pose.position.z);
  ROS_INFO("Surface orientation: (%f,%f,%f,%f)",
           surface.pose_stamped.pose.orientation.x,
           surface.pose_stamped.pose.orientation.y,
           surface.pose_stamped.pose.orientation.z,
           surface.pose_stamped.pose.orientation.w);
  ROS_INFO("Object dimensions: (%f,%f,%f)", condition->surface_box_dims.x,
           condition->surface_box_dims.y, condition->surface_box_dims.z);
  ROS_INFO("Object position: (%f,%f,%f)", condition->position.x,
           condition->position.y, condition->position.z);
  ROS_INFO("Object euler: (%f,%f,%f)", condition->eulerAngles.x,
           condition->eulerAngles.y, condition->eulerAngles.z);
  ROS_INFO("%s quaternion: (%f,%f,%f,%f)", condition->landmark.name.c_str(),
           condition->orientation.w, condition->orientation.x,
           condition->orientation.y, condition->orientation.z);

  geometry_msgs::Vector3 surface_orientation =
      QuaternionToRPY(surface.pose_stamped.pose.orientation);
  // check if orientation matches table
  ROS_INFO("Surface orientation: %f", surface_orientation.z);
  ROS_INFO("Landmark orientation: %f", condition->eulerAngles.z);
  if (fabs(fmod(surface_orientation.z, 90) -
           fmod(condition->eulerAngles.z, 90)) < 5) {
    int multiple = (int)condition->eulerAngles.z / 90;
    condition->eulerAngles.z = multiple * 90.0;
    condition->orientationRelevant = true;
    ROS_INFO("new Landmark orientation: %f", condition->eulerAngles.z);
  }

  // check if position on table is relevant for zones (borders and center)
  // TO DO: Double-check how to detect the edge of the whole surface
  // vertical zones (from the robot's perspective)
  geometry_msgs::Point surface_position = surface.pose_stamped.pose.position;
  geometry_msgs::Vector3 surface_dims = surface.dimensions;
  float dx = fabs(condition->position.x - surface.pose_stamped.pose.position.x);
  float dy = fabs(condition->position.y - surface.pose_stamped.pose.position.y);
  float t_edge = surface_position.x - surface_dims.x * 0.5;
  float b_edge = surface_position.x + surface_dims.x * 0.5;
  ROS_INFO("bottom edge 1 %f < %f", condition->position.x, surface_position.x);
  ROS_INFO("-- 2 %f < %f", fabs(b_edge - condition->position.x),
           condition->surface_box_dims.x * 0.5);
  if (dx < condition->surface_box_dims.y * 0.5) {
    // half width, set x-coordinate to table center
    condition->position.x = surface.pose_stamped.pose.position.x;
    condition->positionVariance.x = 0.01;  // 0.01m
    condition->positionRelevant = true;
  } else if ((condition->position.x < surface_position.x) &&
             (fabs(b_edge - condition->position.x) <
              condition->surface_box_dims.x * 0.5)) {
    // landmark on bottom edge of table, minimize variance
    condition->positionVariance.x = 0.01;
    condition->positionRelevant = true;
  } else if ((condition->position.x > surface_position.x) &&
             (fabs(t_edge - condition->position.x) <
              condition->surface_box_dims.x * 0.5)) {
    // landmark on top edge of table, minimize variance
    condition->positionVariance.x = 0.01;
    condition->positionRelevant = true;
  } else {
    ROS_INFO(
        "x-coordinate does not seem relevant, set range to table width: %f",
        surface_dims.x);
    condition->positionVariance.x = surface_dims.x * 0.5;
  }

  // horizontal zones (from the robot's perspective)
  float l_edge = surface_position.y - surface_dims.y * 0.5;
  float r_edge = surface_position.y + surface_dims.y * 0.5;
  ROS_INFO("half length %f < %f", dy, (condition->surface_box_dims.x * 0.5));
  ROS_INFO("left edge = %f - %f = %f", surface_position.y, surface_dims.y * 0.5,
           l_edge);
  ROS_INFO("left edge 1 %f < %f", condition->position.y, surface_position.y);
  ROS_INFO("left edge 1 %f < %f", condition->position.y, surface_position.y);
  ROS_INFO("-- 2 %f < %f", fabs(l_edge - condition->position.y),
           condition->surface_box_dims.y * 0.5);
  if (dy < condition->surface_box_dims.x * 0.5) {
    // half length
    condition->position.y = surface.pose_stamped.pose.position.y;
    condition->positionVariance.y = 0.01;  // 0.01m
    condition->positionRelevant = true;
  } else if ((condition->position.y <= surface_position.y) &&
             (fabs(l_edge - condition->position.y) <
              condition->surface_box_dims.y * 0.5)) {
    // landmark on left edge of table
    condition->positionVariance.y = 0.01;
    condition->positionRelevant = true;
  } else if ((condition->position.y > surface_position.y) &&
             (fabs(r_edge - condition->position.y) <
              condition->surface_box_dims.y * 0.5)) {
    // landmark on left edge of table
    condition->positionVariance.y = 0.01;
    condition->positionRelevant = true;
  } else {
    ROS_INFO(
        "y-coordinate does not seem relevant, set range to table width: %f",
        surface_dims.y);
    condition->positionVariance.y = surface_dims.y * 0.5;
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
  // the incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
  tf::Quaternion quat;
  tf::quaternionMsgToTF(msg, quat);

  // the tf::Quaternion has a method to access roll pitch and yaw
  double roll, pitch, yaw;
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

  ROS_INFO("quaternion: %f,%f,%f,%f", msg.w, msg.x, msg.y, msg.z);

  // the found angles are written in a geometry_msgs::Vector3
  geometry_msgs::Vector3 rpy;
  rpy.x = roll;
  rpy.y = pitch;
  rpy.z = yaw;

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
