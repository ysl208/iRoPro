//
//  condition->cpp
//
//
//  Created by YSL on 28/07/2017.
//
//

#include "rapid_pbd/condition_generator.h"

#include <exception>
#include <cmath>
#include <vector>
#include <algorithm>    // std::min

#include "rapid_pbd/spatial_relations.h"
#include "transform_graph/graph.h"
#include "rapid_pbd/robot_config.h"

#include "ros/ros.h"

namespace msgs = rapid_pbd_msgs;

namespace rapid {
namespace pbd {
ConditionGenerator::ConditionGenerator(const RobotConfig& robot_config)
    : robot_config_(robot_config) {}


void ConditionGenerator::AssignLandmarkCondition(const World& world, 
                                              const std::string& landmark_name, 
                                              msgs::Condition* condition)
{
  for (size_t i = 0; i < world.surface_box_landmarks.size(); ++i)
  {
    msgs::Landmark world_landmark = world.surface_box_landmarks[i];
    if (IsLandmarkName(world_landmark, landmark_name)){
      condition->relevant = true;
      condition->landmark = world_landmark;
       ROS_INFO("%s at (%f,%f,%f)", 
                world_landmark.name.c_str(), world_landmark.pose_stamped.pose.position.x,
                world_landmark.pose_stamped.pose.position.y, world_landmark.pose_stamped.pose.position.z); 
      
      float defaultVariance = 0.075;
      GetPropertyConditions(world_landmark, world, condition, defaultVariance);

      // Add relation condition for each landmark
      GetRelativeConditions(world_landmark, world, condition, defaultVariance);
      // ROS_INFO("%s relative to (%s)", 
      //   condition->landmark.name.c_str(), condition->reference.name.c_str());    
      break;
    }
  }
}

void ConditionGenerator::AssignConditions(World* world){
  for (size_t i = 0; i < world->surface_box_landmarks.size(); ++i)
  {
    msgs::Condition condition;
    msgs::Landmark world_landmark = world->surface_box_landmarks[i];
    std::string landmark_name = world_landmark.name;
    AssignLandmarkCondition(*world, landmark_name, &condition);
  }
}

bool ConditionGenerator::IsLandmarkName(const msgs::Landmark& landmark, const std::string& landmark_name)
{
  return landmark.name == landmark_name;
}

void ConditionGenerator::GetPropertyConditions(const msgs::Landmark& landmark,
                                              const World& world,
                                              msgs::Condition* condition,
                                              const float& defaultVariance)
{
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
  condition->positionRelevant = true;
  condition->positionVariance = defaultVarVector;
  condition->position = landmark.pose_stamped.pose.position;
  // orientation
  condition->orientationRelevant = true;
  condition->orientationVariance = defaultVarVector;
  condition->orientation = landmark.pose_stamped.pose.orientation;
}

void ConditionGenerator::GetRelativeConditions(const msgs::Landmark& landmark,
                                              const World& world,
                                              msgs::Condition* condition,
                                              const float& defaultVariance)
{
  // Find closest landmark and calculate displacement vector
  //msgs::Condition condition;
  //condition.landmark = landmark;

  // default values
  geometry_msgs::Vector3 defaultVarVector;
  defaultVarVector.x = defaultVariance;
  defaultVarVector.y = defaultVariance;
  defaultVarVector.z = defaultVariance;
  double distance_cutoff = 0.2;
  double squared_cutoff = distance_cutoff * distance_cutoff;

  msgs::Landmark reference;

  if (ReferencedLandmark(landmark, world, squared_cutoff, &reference))
  {
/*     ROS_INFO("%s  references landmark %s with threshold %f", 
      landmark.name.c_str(), reference.name.c_str(), distance_cutoff); */
    condition->referenceRelevant = true;
    condition->reference = reference;
    GetDisplacementVector(landmark, reference, condition);
    condition->contDisplacementVariance = defaultVarVector;

    condition->contOrientationRelevant = true;
    condition->contOrientationVariance = defaultVarVector;

    condition->discDisplacementRelevant = false;
    GetSpatialRelation(condition);
  }
}

bool ConditionGenerator::ReferencedLandmark(const msgs::Landmark& landmark,
                                            const World& world,
                                            const double squared_distance_cutoff,
                                            msgs::Landmark* reference)
{
  // This is copied from editor.cpp ClosestLandmark but excludes the given landmark
  
  bool success = false;
  double closest_distance = std::numeric_limits<double>::max();

  for (size_t i = 0; i < world.surface_box_landmarks.size(); ++i)
  {
    const msgs::Landmark &world_landmark = world.surface_box_landmarks[i];

    if(landmark.name != world_landmark.name){

      geometry_msgs::Point world_pose;
      world_pose.x = world_landmark.pose_stamped.pose.position.x;
      world_pose.y = world_landmark.pose_stamped.pose.position.y;
      world_pose.z = world_landmark.pose_stamped.pose.position.z;
      double dx = world_pose.x - landmark.pose_stamped.pose.position.x;
      double dy = world_pose.y - landmark.pose_stamped.pose.position.y;
      double dz = world_pose.z - landmark.pose_stamped.pose.position.z;
      double squared_distance = dx * dx + dy * dy + dz * dz;
      if (squared_distance < closest_distance &&
          squared_distance <= squared_distance_cutoff)
      {
        *reference = world_landmark;
        closest_distance = squared_distance;
        success = true;
      }
    }
    
  }
  return success;
}

// Gets the current pose of the given landmark relative to the given reference landmark.
void ConditionGenerator::GetDisplacementVector(const msgs::Landmark& landmark,
                                               const msgs::Landmark& reference,
                                               msgs::Condition* condition)
{
  // Get transform from landmark to reference
  transform_graph::Graph graph;

  // Get transform of given landmark relative to base
  tf::StampedTransform transform;
  std::string landmark_frame(landmark.pose_stamped.header.frame_id);
  if (landmark_frame != robot_config_.base_link())
  {
    ROS_WARN("Given landmark not in base frame.");
  }
  graph.Add("landmark", transform_graph::RefFrame(landmark_frame),
            landmark.pose_stamped.pose);

  // Add frame for referenced landmark
  std::string reference_frame(reference.pose_stamped.header.frame_id);
  if (reference_frame != robot_config_.base_link())
  {
    ROS_WARN("Referenced landmark not in base frame.");
  }
  graph.Add("reference", transform_graph::RefFrame(reference_frame),
            reference.pose_stamped.pose);

  // Get 6DOF displacement vector (x, y, z, θx, θy, θz) from transform
  transform_graph::Transform landmark_in_reference;
  bool success = graph.ComputeDescription(
      transform_graph::LocalFrame("reference"),
      transform_graph::RefFrame("landmark"), &landmark_in_reference);
  if (!success)
  {
    ROS_ERROR("Unable to transform given landmark pose into referenced landmark!");
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
    

/*   ROS_INFO("Displacement vector position = (%f,%f,%f), orientation = (%f,%f,%f,%f)",
           condition->contDisplacement.x, condition->contDisplacement.y, condition->contDisplacement.z,
           condition->contDisplacement.orientation.x, condition->contDisplacement.orientation.y, 
           condition->contDisplacement.orientation.z, condition->contDisplacement.orientation.w); */
}

void ConditionGenerator::GetSpatialRelation(msgs::Condition* condition)
{
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
  if ( width < min_dim/2.0 && length < max_dim/2.0)
  {
    
    ROS_INFO("1 checked condition:%f < %f", 
      width, min_dim/2.0);
    ROS_INFO("2 checked condition:%f < %f", 
      length, max_dim/2.0); 
    // above or inside
    if (fabs(dz) < dims.z / 2.0)
    {
    ROS_INFO("check condition:%f < %f", 
       fabs(dz),dims.z/2.0); 
      spatial_relation = aboveSpatialRelation;
    }
    else
    {
      spatial_relation = insideSpatialRelation;
    }
  }
  else
  {
    // Object is 'around' (left/right/front/behind)
    if (fabs(dx) < fabs(dy))
    {
      // left or right
      if (dy < 0)
      {
        spatial_relation = leftSpatialRelation;
      }
      else
      {
        spatial_relation = rightSpatialRelation;
      }
    }
    else
    {
      // behind/front - assuming y-axis is towards the robot
      if (dx < 0)
      {
        spatial_relation = behindSpatialRelation;
      }
      else
      {
        spatial_relation = frontSpatialRelation;
      }
    }
  }
/*   ROS_INFO("%s has dimensions %f, %f, %f", 
    condition->landmark.name.c_str(), dims.x, dims.y, dims.z); */

  ROS_INFO("%s %s %s\n", 
    condition->landmark.name.c_str(), spatial_relation.c_str(),
    condition->reference.name.c_str());
  condition->spatial_relation = spatial_relation;
}

} // namespace pbd
} // namespace rapid
