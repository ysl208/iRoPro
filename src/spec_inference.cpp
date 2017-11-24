//  Created by Ying Siu Sheryl Liang on 28/07/2017.
//
//

#include "rapid_pbd/spec_inference.h"

#include <tf/transform_datatypes.h>
#include <algorithm>
#include <cmath>
#include <exception>
#include <string>
#include <vector>

#include "rapid_pbd/robot_config.h"
#include "rapid_pbd/spatial_relations.h"
#include "transform_graph/graph.h"

#include "ros/ros.h"

namespace msgs = rapid_pbd_msgs;

namespace rapid {
namespace pbd {
SpecInference::SpecInference(const RobotConfig& robot_config)
    : robot_config_(robot_config) {}

void SpecInference::Init(){
  flag1D = true;
  distance_cutoff = 1.0;
  priors_.clear();
  posteriors_.clear();

  avg_dx = 0.0;
  avg_dy = 0.0;
  allowedVariance = 0.03;

  for (size_t i = 0; i < 5; ++i) {
    priors_.push_back(0.2);
    posteriors_.push_back(0.0);
  }

}

bool SpecInference::ReferencedLandmark(const msgs::Landmark& landmark,
                                       const World& world,
                                       const double squared_distance_cutoff,
                                       msgs::Landmark* reference) {
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

int SpecInference::GetPatternIndex(const std::string& s) {
  if (s == "s1") {
    // all x-coordinates are within a range of allowedVariance
    // all y-coordinates need to be > allowedVariance
    // assume allowedVariance < landmark.dims
    return 0;
  } else if (s == "s2") {
    // y-aligned
    return 1;
  } else if (s == "s3") {
    return 2;
  } else if (s == "s4") {
    return 3;
  } else if (s == "s5") {
    return 4;
    // } else if (s == "s6") {
    // s6 for random or user-specified
    //   return 5;
  } else {
    ROS_ERROR("Unknown pattern name");
    return -1;
  }
}

void SpecInference::UpdatePosteriors(const World& world,
                                     const msgs::Landmark& landmark) {
  // 5 specifications (s6 can be random)
  std::vector<float> pOfD;
  float dx, dy;

  // initialise pOfD
  pOfD.clear();
  for (size_t i = 0; i < 5; ++i) {
    pOfD.push_back(1.0);

    std::cout << "Prior for s" << i + 1 << " " << priors_[i] << " \n";
    std::cout << "Posteriors for s" << i + 1 << " " << posteriors_[i] << " \n";
    
  }

  // find closest landmark that can be referenced
  
  double squared_cutoff = distance_cutoff * distance_cutoff;
  msgs::Landmark closest;
  ROS_INFO("** Updating Posteriors **");
  if (ReferencedLandmark(landmark, world, squared_cutoff, &closest)) {
    // calculate dx, dy
    dx = fabs(closest.pose_stamped.pose.position.x -
              landmark.pose_stamped.pose.position.x);
    dy = fabs(closest.pose_stamped.pose.position.y -
              landmark.pose_stamped.pose.position.y);

    ROS_INFO("dx = %f, dy = %f", dx, dy);
    avg_dx = (avg_dx + dx) / world.surface_box_landmarks.size();
    avg_dy = (avg_dy + dy) / world.surface_box_landmarks.size();

    // check s_n conditions
    if (flag1D) {
      if (dx > allowedVariance) {
        // x-aligned
        pOfD.at(GetPatternIndex("s1")) = 0.0;
      }
      if (dy > allowedVariance) {
        // y-aligned
        pOfD.at(GetPatternIndex("s2")) = 0.0;
      }

      if (pOfD[GetPatternIndex("s1")] == 0.0 &&
          pOfD[GetPatternIndex("s2")] == 0.0) {
        flag1D = false;
        ROS_INFO("flag1D set to false");
      }
    } else {
      pOfD.at(GetPatternIndex("s1")) = 0.0;
      pOfD.at(GetPatternIndex("s2")) = 0.0;
    }
    // if not 1-dimensional, then check s3-s5
    // s3
    // s4 s5: check that the object is not aligned
    if (dx > allowedVariance && dy > allowedVariance) {
      // not vertically nor horizontically aligned
      pOfD.at(GetPatternIndex("s3")) = 0.0;
      // s4 are rows, s5 are columns
      if (dx < dy) {
        // s4
        pOfD.at(GetPatternIndex("s4")) = 0.0;
      }

      if (dx > dy) {
        // s5
        pOfD.at(GetPatternIndex("s5")) = 0.0;
      } else {
        // s?
      }
    }
    for (size_t i = 0; i < pOfD.size(); ++i) {
      std::cout << "PofD for s" << i + 1 << " " << pOfD[i] << " \n";
    }
    UpdatePriors(pOfD);
  }
  // add landmark to list
  // landmarks->push_back(landmark);

  for (size_t i = 0; i < posteriors_.size(); ++i) {
    std::cout << "posteriors_ for s" << i + 1 << " " << posteriors_.at(i)
              << " \n";
  }
}
void SpecInference::UpdatePriors(const std::vector<float>& pOfD) {
  // calculate sum of all pOfD
  float sum = 0;
  for (size_t key = 0; key < pOfD.size(); ++key) {
    sum += pOfD[key];
    ROS_INFO("sum = %f", sum);
  }
  for (size_t key = 0; key < priors_.size(); ++key) {
    float posterior = (priors_[key] * pOfD[key]) / sum;
    posteriors_.at(key) = posterior;
    ROS_INFO("posteriors_.at(key) = %f", posteriors_.at(key));
  }
}
}  // namespace pbd
}  // namespace rapid
