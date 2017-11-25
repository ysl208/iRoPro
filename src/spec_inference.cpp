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

void SpecInference::Init() {
  flag1D = true;
  distance_cutoff = 1.0;
  priors_.clear();
  posteriors_.clear();
  specs_.clear();

  avg_dx = 0.0;
  avg_dy = 0.0;
  allowedVariance = 0.03;

  for (size_t i = 0; i < 5; ++i) {
    priors_.push_back(0.2);
    posteriors_.push_back(1.0);
  }
}

void SpecInference::InitSpecs(std::vector<msgs::Specification>* specs,
                              const msgs::Landmark& landmark) {
  // Given the surface and the first object placed, we can initialise the area
  // for this specification
  msgs::Specification spec;
  spec.landmark = landmark;
  spec.avg_dx = landmark.surface_box_dims.x + 0.02;
  spec.avg_dy = landmark.surface_box_dims.y + 0.02;
  spec.obj_num = 100;

  spec.name = "Spec 1";
  spec.row_num = 1;
  spec.col_num = 100;
  specs->push_back(spec);

  spec.name = "Spec 2";
  spec.row_num = 100;
  spec.col_num = 1;
  specs->push_back(spec);

  spec.name = "Spec 3";
  spec.row_num = 100;
  spec.col_num = 100;
  specs->push_back(spec);

  spec.name = "Spec 4";
  spec.row_num = 100;
  spec.col_num = 100;
  specs->push_back(spec);

  spec.name = "Spec 5";
  spec.row_num = 100;
  spec.col_num = 100;
  specs->push_back(spec);
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
                                     const msgs::Landmark& landmark,
                                     std::vector<float>* posteriors) {
  // 5 specifications (s6 can be random)
  std::vector<float> pOfD;
  float dx, dy;

  // initialise pOfD
  pOfD.clear();
  for (size_t i = 0; i < 5; ++i) {
    pOfD.push_back(1.0);

    std::cout << "Prior for s" << i + 1 << " " << priors_[i] << " \n";
    std::cout << "Posteriors for s" << i + 1 << " " << &posteriors[i] << " \n";
  }

  // find closest landmark that can be referenced
  double squared_cutoff = distance_cutoff * distance_cutoff;
  msgs::Landmark closest;
  std::cout << "** Updating Posteriors : " << landmark.name;
  if (ReferencedLandmark(landmark, world, squared_cutoff, &closest)) {
    std::cout << " close to " << closest.name << "\n";
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
        std::cout << "dx " << dx << " > allVar " << allowedVariance << "\n";
      }
      if (dy > allowedVariance) {
        // y-aligned
        pOfD.at(GetPatternIndex("s2")) = 0.0;
        std::cout << "dy " << dy << " > allVar " << allowedVariance << "\n";
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
    UpdatePriors(pOfD, posteriors);
  }
  // add landmark to list
  // landmarks->push_back(landmark);

  for (size_t i = 0; i < posteriors->size(); ++i) {
    std::cout << "posteriors_ for s" << i + 1 << " " << posteriors->at(i)
              << " \n";
  }
}
void SpecInference::UpdatePriors(const std::vector<float>& pOfD,
                                 std::vector<float>* posteriors) {
  // calculate sum of all pOfD
  float sum = 0;
  for (size_t key = 0; key < pOfD.size(); ++key) {
    sum += pOfD[key] * priors_[key];
    ROS_INFO("sum is now %f", sum);
  }
  for (size_t key = 0; key < priors_.size(); ++key) {
    float posterior = (priors_[key] * pOfD[key]) / sum;
    posteriors->at(key) = posterior;
    ROS_INFO("posteriors = %f", posteriors->at(key));
  }
}

void SpecInference::GenerateGrid(const msgs::Specification& spec,
                                 const msgs::Surface& surface,
                                 std::vector<geometry_msgs::PoseArray>* grid) {
  // Given the number of rows/cols, the starting landmark, an avg inter-object
  // distance, and a surface
  // generate obj_num positions on the surface
  // Note: assume that starting landmark is on top right corner of the table

  geometry_msgs::PoseArray pose_array;
  std::vector<geometry_msgs::Pose> positions;

  geometry_msgs::Vector3 obj_distance;
  obj_distance.x = spec.avg_dx;
  obj_distance.y = spec.avg_dy;
  obj_distance.z = spec.landmark.surface_box_dims.z;

  // set corners of the allowed area
  geometry_msgs::Point min_pos = spec.landmark.pose_stamped.pose.position;
  geometry_msgs::Point max_pos, num_based, surface_based;
  // Note: using minus operator because we assume that start object is top right
  num_based.x = min_pos.x - (spec.row_num - 1) * avg_dx;
  num_based.y = min_pos.y - (spec.col_num - 1) * avg_dy;

  surface_based.x =
      surface.pose_stamped.pose.position.x - surface.dimensions.x * 0.5;
  surface_based.y =
      surface.pose_stamped.pose.position.y - surface.dimensions.y * 0.5;

  max_pos.x = fmax(num_based.x, surface_based.x);
  max_pos.y = fmax(num_based.y, surface_based.y);
  max_pos.z =
      surface.pose_stamped.pose.position.z + spec.landmark.surface_box_dims.z;

  GetPositions(min_pos, max_pos, spec.landmark.surface_box_dims, obj_distance,
               &positions, spec.obj_num);

  pose_array.poses = positions;
  grid->push_back(pose_array);
}

void SpecInference::GetPositions(const geometry_msgs::Point& min_pos,
                                 const geometry_msgs::Point& max_pos,
                                 const geometry_msgs::Vector3& dimensions,
                                 const geometry_msgs::Vector3& obj_distance,
                                 std::vector<geometry_msgs::Pose>* positions,
                                 const int& obj_num) {
  // Takes the corner values of the area which should be filled with positions

  geometry_msgs::Pose pose;
  pose.position.z = min_pos.z;
  geometry_msgs::Point local_min = min_pos;
  geometry_msgs::Point local_max = max_pos;

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
      if (positions->size() >= obj_num) {
        return;
      }
    }
    local_min.y = min_pos.y + dimensions.y;  // reset y
    local_min.x += obj_distance.x;
    std::cout << "No. of positions: " << positions->size() << " out of \n"
              << obj_num;
  }
}

}  // namespace pbd
}  // namespace rapid
