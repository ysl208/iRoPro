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
#include "transform_graph/graph.h"

#include "ros/ros.h"

namespace msgs = rapid_pbd_msgs;

namespace rapid {
namespace pbd {
SpecInference::SpecInference(const RobotConfig& robot_config)
    : robot_config_(robot_config) {}

void SpecInference::Init() {
  flag1D = true;
  distance_cutoff = 0.075;
  priors_.clear();
  posteriors_.clear();
  specs_.clear();

  avg_dx = 0.0;
  avg_dy = 0.0;
  allowedVariance = 0.03;
  max_rows_ = 3;
  min_rows_ = 1;

  for (size_t i = 0; i < 5; ++i) {
    priors_.push_back(0.2);
    posteriors_.push_back(0.20);
  }
}
void SpecInference::InitSpec(msgs::Specification* spec) {
  spec->avg_dx = avg_dx;
  spec->avg_dy = avg_dy;
  spec->row_num = max_rows_;
  spec->col_num = max_rows_;
  spec->height_num = min_rows_;
  spec->flag1D = flag1D;
  spec->offset.x = spec->offset.x * spec->avg_dx * 0.5;
  spec->offset.y = spec->offset.y * spec->avg_dy * 0.5;
}

void SpecInference::GetOffset(const msgs::Specification& spec,
                              geometry_msgs::Vector3* offset) {
  offset->x = offset->x * spec.avg_dx * 0.5;
  offset->y = offset->y * spec.avg_dy * 0.5;
}

void SpecInference::InitSpecs(std::vector<msgs::Specification>* specs,
                              const msgs::Landmark& landmark) {
  // Given the surface and the first object placed, we can initialise the area
  // for this specification
  Init();
  msgs::Specification spec;
  spec.landmark = landmark;
  spec.avg_dx =
      fmin(landmark.surface_box_dims.x, landmark.surface_box_dims.y) + 0.02;
  spec.avg_dy =
      fmax(landmark.surface_box_dims.x, landmark.surface_box_dims.y) + 0.02;
  spec.obj_num = 10;
  spec.height_num = min_rows_;
  spec.offset.x = 0;
  spec.offset.y = 0;
  spec.flag1D = true;

  spec.name = "Spec 1";
  spec.row_num = min_rows_;
  spec.col_num = max_rows_;
  specs->push_back(spec);

  spec.name = "Spec 2";
  spec.row_num = max_rows_;
  spec.col_num = min_rows_;
  specs->push_back(spec);

  spec.flag1D = false;
  spec.name = "Spec 3";
  spec.row_num = max_rows_;
  spec.col_num = max_rows_;
  specs->push_back(spec);

  spec.name = "Spec 4";
  spec.row_num = max_rows_;
  spec.col_num = max_rows_;
  spec.offset.y = spec.avg_dy * 0.5;
  specs->push_back(spec);

  spec.name = "Spec 5";
  spec.row_num = max_rows_;
  spec.col_num = max_rows_;
  spec.offset.x = spec.avg_dx * 0.5;
  spec.offset.y = 0;
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
    if (landmark.name != world_landmark.name &&
        SimilarSized(world_landmark, landmark)) {
      geometry_msgs::Point world_pose;
      world_pose.x = world_landmark.pose_stamped.pose.position.x;
      world_pose.y = world_landmark.pose_stamped.pose.position.y;
      world_pose.z = world_landmark.pose_stamped.pose.position.z;
      double dx = world_pose.x - landmark.pose_stamped.pose.position.x;
      double dy = world_pose.y - landmark.pose_stamped.pose.position.y;
      double dz = world_pose.z - landmark.pose_stamped.pose.position.z;
      double squared_distance = dx * dx + dy * dy + dz * dz;

      std::cout << "Checking distance to " << world_landmark.name << " is "
                << squared_distance << "\n";
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

bool SpecInference::SimilarSized(const msgs::Landmark& landmark1,
                                 const msgs::Landmark& landmark2) {
  double variance = 0.075;
  const double kMaxDistance = variance * variance;

  double dx = (landmark1.surface_box_dims.x - landmark2.surface_box_dims.x);
  double dy = (landmark1.surface_box_dims.y - landmark2.surface_box_dims.y);
  double dz = (landmark1.surface_box_dims.z - landmark2.surface_box_dims.z);
  double distance = dx * dx + dy * dy + dz * dz;
  std::cout << "landmarks distance is " << distance << "\n";
  return distance <= kMaxDistance;
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
                                     std::vector<float>* posteriors,
                                     msgs::Specification* spec) {
  // 5 specifications (s6 can be random)
  std::vector<float> pOfD, priors;
  std::vector<float> new_posteriors;
  float dx, dy;

  // initialise pOfD for all s with 1
  std::cout << "New priors: \n";
  for (size_t i = 0; i < posteriors->size(); ++i) {
    pOfD.push_back(1.0);
    priors.push_back(posteriors->at(i));
    std::cout << " P(s" << i + 1 << ") = " << priors[i] << " \n";
  }
  // find closest landmark that can be referenced
  // double squared_cutoff = distance_cutoff * distance_cutoff;
  msgs::Landmark closest;
  std::cout << "** Updating Posteriors : " << landmark.name << " \n";
  if (ReferencedLandmark(landmark, world, distance_cutoff, &closest)) {
    std::cout << " closest to " << closest.name << "\n";
    // calculate dx, dy
    dx = fabs(closest.pose_stamped.pose.position.x -
              landmark.pose_stamped.pose.position.x);
    dy = fabs(closest.pose_stamped.pose.position.y -
              landmark.pose_stamped.pose.position.y);

    ROS_INFO("dx = %f, dy = %f", dx, dy);
    std::cout << "avg_dx,dy was " << spec->avg_dx << ", " << spec->avg_dy;
    spec->avg_dx = dx;
    spec->avg_dy = dy;
    // spec->avg_dx = (spec->avg_dx + dx) / world.surface_box_landmarks.size();
    // spec->avg_dy = (spec->avg_dy + dy) / world.surface_box_landmarks.size();
    std::cout << " is now " << spec->avg_dx << ", " << spec->avg_dy << "\n";

    // check s_n conditions
    if (flag1D) {
      if (dx > allowedVariance) {
        // x-aligned
        pOfD.at(GetPatternIndex("s1")) = 0.0;
        std::cout << "dx " << dx << " > allowedVar " << allowedVariance << "\n";
      }
      if (dy > allowedVariance) {
        // y-aligned
        pOfD.at(GetPatternIndex("s2")) = 0.0;
        std::cout << "dy " << dy << " > allowedVar " << allowedVariance << "\n";
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
    // s3 s4 s5: check that the object is not aligned
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

    UpdatePriors(pOfD, priors, &new_posteriors);

    posteriors->clear();
    std::cout << "New posteriors are: \n";
    for (size_t i = 0; i < new_posteriors.size(); ++i) {
      posteriors->push_back(new_posteriors[i]);
      std::cout << " P(s" << i + 1 << "|D) = " << posteriors->at(i) << " \n";
    }
  } else {
    std::cout << " No closest landmark found with sqred cutoff "
              << distance_cutoff << "\n";
  }
}
void SpecInference::UpdatePriors(const std::vector<float>& pOfD,
                                 const std::vector<float>& priors,
                                 std::vector<float>* posteriors) {
  // calculate sum of all pOfD

  std::cout << "Updating priors... \n";
  float sum = 0;
  for (size_t key = 0; key < pOfD.size(); ++key) {
    sum += pOfD[key] * priors[key];
  }
  for (size_t key = 0; key < priors.size(); ++key) {
    float posterior = (priors[key] * pOfD[key]) / sum;
    // posteriors->at(key) = posterior;
    posteriors->push_back(posterior);
  }
}
void SpecInference::GenerateStacks(
    const msgs::Specification& spec, const msgs::Surface& surface,
    std::vector<geometry_msgs::PoseArray>* grid) {
  // Given a grid with positions, generate height_num stacks of these initial
  // positions
  // for (size_t row = 0; row < grid.size(); ++row) {
  //   for (size_t col = 0; col < grid[row].poses.size(); ++col) {
  //     geometry_msgs::Pose pose = grid[row].poses[col];
  //   }
  // }
}
void SpecInference::GenerateGrid(const msgs::Specification& spec,
                                 const msgs::Surface& surface,
                                 std::vector<geometry_msgs::PoseArray>* grid) {
  // Given the number of rows/cols, the starting landmark, an avg inter-object
  // distance, and a surface
  // generate obj_num positions on the surface
  // Note: assume that starting landmark is on top right corner of the table
  std::cout << "Generate Grid \n";
  geometry_msgs::PoseArray pose_array;
  std::vector<geometry_msgs::Pose> positions;

  geometry_msgs::Vector3 obj_distance;
  obj_distance.x = spec.avg_dx;
  obj_distance.y = spec.avg_dy;
  obj_distance.z = spec.landmark.surface_box_dims.z;

  std::cout << "obj_distance = " << obj_distance.x << "," << obj_distance.y
            << "\n";
  // set corners of the allowed area
  geometry_msgs::Point min_pos = spec.landmark.pose_stamped.pose.position;
  std::cout << "min_pos = " << min_pos.x << "," << min_pos.y << "\n";
  geometry_msgs::Point max_pos, num_based, surface_based;
  // Fit the number of rows/cols required and stop at the surface border
  // Note: using minus operator for x because we assume that start object is top
  // right
  num_based.x = min_pos.x - (spec.row_num - 1) * obj_distance.x;
  num_based.y = min_pos.y + (spec.col_num - 1) * obj_distance.y;
  std::cout << "num_based.x = " << num_based.x << " = " << min_pos.x << " - ("
            << spec.row_num << "-1)*" << obj_distance.x << "\n";
  std::cout << "num_based.y = " << num_based.y << " = " << min_pos.y << " - ("
            << spec.col_num << "-1)*" << obj_distance.y << "\n";

  surface_based.x =
      surface.pose_stamped.pose.position.x - surface.dimensions.x * 0.5;
  surface_based.y =
      surface.pose_stamped.pose.position.y + surface.dimensions.y * 0.5;
  std::cout << "surface_based.x = " << surface_based.x << " = "
            << surface.pose_stamped.pose.position.x << " - "
            << surface.dimensions.x * 0.5 << "\n";
  std::cout << "surface_based.y = " << surface_based.y << " = "
            << surface.pose_stamped.pose.position.y << " - "
            << surface.dimensions.y * 0.5 << "\n";

  // set max_pos as mininum of either end of the border or rows/cols defined
  max_pos.x = fmax(num_based.x, surface_based.x);
  max_pos.y = fmin(num_based.y, surface_based.y);
  std::cout << "max_pos.x = fmax( " << num_based.x << "," << surface_based.x
            << "\n";
  std::cout << "max_pos.y = fmin( " << num_based.y << "," << surface_based.y
            << "\n";
  max_pos.z =
      surface.pose_stamped.pose.position.z + spec.landmark.surface_box_dims.z;

  std::cout << "GetPositions with \n"
            << "min_pos:" << min_pos << ", max_pos:" << max_pos
            << ", lm dims: " << spec.landmark.surface_box_dims
            << ", obj_dist: " << obj_distance << "\n";
  GetPositions(min_pos, max_pos, spec.offset,
               spec.landmark.pose_stamped.pose.orientation, obj_distance,
               &positions, spec.obj_num);

  pose_array.poses = positions;
  grid->push_back(pose_array);
}

void SpecInference::GetPositions(const geometry_msgs::Point& min_pos,
                                 const geometry_msgs::Point& max_pos,
                                 const geometry_msgs::Vector3& offset,
                                 const geometry_msgs::Quaternion& orientation,
                                 const geometry_msgs::Vector3& obj_distance,
                                 std::vector<geometry_msgs::Pose>* positions,
                                 const int& obj_num) {
  // Takes the corner values of the area which should be filled with positions
  geometry_msgs::Pose pose;
  pose.orientation = orientation;
  pose.position.z = min_pos.z;
  geometry_msgs::Point local_min = min_pos;
  geometry_msgs::Point local_max = max_pos;

  bool evenRow = false;
  bool evenCol = false;

  while (local_min.x >= max_pos.x - 0.001) {
    pose.position.x = local_min.x;
    if (evenRow) {
      local_min.y += offset.y;
      evenRow = false;
    } else {
      evenRow = true;
    }

    while (local_min.y <= max_pos.y + 0.001) {
      pose.position.y = local_min.y;
      std::cout << "x y : " << pose.position.x << " , " << pose.position.y
                << "\n";

      if (evenCol) {
        pose.position.x -= offset.x;
        evenCol = false;
      } else {
        evenCol = true;
        pose.position.x = local_min.x;
      }

      positions->push_back(pose);

      local_min.y += obj_distance.y;
      std::cout << "local_min.y = " << local_min.y << "\n";
      if (positions->size() >= obj_num) {
        return;
      }
    }
    evenCol = false;
    local_min.y = min_pos.y;  // reset y for next iteration
    local_min.x -= obj_distance.x;
  }

  std::cout << "No. of positions: " << positions->size() << " out of "
            << obj_num << "\n";
}

}  // namespace pbd
}  // namespace rapid
