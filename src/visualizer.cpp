#include "rapid_pbd/visualizer.h"

#include <cmath>  // sqrt
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/TransformStamped.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "rapid_pbd/joint_state.h"
#include "rapid_pbd_msgs/EditorEvent.h"
#include "rapid_pbd_msgs/Landmark.h"
#include "rapid_pbd_msgs/Program.h"
#include "rapid_pbd_msgs/Surface.h"
#include "robot_markers/builder.h"
#include "sensor_msgs/PointCloud2.h"
#include "surface_perception/visualization.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

#include "rapid_pbd/robot_config.h"
#include "rapid_pbd/world.h"
#include "ros/ros.h"

namespace msgs = rapid_pbd_msgs;
using sensor_msgs::PointCloud2;
using visualization_msgs::Marker;
using visualization_msgs::MarkerArray;

namespace rapid {
namespace pbd {
Visualizer::Visualizer(const SceneDb& scene_db,
                       const robot_markers::Builder& marker_builder,
                       const RobotConfig& robot_config)
    : scene_db_(scene_db),
      marker_builder_(marker_builder),
      robot_config_(robot_config),
      step_vizs_(),
      nh_() {}

void Visualizer::Init() {
  marker_builder_.Init();
  marker_builder_.SetNamespace("robot");
  marker_builder_.SetFrameId(robot_config_.base_link());
}

void Visualizer::Publish(const std::string& program_id, const World& world) {
  CreateStepVizIfNotExists(program_id);

  // Publish the robot visualization
  MarkerArray robot_markers;
  std::map<std::string, double> joint_positions;
  world.joint_state.ToMap(&joint_positions);
  marker_builder_.SetJointPositions(joint_positions);
  marker_builder_.Build(&robot_markers);
  step_vizs_[program_id].robot_pub.publish(robot_markers);

  std::string base_link(robot_config_.base_link());

  // Publish the scene
  PointCloud2 scene;
  if (world.scene_id != "" && scene_db_.Get(world.scene_id, &scene)) {
    if (world.scene_id != step_vizs_[program_id].last_scene_id) {
      step_vizs_[program_id].scene_pub.publish(scene);
    }
  } else {
    pcl::PointCloud<pcl::PointXYZRGB> blank;
    pcl::PointXYZRGB pt;
    blank.points.push_back(pt);
    pcl::toROSMsg(blank, scene);
    scene.header.frame_id = base_link;
    step_vizs_[program_id].scene_pub.publish(scene);
  }
  step_vizs_[program_id].last_scene_id = world.scene_id;

  // Publish landmark markers
  MarkerArray scene_markers;
  GetSegmentationMarker(world.surface_box_landmarks, robot_config_,
                        &scene_markers);
  GetSurfaceMarker(world.surface, robot_config_, &scene_markers);
  if (scene_markers.markers.size() > 0) {
    step_vizs_[program_id].surface_seg_pub.publish(scene_markers);
  } else {
    for (size_t i = 0; i < 100; ++i) {
      Marker blank;
      blank.ns = "segmentation";
      blank.id = i;
      blank.header.frame_id = base_link;
      blank.type = Marker::CUBE;
      blank.pose.orientation.w = 1;
      blank.scale.x = 0.05;
      blank.scale.y = 0.05;
      blank.scale.z = 0.05;
      scene_markers.markers.push_back(blank);
    }
    step_vizs_[program_id].surface_seg_pub.publish(scene_markers);
  }
}

void Visualizer::StopPublishing(const std::string& program_id) {
  if (step_vizs_.find(program_id) != step_vizs_.end()) {
    step_vizs_.erase(program_id);
  }
}

void Visualizer::PublishConditionMarkers(const std::string& program_id,
                                         const World& world,
                                         const msgs::Condition& condition) {
  if (step_vizs_.find(program_id) == step_vizs_.end()) {
    Publish(program_id, world);
  }
  MarkerArray scene_markers;
  GetConditionMarker(condition, robot_config_, &scene_markers);
  GetGridMarker(world.surface, condition, robot_config_, &scene_markers);
  if (scene_markers.markers.size() > 0) {
    step_vizs_[program_id].surface_seg_pub.publish(scene_markers);
  } else {
    ROS_INFO("No markers to publish");
  }
}

void Visualizer::CreateStepVizIfNotExists(const std::string& program_id) {
  // Create the publisher if it doesn't exist.
  if (step_vizs_.find(program_id) == step_vizs_.end()) {
    step_vizs_[program_id].robot_pub =
        nh_.advertise<MarkerArray>("robot/" + program_id, 10, true);
    step_vizs_[program_id].scene_pub =
        nh_.advertise<PointCloud2>("scene/" + program_id, 10, true);
    step_vizs_[program_id].surface_seg_pub = nh_.advertise<MarkerArray>(
        "surface_segmentation/" + program_id, 10, true);
    step_vizs_[program_id].last_scene_id = "";
  }
}

RuntimeVisualizer::RuntimeVisualizer(const RobotConfig& robot_config,
                                     const ros::Publisher& surface_box_pub)
    : robot_config_(robot_config), surface_box_pub_(surface_box_pub) {}

void RuntimeVisualizer::PublishSurfaceBoxes(
    const std::vector<msgs::Landmark>& box_landmarks) const {
  MarkerArray scene_markers;
  GetSegmentationMarker(box_landmarks, robot_config_, &scene_markers);
  surface_box_pub_.publish(scene_markers);
}

void GetGridMarker(const msgs::Surface& surface,
                   const msgs::Condition& condition,
                   const RobotConfig& robot_config,
                   visualization_msgs::MarkerArray* scene_markers) {
  Marker points;  // describes the main object's allowed position
  std::string base_link(robot_config.base_link());
  points.header.frame_id = base_link;
  points.type = Marker::POINTS;
  points.ns = "grid";
  geometry_msgs::Pose pose;

  // msgs::Surface surface = surfaces[0];
  pose.position = surface.pose_stamped.pose.position;
  pose.orientation = surface.pose_stamped.pose.orientation;
  points.pose = pose;
  points.pose.position.z += 0.02;

  points.scale.x = 0.02;  // surface.dimensions.x;
  points.scale.y = 0.02;  // surface.dimensions.y;
  points.scale.z = 0.02;

  points.color.b = 1.0;
  points.color.a = 1.0;

  // for (size_t i = 0; i < condition.grid.size(), ++i) {
  //   for (size_t j = 0; j < condition.grid.size(), ++j) {
  // geometry_msgs::Point p = condition.grid[i][j];
  geometry_msgs::Point p;
  p.x = 0.821929;
  p.y - -0.113207;
  p.z = 0.501502 + 0.02;
  points.points.push_back(p);
  p.x = 0.758341;
  points.points.push_back(p);
  p.x = 0.694752;
  points.points.push_back(p);
  p.x = 0.631163;
  points.points.push_back(p);
  //   }
  // }
  scene_markers->markers.push_back(points);
}

void GetConditionMarker(const msgs::Condition& condition,
                        const RobotConfig& robot_config,
                        visualization_msgs::MarkerArray* scene_markers) {
  ROS_INFO("Get condition markers...");
  std::string base_link(robot_config.base_link());
  if (condition.positionRelevant) {
    Marker cylinder;  // describes the main object's allowed position
    cylinder.header.frame_id = base_link;
    cylinder.type = Marker::CUBE;
    cylinder.ns = "displacement_cylinder";
    geometry_msgs::Pose pose;
    pose.position = condition.position;

    geometry_msgs::Quaternion quat;
    quat = tf::createQuaternionMsgFromRollPitchYaw(condition.eulerAngles.x,
                                                   condition.eulerAngles.y,
                                                   condition.eulerAngles.z);
    pose.orientation =
        condition.surface.pose_stamped.pose.orientation;  // quat;
    cylinder.pose = pose;
    cylinder.pose.position.x =
        condition.min_pos.x + (condition.max_pos.x - condition.min_pos.x) * 0.5;
    cylinder.pose.position.y =
        condition.min_pos.y + (condition.max_pos.y - condition.min_pos.y) * 0.5;
    cylinder.pose.position.z -=
        condition.surface_box_dims.z / 2;  // table height
    cylinder.scale.x = fmax(condition.surface_box_dims.x + 0.01,
                            condition.max_pos.x - condition.min_pos.x);
    ROS_INFO("max min x: %f %f", condition.max_pos.x, condition.min_pos.x);

    cylinder.scale.y = fmax(condition.surface_box_dims.y + 0.01,
                            condition.max_pos.y - condition.min_pos.y);
    ROS_INFO("max min y: %f %f", condition.max_pos.y, condition.min_pos.y);
    ROS_INFO("scale x y: %f %f", cylinder.scale.x, cylinder.scale.y);

    cylinder.scale.z = 0.005;  // head length

    cylinder.color.r = 0;  // violet
    cylinder.color.g = 0;
    cylinder.color.b = 1;
    cylinder.color.a = 1;
    scene_markers->markers.push_back(cylinder);
  }
  if (condition.contDisplacementRelevant) {
    // if (condition.referenceRelevant && condition.reference.name != "") {
    Marker arrow;  // displacement arrow
    arrow.type = Marker::ARROW;
    arrow.header.frame_id = base_link;
    arrow.ns = "displacement_arrow";
    geometry_msgs::Point displacement;
    displacement.x = condition.position.x + condition.contDisplacement.x;
    displacement.y = condition.position.y + condition.contDisplacement.y;
    displacement.z = condition.position.z + condition.contDisplacement.z;
    arrow.points.push_back(condition.position);
    arrow.points.push_back(displacement);

    arrow.scale.x = 0.005;  // shaft diameter
    arrow.scale.y = 0.02;   // head diameter
    arrow.scale.z = 0.02;   // head length
    arrow.color.r = 1;      // red
    arrow.color.g = 0;
    arrow.color.b = 0;
    arrow.color.a = 1;
    scene_markers->markers.push_back(arrow);

    // three lines indicating the allowed variances
    Marker line_strip;
    line_strip.header.frame_id = base_link;
    line_strip.type = Marker::LINE_STRIP;
    line_strip.ns = "displacement_line_strip";

    geometry_msgs::Point x_bound1;
    x_bound1.x = condition.position.x + condition.contDisplacement.x -
                 condition.contDisplacementVariance.x;
    x_bound1.y = condition.position.y + condition.contDisplacement.y;
    x_bound1.z = condition.position.z + condition.contDisplacement.z;

    geometry_msgs::Point x_bound2;
    x_bound2.x = condition.position.x + condition.contDisplacement.x +
                 condition.contDisplacementVariance.x;
    x_bound2.y = condition.position.y + condition.contDisplacement.y;
    x_bound2.z = condition.position.z + condition.contDisplacement.z;

    geometry_msgs::Point y_bound1;
    y_bound1.x = condition.position.x + condition.contDisplacement.x;
    y_bound1.y = condition.position.y + condition.contDisplacement.y -
                 condition.contDisplacementVariance.y;
    y_bound1.z = condition.position.z + condition.contDisplacement.z;

    geometry_msgs::Point y_bound2;
    y_bound2.x = condition.position.x + condition.contDisplacement.x;
    y_bound2.y = condition.position.y + condition.contDisplacement.y +
                 condition.contDisplacementVariance.y;
    y_bound2.z = condition.position.z + condition.contDisplacement.z;

    line_strip.scale.x = 0.01;  // width of line segments
    line_strip.color.r = 1;     // red
    line_strip.color.g = 0;
    line_strip.color.b = 0;
    line_strip.color.a = 1;
    line_strip.points.push_back(condition.position);
    line_strip.points.push_back(x_bound2);
    line_strip.points.push_back(condition.position);
    line_strip.points.push_back(y_bound2);
    line_strip.points.push_back(condition.position);
    line_strip.points.push_back(x_bound1);
    line_strip.points.push_back(condition.position);
    line_strip.points.push_back(y_bound1);
    line_strip.points.push_back(condition.position);
    scene_markers->markers.push_back(line_strip);

    Marker reference;  // describes the reference's allowed positions
    reference.header.frame_id = base_link;
    reference.type = Marker::CUBE;
    reference.ns = "displacement_cube";
    reference.pose.orientation =
        condition.reference.pose_stamped.pose.orientation;
    reference.pose.position = condition.reference.pose_stamped.pose.position;
    reference.pose.position.z -=
        condition.reference.surface_box_dims.z / 2;  // same as table height
    reference.scale.x = condition.reference.surface_box_dims.x +
                        condition.contDisplacementVariance.x;
    reference.scale.y = condition.reference.surface_box_dims.y +
                        condition.contDisplacementVariance.y;
    // // head diameter
    reference.scale.z = 0.005;  // head length

    reference.color.r = 0.5;  // pink
    reference.color.g = 0;
    reference.color.b = 1;
    reference.color.a = 1;
    scene_markers->markers.push_back(reference);
  }
}

void GetSegmentationMarker(const std::vector<msgs::Landmark>& landmarks,
                           const RobotConfig& robot_config,
                           visualization_msgs::MarkerArray* scene_markers) {
  std::vector<surface_perception::Object> objects;
  for (size_t li = 0; li < landmarks.size(); ++li) {
    const msgs::Landmark& landmark = landmarks[li];
    surface_perception::Object object;
    object.pose_stamped = landmark.pose_stamped;
    object.dimensions = landmark.surface_box_dims;
    objects.push_back(object);
  }
  surface_perception::ObjectMarkers(objects, &scene_markers->markers);

  std::string base_link(robot_config.base_link());

  for (size_t i = 0; i < objects.size(); ++i) {
    scene_markers->markers[i].ns = "segmentation";
    scene_markers->markers[i].id = i;
  }
  for (size_t i = 0; i < objects.size(); ++i) {
    Marker marker = scene_markers->markers[i];
    marker.type = Marker::TEXT_VIEW_FACING;
    marker.ns = "segmentation_names";
    marker.text = landmarks[i].name;
    marker.pose.position.z += 0.15;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.r = 1;
    marker.color.g = 0;
    marker.color.b = 0;
    marker.color.a = 1;
    scene_markers->markers.push_back(marker);
  }
  int num_objects = objects.size();
  for (size_t i = num_objects; i < 100; ++i) {
    Marker blank;
    blank.ns = "segmentation";
    blank.id = i;
    blank.header.frame_id = base_link;
    blank.type = Marker::CUBE;
    blank.pose.orientation.w = 1;
    blank.scale.x = 0.05;
    blank.scale.y = 0.05;
    blank.scale.z = 0.05;
    scene_markers->markers.push_back(blank);

    blank.ns = "segmentation_names";
    scene_markers->markers.push_back(blank);
  }
}

void GetSurfaceMarker(const msgs::Surface& surface,
                      const RobotConfig& robot_config,
                      visualization_msgs::MarkerArray* scene_markers) {
  std::string base_link(robot_config.base_link());
  if (surface.dimensions.x <= 0) {
    ROS_INFO("No surface to publish");
  }
  Marker table;
  table.header.frame_id = base_link;
  table.type = Marker::CUBE;
  table.ns = "table";
  geometry_msgs::Pose pose;
  pose.position = surface.pose_stamped.pose.position;
  pose.orientation = surface.pose_stamped.pose.orientation;
  table.pose = pose;
  table.pose.position.z -= surface.dimensions.z;
  table.scale.x = surface.dimensions.x;
  table.scale.y = surface.dimensions.y;
  table.scale.z = surface.dimensions.z;

  table.color.r = 0.01;
  table.color.g = 0.01;
  table.color.b = 0.01;
  table.color.a = 1;
  scene_markers->markers.push_back(table);
}

}  // namespace pbd
}  // namespace rapid
