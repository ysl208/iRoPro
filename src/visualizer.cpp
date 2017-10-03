#include "rapid_pbd/visualizer.h"

#include <cmath> // sqrt
#include <map>
#include <sstream>
#include <iostream>
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
                                          const msgs::Condition& condition){
  if (step_vizs_.find(program_id) == step_vizs_.end()) {
    Publish(program_id, world);
  }
  MarkerArray scene_markers;
  GetConditionMarker(condition, robot_config_, &scene_markers);

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

void GetConditionMarker(const msgs::Condition& condition,
                           const RobotConfig& robot_config,
                           visualization_msgs::MarkerArray* scene_markers) {
  ROS_INFO("Get condition markers...");
    std::string base_link(robot_config.base_link());
    if (condition.positionRelevant){

      Marker cylinder; // describes the main object's allowed positions
      cylinder.header.frame_id = base_link;
      cylinder.type = Marker::CUBE;
      cylinder.ns = "displacement_cylinder";
      geometry_msgs::Pose pose;
      pose.position = condition.position;

      geometry_msgs::Quaternion quat;
      quat = tf::createQuaternionMsgFromRollPitchYaw(condition.eulerAngles.x, 
                            condition.eulerAngles.y, condition.eulerAngles.z);
      pose.orientation = quat;
      cylinder.pose = pose;
      cylinder.pose.position.z = 0.7072; // same as table height
      cylinder.scale.x = condition.surface_box_dims.x + condition.positionVariance.x;//2*fabs(condition.contDisplacement.x+condition.contDisplacementVariance.x);
      cylinder.scale.y = condition.surface_box_dims.y + condition.positionVariance.y;//2*fabs(condition.contDisplacement.y+condition.contDisplacementVariance.y); // head diameter
      cylinder.scale.z = 0.005; // head length

      cylinder.color.r = 0;
      cylinder.color.g = 0;
      cylinder.color.b = 1;
      cylinder.color.a = 1;
      scene_markers->markers.push_back(cylinder);
    }
    if (condition.referenceRelevant && condition.reference.name != "") {
      ROS_INFO("Reference with name: %s", condition.reference.name.c_str());
      std::cout << condition.referenceRelevant << std::endl;

      Marker arrow; // displacement arrow
      arrow.type = Marker::ARROW;
      arrow.header.frame_id = base_link;
      arrow.ns = "displacement_arrow";
      geometry_msgs::Point displacement;
      displacement.x = condition.position.x + condition.contDisplacement.x;
      displacement.y = condition.position.y + condition.contDisplacement.y;
      displacement.z = condition.position.z + condition.contDisplacement.z;
      arrow.points.push_back(condition.position);
      arrow.points.push_back(displacement);
      
      arrow.scale.x = 0.005; // shaft diameter
      arrow.scale.y = 0.02; // head diameter
      arrow.scale.z = 0.02; // head length
      arrow.color.r = 1;
      arrow.color.g = 0;
      arrow.color.b = 0;
      arrow.color.a = 1;
      scene_markers->markers.push_back(arrow);

    }
    if (condition.contDisplacementRelevant){

      Marker line_strip;
      line_strip.header.frame_id = base_link;
      line_strip.type = Marker::LINE_STRIP;
      line_strip.ns = "displacement_line_strip";

      geometry_msgs::Point x_bound1;
      x_bound1.x = condition.position.x + condition.contDisplacement.x - condition.contDisplacementVariance.x;
      x_bound1.y = condition.position.y + condition.contDisplacement.y;
      x_bound1.z = condition.position.z + condition.contDisplacement.z;

      geometry_msgs::Point x_bound2;
      x_bound2.x = condition.position.x + condition.contDisplacement.x + condition.contDisplacementVariance.x;
      x_bound2.y = condition.position.y + condition.contDisplacement.y;
      x_bound2.z = condition.position.z + condition.contDisplacement.z;

      geometry_msgs::Point y_bound1;
      y_bound1.x = condition.position.x + condition.contDisplacement.x;
      y_bound1.y = condition.position.y + condition.contDisplacement.y - condition.contDisplacementVariance.y;
      y_bound1.z = condition.position.z + condition.contDisplacement.z;

      geometry_msgs::Point y_bound2;
      y_bound2.x = condition.position.x + condition.contDisplacement.x;
      y_bound2.y = condition.position.y + condition.contDisplacement.y + condition.contDisplacementVariance.y;
      y_bound2.z = condition.position.z + condition.contDisplacement.z;

      line_strip.scale.x = 0.01; // width of line segments
      line_strip.color.r = 1;
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

      Marker reference; // describes the main object's allowed positions
      reference.header.frame_id = base_link;
      reference.type = Marker::CUBE;
      reference.ns = "displacement_cube";
      reference.pose.orientation = condition.reference.pose_stamped.pose.orientation;
      reference.pose.position = condition.reference.pose_stamped.pose.position;
      reference.pose.position.z -= condition.reference.surface_box_dims.z/2; // same as table height
      ROS_INFO("Marker z-position set to : %f", reference.pose.position.z);
      reference.scale.x = condition.reference.surface_box_dims.x + condition.contDisplacementVariance.x;//2*fabs(condition.contDisplacement.x+condition.contDisplacementVariance.x);
      reference.scale.y = condition.reference.surface_box_dims.y + condition.contDisplacementVariance.y;//2*fabs(condition.contDisplacement.y+condition.contDisplacementVariance.y); // head diameter
      reference.scale.z = 0.005; // head length

      reference.color.r = 0.5;
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
}  // namespace pbd
}  // namespace rapid
