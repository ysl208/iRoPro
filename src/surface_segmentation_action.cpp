#include "rapid_pbd/surface_segmentation_action.h"

#include <algorithm>
#include <sstream>
#include <string>

#include "actionlib/server/simple_action_server.h"
#include "pcl/common/common.h"
#include "pcl/filters/crop_box.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/transforms.h"
#include "rapid_pbd/action_names.h"
#include "rapid_pbd_msgs/SegmentSurfacesAction.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "surface_perception/segmentation.h"
#include "surface_perception/surface_objects.h"
#include "surface_perception/typedefs.h"
#include "surface_perception/visualization.h"
#include "tf/transform_listener.h"
#include "visualization_msgs/Marker.h"

#include "rapid_pbd/program_db.h"
#include "rapid_pbd/robot_config.h"

using surface_perception::Object;
using surface_perception::SurfaceObjects;

namespace msgs = rapid_pbd_msgs;
namespace rapid {
namespace pbd {
SurfaceSegmentationAction::SurfaceSegmentationAction(
    const std::string& topic, const SceneDb& scene_db,
    const RobotConfig& robot_config)
    : topic_(topic),
      scene_db_(scene_db),
      robot_config_(robot_config),
      as_(kSurfaceSegmentationActionName,
          boost::bind(&SurfaceSegmentationAction::Execute, this, _1), false),
      seg_(),
      nh_(),
      marker_pub_(nh_.advertise<visualization_msgs::Marker>(
          "surface_seg/visualization", 100)),
      viz_(marker_pub_),
      tf_listener_() {}

void SurfaceSegmentationAction::Start() { as_.start(); }

void SurfaceSegmentationAction::Execute(
    const msgs::SegmentSurfacesGoalConstPtr& goal) {
  ros::Time start = ros::Time::now();
  ROS_INFO("SurfaceSegmentationAction::Execute... ");
  boost::shared_ptr<const sensor_msgs::PointCloud2> cloud_in;
  for (size_t i = 0; i < 10; ++i) {
    cloud_in = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(
        topic_, ros::Duration(10.0));
    if (!cloud_in) {
      msgs::SegmentSurfacesResult result;
      ROS_ERROR("Failed to get point cloud on topic: %s.", topic_.c_str());
      as_.setAborted(result);
      return;
    }
    if (cloud_in->header.stamp >= start) {
      break;
    } else {
      if (i == 9) {
        ROS_ERROR(
            "Got old point cloud! Started at: %f, cloud was captured at: %f",
            start.toSec(), cloud_in->header.stamp.toSec());
      } else {
        ROS_WARN(
            "Got old point cloud! Started at: %f, cloud was captured at: %f",
            start.toSec(), cloud_in->header.stamp.toSec());
      }
    }
  }

  // Transform into base frame.

  ROS_INFO("Surf Seg Action: Transform into base frame... ");
  tf::TransformListener tf_listener;
  std::string base_link(robot_config_.base_link());

  ROS_INFO("cloud_in frame_id is %s", cloud_in->header.frame_id.c_str());

  tf_listener.waitForTransform(base_link, cloud_in->header.frame_id,
                               ros::Time(0), ros::Duration(5.0));
  tf::StampedTransform transform;
  try {
    tf_listener.lookupTransform(base_link, cloud_in->header.frame_id,
                                ros::Time(0), transform);
  } catch (tf::TransformException& e) {
    ROS_ERROR("lookupTransform error: %s", e.what());
    msgs::SegmentSurfacesResult result;
    as_.setAborted(result, std::string(e.what()));
    return;
  }

  sensor_msgs::PointCloud2 cloud_msg;
  pcl_ros::transformPointCloud(robot_config_.base_link(), transform, *cloud_in,
                               cloud_msg);
  cloud_msg.header.frame_id = "base";
  // Start processing cloud with PCL
  ROS_INFO("Start processing cloud with PCL... ");
  msgs::SegmentSurfacesResult result;
  PointCloudC::Ptr cloud(new PointCloudC);
  pcl::fromROSMsg(cloud_msg, *cloud);

  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

  // Crop
  double min_x = 0, min_y = 0, min_z = 0;
  double max_x = 0, max_y = 0, max_z = 0;
  ros::param::param<double>("surface_segmentation/crop_min_x", min_x, 0);
  ros::param::param<double>("surface_segmentation/crop_min_y", min_y, -1);
  ros::param::param<double>("surface_segmentation/crop_min_z", min_z, 0.1);
  ros::param::param<double>("surface_segmentation/crop_max_x", max_x, 1);
  ros::param::param<double>("surface_segmentation/crop_max_y", max_y, 1);
  ros::param::param<double>("surface_segmentation/crop_max_z", max_z, 1.5);
  pcl::PointIndices::Ptr point_indices(new pcl::PointIndices);
  pcl::CropBox<PointC> crop;
  crop.setInputCloud(cloud);
  Eigen::Vector4f min;
  min << min_x, min_y, min_z, 1;
  crop.setMin(min);
  Eigen::Vector4f max;
  max << max_x, max_y, max_z, 1;
  crop.setMax(max);
  crop.filter(point_indices->indices);

  // Save cloud if requested
  ROS_INFO("Save cloud if requested... ");
  if (goal->save_cloud) {
    PointCloudC::Ptr downsampled_cloud(new PointCloudC());
    pcl::VoxelGrid<PointC> vox;
    vox.setInputCloud(cloud);
    vox.setIndices(point_indices);
    float leaf_size = 0.01;
    ros::param::param<float>("surface_segmentation/vox_leaf_size", leaf_size,
                             0.01);
    vox.setLeafSize(leaf_size, leaf_size, leaf_size);
    vox.filter(*downsampled_cloud);
    sensor_msgs::PointCloud2 downsampled_cloud_msg;
    pcl::toROSMsg(*downsampled_cloud, downsampled_cloud_msg);
    result.cloud_db_id = scene_db_.Insert(downsampled_cloud_msg);
  }

  double horizontal_tolerance_degrees;
  ros::param::param("surface_segmentation/horizontal_tolerance_degrees",
                    horizontal_tolerance_degrees, 10.0);
  double margin_above_surface;
  ros::param::param("surface_segmentation/margin_above_surface",
                    margin_above_surface, 0.015);
  double cluster_distance;
  ros::param::param("surface_segmentation/cluster_distance", cluster_distance,
                    0.025);
  int min_cluster_size;
  ros::param::param("surface_segmentation/min_cluster_size", min_cluster_size,
                    300);
  int max_cluster_size;
  ros::param::param("surface_segmentation/max_cluster_size", max_cluster_size,
                    10000);
  int min_surface_size;
  ros::param::param("surface_segmentation/min_surface_size", min_surface_size,
                    8000);
  double max_point_distance;
  ros::param::param("surface_segmentation/max_point_distance",
                    max_point_distance, 0.01);

  surface_perception::Segmentation seg;
  seg.set_input_cloud(cloud);
  seg.set_indices(point_indices);
  seg.set_horizontal_tolerance_degrees(horizontal_tolerance_degrees);
  seg.set_margin_above_surface(margin_above_surface);
  seg.set_min_surface_size(min_surface_size);
  seg.set_max_point_distance(max_point_distance);
  seg.set_cluster_distance(cluster_distance);
  seg.set_min_cluster_size(min_cluster_size);
  seg.set_max_cluster_size(max_cluster_size);

  std::vector<surface_perception::SurfaceObjects> surface_objects;
  bool success = seg.Segment(&surface_objects);

  if (!success) {
    ROS_ERROR("Failed to perceive surface objects.");
    as_.setSucceeded(result);
    return;
  }

  size_t min_size = std::numeric_limits<size_t>::max();
  size_t max_size = std::numeric_limits<size_t>::min();
  size_t num_objects = 0;

  int obj_count = 0;
  for (size_t i = 0; i < surface_objects.size(); ++i) {
    const SurfaceObjects& surface_scene = surface_objects[i];
    // get tabletop objects as landmarks
    num_objects += surface_scene.objects.size();
    if (i == 0) {
      msgs::Surface surface;
      surface.dimensions = surface_objects[i].surface.dimensions;
      surface.pose_stamped = surface_objects[i].surface.pose_stamped;
      result.surface = surface;
    }

    for (size_t j = 0; j < surface_scene.objects.size(); ++j) {
      const Object& object = surface_scene.objects[j];
      size_t cloud_size = object.indices->indices.size();
      if (cloud_size < min_size) {
        min_size = cloud_size;
      }
      if (cloud_size > max_size) {
        max_size = cloud_size;
      }

      // get object colour from point cloud
      pcl::PointXYZRGB point;
      GetRGB(object.cloud, object.indices, &point);

      ++obj_count;
      msgs::Landmark landmark;
      landmark.type = msgs::Landmark::SURFACE_BOX;
      landmark.marker_type = visualization_msgs::Marker::CUBE;
      std::stringstream ss;
      ss << "obj" << obj_count;
      landmark.name = ss.str();
      landmark.pose_stamped = object.pose_stamped;
      landmark.color.r = point.r / 255.0;
      landmark.color.g = point.g / 255.0;
      landmark.color.b = point.b / 255.0;
      landmark.color.a = 1;
      landmark.surface_box_dims = object.dimensions;
      result.landmarks.push_back(landmark);
    }

    // Add Position Landmarks from yaml file
    std::vector<double> pos_x_list, pos_y_list, pos_z_list, obj_dims;
    std::vector<std::string> name_list;

    ros::param::param<std::vector<std::string> >("world_positions/names",
                                                 name_list, name_list);
    ros::param::param<std::vector<double> >("world_positions/pos_x", pos_x_list,
                                            pos_x_list);
    ros::param::param<std::vector<double> >("world_positions/pos_y", pos_y_list,
                                            pos_x_list);
    ros::param::param<std::vector<double> >("world_positions/pos_z", pos_z_list,
                                            pos_x_list);
    ros::param::param<std::vector<double> >("world_objects/position", obj_dims,
                                            obj_dims);

    obj_count = 0;
    for (size_t i = 0; i < name_list.size(); ++i) {
      ++obj_count;
      msgs::Landmark landmark;
      landmark.type = msgs::Landmark::SURFACE_BOX;
      landmark.marker_type = visualization_msgs::Marker::CUBE;
      std::stringstream ss;
      ss << "pos" << name_list[i];
      landmark.name = ss.str();
      landmark.pose_stamped = result.surface.pose_stamped;
      landmark.pose_stamped.pose.position.x = pos_x_list[i];
      landmark.pose_stamped.pose.position.y = pos_y_list[i];
      landmark.surface_box_dims.x = obj_dims[0];
      landmark.surface_box_dims.y = obj_dims[1];
      landmark.surface_box_dims.z = obj_dims[2];
      landmark.color.r = 1;
      landmark.color.g = 1;
      landmark.color.b = 1;
      landmark.color.a = 1;
      result.landmarks.push_back(landmark);
    }

    ROS_INFO("Added %d positions", obj_count);
    ROS_INFO("Detected %ld objects, smallest: %ld points, largest: %ld points",
             num_objects, min_size, max_size);

    viz_.set_surface_objects(surface_objects);
    viz_.Show();
    as_.setSucceeded(result);
  }
}
int SurfaceSegmentationAction::GetDistance(const std::vector<int>& a,
                                           const std::vector<int>& b) {
  int dx = (a[0] - b[0]);
  int dy = (a[1] - b[1]);
  int dz = (a[2] - b[2]);
  return dx * dx + dy * dy + dz * dz;
}

void SurfaceSegmentationAction::GetRGB(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
    const pcl::PointIndices::Ptr indices, pcl::PointXYZRGB* point) {
  // look through point cloud and get average colour
  int distance_r = 0;
  int distance_g = 0;
  int distance_b = 0;
  int count_r = 0;
  int count_g = 0;
  int count_b = 0;

  int red[] = {255, 0, 0};
  std::vector<int> rgb_red(red, red + sizeof(red) / sizeof(int));
  int green[] = {0, 255, 0};
  std::vector<int> rgb_green(green, green + sizeof(green) / sizeof(int));
  int blue[] = {0, 0, 255};
  std::vector<int> rgb_blue(blue, blue + sizeof(blue) / sizeof(int));

  ROS_INFO("Colours for %zu points are: ", indices->indices.size());
  for (std::vector<int>::const_iterator i = indices->indices.begin();
       i != indices->indices.end(); ++i) {
    std::vector<int> p;
    p.push_back(cloud->points[*i].r);
    p.push_back(cloud->points[*i].g);
    p.push_back(cloud->points[*i].b);

    distance_r = GetDistance(rgb_red, p);
    distance_g = GetDistance(rgb_green, p);
    distance_b = GetDistance(rgb_blue, p);

    if (distance_r < distance_g) {
      if (distance_r < distance_b) {
        ++count_r;
      } else {
        ++count_b;
      }
    } else {
      if (distance_g < distance_b) {
        ++count_g;
      } else {
        ++count_b;
      }
    }
  }
  int* colour;
  if (count_r > count_g) {
    if (count_r > count_b) {
      colour = red;
      ROS_INFO("color is red");
    } else {
      colour = blue;
      ROS_INFO("color is blue");
    }
  } else {
    if (count_g > count_b) {
      colour = green;
      ROS_INFO("color is green");
    } else {
      colour = blue;
      ROS_INFO("color is blue");
    }
  }
  point->r = *(colour + 0);
  point->g = *(colour + 1);
  point->b = *(colour + 2);
  ROS_INFO("distribution r: %d g: %d b: %d", count_r, count_g, count_b);
  ROS_INFO("color is r: %d g: %d b: %d", point->r, point->g, point->b);
}

}  // namespace pbd
}  // namespace rapid