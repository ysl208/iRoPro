#ifndef _RAPID_PBD_VIZ_SERVER_H_
#define _RAPID_PBD_VIZ_SERVER_H_

#include <map>
#include <string>
#include <utility>
#include <vector>

#include "rapid_pbd_msgs/Condition.h"
#include "rapid_pbd_msgs/EditorEvent.h"
#include "rapid_pbd_msgs/Landmark.h"
#include "rapid_pbd_msgs/Program.h"
#include "rapid_pbd_msgs/Surface.h"
#include "robot_markers/builder.h"
#include "ros/ros.h"

#include "rapid_pbd/joint_state.h"
#include "rapid_pbd/pddl_domain.h"
#include "rapid_pbd/program_db.h"
#include "rapid_pbd/robot_config.h"
#include "rapid_pbd/world.h"

namespace msgs = rapid_pbd_msgs;
namespace rapid {
namespace pbd {

typedef std::pair<std::string, int> ProgramStep;

struct StepVisualization {
 public:
  ros::Publisher robot_pub;
  ros::Publisher scene_pub;
  ros::Publisher surface_seg_pub;
  std::string last_scene_id;
};

// Visualization server for PbD programs being edited.
class Visualizer {
 public:
  Visualizer(const SceneDb& scene_db,
             const robot_markers::Builder& marker_builder,
             const RobotConfig& robot_config);
  void Init();

  // Publish the visualization for a particular step.
  void Publish(const std::string& program_id, const World& world);
  void PublishScene(const std::string& scene_id,
                    const std::vector<msgs::Landmark>& surface_box_landmarks,
                    const msgs::Surface& surface);
  void PublishConditionMarkers(const std::string& program_id,
                               const World& world,
                               const msgs::Condition& condition);
  void PublishSpecMarkers(const std::string& program_id, const World& world,
                          const std::vector<geometry_msgs::PoseArray>& grid,
                          const msgs::Landmark& landmark);
  void StopPublishing(const std::string& program_id);

 private:
  void CreateStepVizIfNotExists(const std::string& program_id);

  const SceneDb scene_db_;
  robot_markers::Builder marker_builder_;
  const RobotConfig& robot_config_;
  std::map<std::string, StepVisualization> step_vizs_;

  ros::NodeHandle nh_;
};

// Visualizes the state of a program execution.
class RuntimeVisualizer {
 public:
  RuntimeVisualizer(const RobotConfig& robot_config,
                    const ros::Publisher& surface_box_pub);
  void PublishSurfaceBoxes(
      const std::vector<msgs::Landmark>& box_landmarks) const;

 private:
  const RobotConfig& robot_config_;
  ros::Publisher surface_box_pub_;
};
void GetConditionMarker(const msgs::Condition& condition,
                        const RobotConfig& robot_config,
                        visualization_msgs::MarkerArray* scene_markers);
void GetGridMarker(const msgs::Landmark& landmark, const msgs::Surface& surface,
                   const std::vector<geometry_msgs::PoseArray>& grid,
                   const RobotConfig& robot_config,
                   visualization_msgs::MarkerArray* scene_markers);
void GetSegmentationMarker(const std::vector<msgs::Landmark>& landmarks,
                           const RobotConfig& robot_config,
                           visualization_msgs::MarkerArray* scene_markers);
void GetSurfaceMarker(const msgs::Surface& surface,
                      const RobotConfig& robot_config,
                      visualization_msgs::MarkerArray* scene_markers);

void ClearMarkers(visualization_msgs::MarkerArray* scene_markers,
                  const int& type, const std::string& ns);
}  // namespace pbd
}  // namespace rapid

#endif  // _RAPID_PBD_VIZ_SERVER_H_
