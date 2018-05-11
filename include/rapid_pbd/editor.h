#ifndef _RAPID_PBD_EDITOR_H_
#define _RAPID_PBD_EDITOR_H_

#include <string>
#include <vector>

#include "rapid_pbd_msgs/Action.h"
#include "rapid_pbd_msgs/CreateDomain.h"
#include "rapid_pbd_msgs/CreateProgram.h"
#include "rapid_pbd_msgs/EditorEvent.h"
#include "rapid_pbd_msgs/Program.h"
#include "rapid_pbd_msgs/Specification.h"
#include "rapid_pbd_msgs/Step.h"
#include "tf/transform_listener.h"
#include "transform_graph/graph.h"

#include "rapid_pbd/action_clients.h"
#include "rapid_pbd/condition_generator.h"
#include "rapid_pbd/joint_state_reader.h"
#include "rapid_pbd/program_db.h"
#include "rapid_pbd/robot_config.h"
#include "rapid_pbd/spec_inference.h"
#include "rapid_pbd/visualizer.h"
#include "rapid_pbd/world.h"

namespace rapid {
namespace pbd {

static const char kEditorEventsTopic[] = "editor_events";

class Editor {
 public:
  Editor(const ProgramDb& db, const SceneDb& scene_db, const DomainDb& domain_db, 
         const JointStateReader& joint_state_reader,
         const Visualizer& visualizer, ActionClients* action_clients,
         const ConditionGenerator& cond_gen, const SpecInference& spec_inf,
         const RobotConfig& robot_config);
  void Start();
  void HandleEvent(const rapid_pbd_msgs::EditorEvent& event);
  bool HandleCreateProgram(rapid_pbd_msgs::CreateProgram::Request&,
                           rapid_pbd_msgs::CreateProgram::Response&);
  bool HandleCreateDomain(rapid_pbd_msgs::CreateDomain::Request&,
                           rapid_pbd_msgs::CreateDomain::Response&);

 private:
  std::string Create(const std::string& name);
  std::string CreateDomain(const std::string& name);
  void AddSenseSteps(const std::string& db_id, size_t step_id);
  void Update(const std::string& db_id, const rapid_pbd_msgs::Program& program);
  void Delete(const std::string& db_id);
  void GenerateConditions(const std::string& db_id, size_t step_id,
                          size_t action_id, const std::string& landmark_name);
  void UpdateConditions(const std::string& db_id, size_t step_id,
                        size_t action_id,
                        const rapid_pbd_msgs::Landmark& reference);
  void ViewConditions(const std::string& db_id, size_t step_id,
                      size_t action_id);
  void InferSpecification(const std::string& db_id, size_t step_id,
                          size_t action_id,
                          const rapid_pbd_msgs::Landmark& landmark);
  void ViewSpecification(const std::string& db_id, size_t step_id,
                         const rapid_pbd_msgs::Specification& spec);
  void SelectSpecification(const std::string& db_id, size_t step_id,
                           const rapid_pbd_msgs::Specification& spec);

  void AddStep(const std::string& db_id);
  void DeleteStep(const std::string& db_id, size_t step_id);
  void AddAction(const std::string& db_id, size_t step_id,
                 rapid_pbd_msgs::Action action);
  void AssignGripperBoundingBox(rapid_pbd_msgs::Step* cart_step,
                                rapid_pbd_msgs::Landmark* gripper_box);
  void DeleteAction(const std::string& db_id, size_t step_id, size_t action_id);
  void ViewStep(const std::string& db_id, size_t step_id);
  void DetectSurfaceObjects(const std::string& db_id, size_t step_id);
  void GetJointValues(const std::string& db_id, size_t step_id,
                      size_t action_id, const std::string& actuator_group);
  // Pose actions
  // Main handler
  void GetPose(const std::string& db_id, size_t step_id, size_t action_id,
               const std::string& actuator_group,
               const rapid_pbd_msgs::Landmark& landmark);
  // Handler if getting a new pose
  void GetNewPose(const rapid_pbd_msgs::Landmark& landmark, const World& world,
                  const std::string& actuator_group,
                  rapid_pbd_msgs::Action* action);
  // Handler if changing reference frame for a pose
  void ReinterpretPose(const rapid_pbd_msgs::Landmark& new_landmark,
                       rapid_pbd_msgs::Action* action);
  // Returns the closest landmark
  // ee_position is the position of the end-effector in the base frame.
  // squared_distance_cutoff is the cutoff for a landmark to be considered
  // "close". The distance should be squared, i.e., in units of meters^2.
  // Returns true if a landmark closer than the cutoff was found. In that case,
  // landmark is mutated.
  bool ClosestLandmark(const geometry_msgs::Vector3& ee_position,
                       const World& world, const double squared_distance_cutoff,
                       rapid_pbd_msgs::Landmark* landmark);

  // Delete a scene from the scene_db by ID if it exists.
  // Logs an error message if the ID is non-empty and does not exist in the DB.
  void DeleteScene(const std::string& scene_id);

  // Removes all landmarks of the given type from the given step.
  // The types are defined in Landmark.msg.
  void DeleteLandmarks(const std::string& landmark_type,
                       rapid_pbd_msgs::Step* step);
  void AddDetectTTObjectsAction(const std::string& db_id, size_t step_id);
  void AddCheckConditionsAction(const std::string& db_id, size_t step_id);
  void AddMoveHeadAction(const std::string& db_id, size_t step_id, size_t pan,
                         size_t tilt);
  void AddOpenGripperAction(const std::string& db_id, size_t step_id,
                            size_t position, size_t max_effort);
  void AddJointStates(rapid_pbd_msgs::Action* action,
                      const std::vector<double>& default_pose);
  void AddGripperPoseAction(const std::string& db_id, size_t step_id,
                            const std::vector<double>& default_pose);
  bool GetCartActions(std::vector<std::pair<int, int> >* cart_pose_actions,
                      const rapid_pbd_msgs::Program& program,
                      rapid_pbd_msgs::Program* new_program);
  void GetDemonstrationSteps(const rapid_pbd_msgs::Program& program,
                             std::vector<rapid_pbd_msgs::Step>* demo_steps);
  void RunProgram(const std::string& program_name);
  geometry_msgs::Vector3 QuaternionToRPY(const geometry_msgs::Quaternion& msg);
  bool AABBintersect(const rapid_pbd_msgs::Landmark& lm1,
                     const rapid_pbd_msgs::Landmark& lm2);
  void GetMinMax(const rapid_pbd_msgs::Landmark& lm,
                 geometry_msgs::Vector3* min, geometry_msgs::Vector3* max);
  bool CheckObjectCollision(
      const std::vector<rapid_pbd_msgs::Landmark>& landmarks,
      const rapid_pbd_msgs::Landmark& bounding_box);
  bool CheckGripperSpaceEnough(
      const std::vector<rapid_pbd_msgs::Landmark>& landmarks,
      const geometry_msgs::Point& position,
      const rapid_pbd_msgs::Landmark& bounding_box);
  bool CheckGridPositionFree(
      const std::vector<rapid_pbd_msgs::Landmark>& landmarks,
      const geometry_msgs::Point& position);
  ProgramDb db_;
  SceneDb scene_db_;
  DomainDb domain_db_;
  JointStateReader joint_state_reader_;
  Visualizer viz_;
  ActionClients* action_clients_;
  ConditionGenerator cond_gen_;
  SpecInference spec_inf_;
  const RobotConfig& robot_config_;
  tf::TransformListener tf_listener_;
  std::map<std::string, size_t> last_viewed_;
};
}  // namespace pbd
}  // namespace rapid

#endif  // _RAPID_PBD_EDITOR_H_
