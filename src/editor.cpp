#include "rapid_pbd/editor.h"

#include <string.h>
#include <cmath>
#include <exception>
#include <string>
#include <utility>
#include <vector>

#include "pddl_msgs/PDDLPlannerGoal.h"
#include "rapid_pbd_msgs/Action.h"
#include "rapid_pbd_msgs/Condition.h"
#include "rapid_pbd_msgs/EditorEvent.h"
#include "rapid_pbd_msgs/ExecuteProgramGoal.h"
#include "rapid_pbd_msgs/Landmark.h"
#include "rapid_pbd_msgs/PDDLDomain.h"
#include "rapid_pbd_msgs/Program.h"
#include "rapid_pbd_msgs/SegmentSurfacesGoal.h"
#include "rapid_pbd_msgs/Step.h"
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "tf/transform_listener.h"
#include "transform_graph/graph.h"

#include "rapid_pbd/action_clients.h"
#include "rapid_pbd/action_utils.h"
#include "rapid_pbd/condition_generator.h"
#include "rapid_pbd/joint_state_reader.h"
#include "rapid_pbd/landmarks.h"
#include "rapid_pbd/pddl_domain.h"
#include "rapid_pbd/program_db.h"
#include "rapid_pbd/robot_config.h"
#include "rapid_pbd/spec_inference.h"
#include "rapid_pbd/visualizer.h"
#include "rapid_pbd/world.h"
using rapid_pbd_msgs::Action;

namespace msgs = rapid_pbd_msgs;
namespace rapid {
namespace pbd {
Editor::Editor(const ProgramDb& db, const SceneDb& scene_db,
               const PDDLDomainDb& domain_db,
               const JointStateReader& joint_state_reader,
               const Visualizer& visualizer, ActionClients* action_clients,
               const ConditionGenerator& cond_gen,
               const SpecInference& spec_inf,
               const ros::Publisher pddl_domain_pub,
               const RobotConfig& robot_config)
    : db_(db),
      scene_db_(scene_db),
      domain_db_(domain_db),
      joint_state_reader_(joint_state_reader),
      viz_(visualizer),
      action_clients_(action_clients),
      cond_gen_(cond_gen),
      pddl_domain_(pddl_domain_pub),
      spec_inf_(spec_inf),
      robot_config_(robot_config),
      tf_listener_(),
      last_viewed_() {}

void Editor::Start() {
  db_.Start();
  domain_db_.Start();
  joint_state_reader_.Start();
  viz_.Init();
  spec_inf_.Init();
  // CreatePDDLDomain("MainDomain");
}

void Editor::HandleEvent(const msgs::EditorEvent& event) {
  try {
    if (event.type == msgs::EditorEvent::UPDATE) {
      Update(event.program_info.db_id, event.program);
    } else if (event.type == msgs::EditorEvent::DELETE) {
      Delete(event.program_info.db_id);
    }
    // PDDL events
    else if (event.type == msgs::EditorEvent::SAVE_ON_EXIT) {
      SaveOnExit(event.domain_id, event.action_name);
    } else if (event.type == msgs::EditorEvent::UPDATE_PDDL_DOMAIN) {
      UpdatePDDLDomain(event.domain_id, event.pddl_domain);
    } else if (event.type == msgs::EditorEvent::DETECT_ACTION_CONDITIONS) {
      DetectActionConditions(event.domain_id, event.action_name,
                             event.state_name);
    } else if (event.type == msgs::EditorEvent::ASSIGN_SURFACE_OBJECTS) {
      AssignSurfaceObjects(event.program_info.db_id, event.pddl_action,
                           event.state_name, event.step_num);
      // PDDL actions
    } else if (event.type == msgs::EditorEvent::ADD_PDDL_ACTION) {
      AddPDDLAction(event.domain_id, event.action_name);
    } else if (event.type == msgs::EditorEvent::COPY_PDDL_ACTION) {
      CopyPDDLAction(event.domain_id, event.action_name);
    } else if (event.type == msgs::EditorEvent::DELETE_PDDL_ACTION) {
      DeletePDDLAction(event.domain_id, event.action_name);
    } else if (event.type == msgs::EditorEvent::UPDATE_PDDL_ACTION) {
      UpdatePDDLAction(event.domain_id, event.pddl_action, event.action_name);
      // PDDL problems
    } else if (event.type == msgs::EditorEvent::DETECT_WORLD_STATE) {
      DetectWorldState(event.domain_id, event.problem_name, event.state_name);
    } else if (event.type == msgs::EditorEvent::ADD_PDDL_PROBLEM) {
      AddPDDLProblem(event.domain_id, event.problem_name);
    } else if (event.type == msgs::EditorEvent::DELETE_PDDL_PROBLEM) {
      DeletePDDLProblem(event.domain_id, event.problem_name);
    } else if (event.type == msgs::EditorEvent::UPDATE_PDDL_PROBLEM) {
      UpdatePDDLProblem(event.domain_id, event.pddl_problem,
                        event.problem_name);
    } else if (event.type == msgs::EditorEvent::SOLVE_PDDL_PROBLEM) {
      SolvePDDLProblem(event.domain_id, event.pddl_problem, event.planner);
    } else if (event.type == msgs::EditorEvent::RUN_PDDL_PLAN) {
      RunPDDLPlan(event.domain_id, event.pddl_problem, event.planner);
    }

    // Condition events
    else if (event.type == msgs::EditorEvent::GENERATE_CONDITIONS) {
      GenerateConditions(event.program_info.db_id, event.step_num,
                         event.action_num, event.landmark.name);
    } else if (event.type == msgs::EditorEvent::VIEW_CONDITIONS) {
      ViewConditions(event.program_info.db_id, event.step_num,
                     event.action_num);
    } else if (event.type == msgs::EditorEvent::UPDATE_CONDITIONS) {
      UpdateConditions(event.program_info.db_id, event.step_num,
                       event.action_num, event.action.condition.reference);
    }
    // Specification Inference events
    else if (event.type == msgs::EditorEvent::INFER_SPECIFICATION) {
      InferSpecification(event.program_info.db_id, event.step_num,
                         event.action_num, event.landmark);
    } else if (event.type == msgs::EditorEvent::VIEW_SPECIFICATION) {
      ViewSpecification(event.program_info.db_id, event.step_num, event.spec);
    } else if (event.type == msgs::EditorEvent::SELECT_SPECIFICATION) {
      SelectSpecification(event.program_info.db_id, event.step_num, event.spec);
    } else if (event.type == msgs::EditorEvent::ADD_SENSE_STEPS) {
      AddSenseSteps(event.program_info.db_id, event.step_num);
    } else if (event.type == msgs::EditorEvent::ADD_STEP) {
      AddStep(event.program_info.db_id);
    } else if (event.type == msgs::EditorEvent::DELETE_STEP) {
      DeleteStep(event.program_info.db_id, event.step_num);
    } else if (event.type == msgs::EditorEvent::ADD_ACTION) {
      AddAction(event.program_info.db_id, event.step_num, event.action);
    } else if (event.type == msgs::EditorEvent::DELETE_ACTION) {
      DeleteAction(event.program_info.db_id, event.step_num, event.action_num);
    } else if (event.type == msgs::EditorEvent::VIEW_STEP) {
      ViewStep(event.program_info.db_id, event.step_num);
    } else if (event.type == msgs::EditorEvent::DETECT_SURFACE_OBJECTS) {
      DetectSurfaceObjects(event.program_info.db_id, event.step_num);
    } else if (event.type == msgs::EditorEvent::GET_JOINT_VALUES) {
      GetJointValues(event.program_info.db_id, event.step_num, event.action_num,
                     event.action.actuator_group);
    } else if (event.type == msgs::EditorEvent::GET_POSE) {
      GetPose(event.program_info.db_id, event.step_num, event.action_num,
              event.action.actuator_group, event.action.landmark);
    } else {
      ROS_ERROR("Unknown event type \"%s\"", event.type.c_str());
    }
  } catch (const std::exception& ex) {
    ROS_ERROR("Unhandled exception for event %s: %s", event.type.c_str(),
              ex.what());
  }
}  // namespace pbd

bool Editor::HandleCreatePDDLDomain(
    msgs::CreatePDDLDomain::Request& request,
    msgs::CreatePDDLDomain::Response& response) {
  ROS_INFO("Handle request to create PDDL domain '%s'", request.name.c_str());
  response.domain_id = CreatePDDLDomain(request.name);
  return true;
}

std::string Editor::CreatePDDLDomain(const std::string& name) {
  msgs::PDDLDomain domain;
  // if (!domain_db_.GetByName(name, &domain)) {
  pddl_domain_.Init(&domain, name);
  std::string id = domain_db_.Insert(domain);
  pddl_domain_.domain_id = id;
  ROS_INFO("Created new domain '%s' with id %s", name.c_str(),
           pddl_domain_.domain_id.c_str());
  //}
  if (pddl_domain_.domain_id.size() == 0)
    pddl_domain_.domain_id = "5bc0a6266fbe3c069ef413e3";
  domain_db_.StartPublishingPDDLDomainById(pddl_domain_.domain_id);
  pddl_domain_.PublishPDDLDomain(domain);
  return id;
}

bool Editor::HandleCreateProgram(msgs::CreateProgram::Request& request,
                                 msgs::CreateProgram::Response& response) {
  response.db_id = Create(request.name);
  return true;
}

std::string Editor::Create(const std::string& name) {
  msgs::Program program;
  program.name = name;

  joint_state_reader_.ToMsg(&program.start_joint_state);
  std::string id = db_.Insert(program);
  db_.StartPublishingProgramById(id);

  AddSenseSteps(id, 0);

  msgs::Program test_program;
  bool success = db_.Get(id, &test_program);
  if (!success) {
    ROS_ERROR("Unable to view program \"%s\"", id.c_str());
  }
  ROS_INFO("Program '%s' should have 2 steps but has '%ld'",
           test_program.name.c_str(), test_program.steps.size());

  // AddStep(id);
  ROS_INFO("Created new Program '%s' (%s)", program.name.c_str(), id.c_str());
  return id;
}

void Editor::AddSenseSteps(const std::string& db_id, size_t step_id) {
  last_viewed_[db_id] = step_id;
  AddStep(db_id);
  // AddMoveHeadAction(db_id, step_id, 45, 0);
  // AddOpenGripperAction(db_id, step_id, 100, 0);
  std::vector<double> poses;
  robot_config_.default_gripper_poses(&poses);
  AddGripperPoseAction(db_id, step_id, poses);

  AddStep(db_id);
  ++step_id;
  AddDetectTTObjectsAction(db_id, step_id);
  ROS_INFO("Added sense steps for '%s'", db_id.c_str());
  // AddStep(db_id);
  //++step_id;
  // AddCheckConditionsAction(db_id, step_id);
}

void Editor::AddDetectTTObjectsAction(const std::string& db_id,
                                      size_t step_id) {
  msgs::Action detect_action;
  detect_action.type = msgs::Action::DETECT_TABLETOP_OBJECTS;
  AddAction(db_id, step_id, detect_action);
}

void Editor::AddCheckConditionsAction(const std::string& db_id,
                                      size_t step_id) {
  msgs::Action check_action;
  check_action.type = msgs::Action::CHECK_CONDITIONS;
  check_action.condition.relevant = false;
  AddAction(db_id, step_id, check_action);
}

void Editor::AddMoveHeadAction(const std::string& db_id, size_t step_id,
                               size_t pan, size_t tilt) {
  msgs::Action head_action;
  head_action.type = msgs::Action::MOVE_TO_JOINT_GOAL;
  head_action.actuator_group = msgs::Action::HEAD;
  trajectory_msgs::JointTrajectory joint_trajectory;
  head_action.joint_trajectory.joint_names.insert(
      head_action.joint_trajectory.joint_names.begin(), "head_pan_joint");
  head_action.joint_trajectory.joint_names.insert(
      head_action.joint_trajectory.joint_names.begin(), "head_tilt_joint");
  trajectory_msgs::JointTrajectoryPoint point;
  point.positions.insert(point.positions.begin(), pan * M_PI / 180);
  point.positions.insert(point.positions.begin(), tilt * M_PI / 180);
  // std::duration data = {secs: 2, nsecs: 0};
  // point.time_from_start = data;
  head_action.joint_trajectory.points.insert(
      head_action.joint_trajectory.points.begin(), point);
  AddAction(db_id, step_id, head_action);
}

void Editor::AddOpenGripperAction(const std::string& db_id, size_t step_id,
                                  size_t position, size_t max_effort) {
  if (robot_config_.num_arms() == 1) {
    msgs::Action open_gripper_action;
    open_gripper_action.type = msgs::Action::ACTUATE_GRIPPER;
    open_gripper_action.actuator_group = msgs::Action::GRIPPER;
    open_gripper_action.gripper_command.position = position;
    open_gripper_action.gripper_command.max_effort = max_effort;
    AddAction(db_id, step_id, open_gripper_action);
  } else if (robot_config_.num_arms() == 2) {
    msgs::Action open_l_gripper_action;
    open_l_gripper_action.type = msgs::Action::ACTUATE_GRIPPER;
    open_l_gripper_action.actuator_group = msgs::Action::LEFT_GRIPPER;
    open_l_gripper_action.gripper_command.position = position;
    open_l_gripper_action.gripper_command.max_effort = max_effort;
    AddAction(db_id, step_id, open_l_gripper_action);

    msgs::Action open_r_gripper_action;
    open_r_gripper_action.type = msgs::Action::ACTUATE_GRIPPER;
    open_r_gripper_action.actuator_group = msgs::Action::RIGHT_GRIPPER;
    open_r_gripper_action.gripper_command.position = position;
    open_r_gripper_action.gripper_command.max_effort = max_effort;
    AddAction(db_id, step_id, open_r_gripper_action);
  }
}

void Editor::AddJointStates(msgs::Action* move_arm_action,
                            const std::vector<double>& default_pose) {
  move_arm_action->pose.position.x = default_pose[0];
  move_arm_action->pose.position.y = default_pose[1];
  move_arm_action->pose.position.z = default_pose[2];
  move_arm_action->pose.orientation.w = default_pose[3];
  move_arm_action->pose.orientation.x = default_pose[4];
  move_arm_action->pose.orientation.y = default_pose[5];
  move_arm_action->pose.orientation.z = default_pose[6];

  msgs::Landmark landmark;
  landmark.type = msgs::Landmark::TF_FRAME;
  landmark.name = robot_config_.torso_link();
  move_arm_action->landmark = landmark;
  // Set joint angles as a seed.
  std::vector<std::string> joint_names;
  robot_config_.joints_for_group(move_arm_action->actuator_group, &joint_names);
  if (joint_names.size() == 0) {
    ROS_ERROR("Can't get joint angles for actuator group \"%s\"",
              move_arm_action->actuator_group.c_str());
    return;
  }

  std::vector<double> joint_positions;
  for (size_t i = 0; i < joint_names.size(); ++i) {
    const std::string& name = joint_names[i];
    double pos = joint_state_reader_.get_position(name);
    if (pos == kNoJointValue) {
      ROS_ERROR("Could not get angle for joint \"%s\"", name.c_str());
      joint_positions.push_back(0);
    } else {
      joint_positions.push_back(pos);
    }
  }
  SetJointPositions(joint_names, joint_positions, move_arm_action);
}
void Editor::AddGripperPoseAction(const std::string& db_id, size_t step_id,
                                  const std::vector<double>& default_pose) {
  if (robot_config_.num_arms() == 1) {
    msgs::Action move_arm_action;
    move_arm_action.type = msgs::Action::MOVE_TO_CARTESIAN_GOAL;
    move_arm_action.actuator_group = msgs::Action::ARM;

    AddJointStates(&move_arm_action, default_pose);
    AddAction(db_id, step_id, move_arm_action);

  } else if (robot_config_.num_arms() == 2) {
    msgs::Action move_l_arm_action;
    move_l_arm_action.type = msgs::Action::MOVE_TO_CARTESIAN_GOAL;
    move_l_arm_action.actuator_group = msgs::Action::LEFT_ARM;
    AddJointStates(&move_l_arm_action, default_pose);
    AddAction(db_id, step_id, move_l_arm_action);

    msgs::Action move_r_arm_action;
    move_r_arm_action.type = msgs::Action::MOVE_TO_CARTESIAN_GOAL;
    move_r_arm_action.actuator_group = msgs::Action::RIGHT_ARM;
    std::vector<double> r_default_pose(&default_pose[7], &default_pose[11]);

    AddJointStates(&move_r_arm_action, r_default_pose);
    AddAction(db_id, step_id, move_r_arm_action);
  }
}

void Editor::Update(const std::string& db_id, const msgs::Program& program) {
  db_.Update(db_id, program);
  if (last_viewed_.find(db_id) != last_viewed_.end()) {
    World world;
    GetWorld(robot_config_, program, last_viewed_[db_id], &world);
    viz_.Publish(db_id, world);
  } else {
    ROS_ERROR("Unable to publish visualization: unknown step");
  }
}

void Editor::Delete(const std::string& db_id) {
  msgs::Program program;
  bool success = db_.Get(db_id, &program);
  if (!success) {
    ROS_ERROR("Unable to delete program ID \"%s\"", db_id.c_str());
    return;
  }
  for (size_t i = 0; i < program.steps.size(); ++i) {
    const msgs::Step& step = program.steps[i];
    DeleteScene(step.scene_id);
  }

  db_.Delete(db_id);
  viz_.StopPublishing(db_id);
}

void Editor::GenerateConditions(const std::string& db_id, size_t step_id,
                                size_t action_id,
                                const std::string& landmark_name) {
  // Generates conditions for current step
  msgs::Program program;
  int obj_num = 5;
  bool success = db_.Get(db_id, &program);
  if (!success) {
    ROS_ERROR("Unable to submit program \"%s\"", db_id.c_str());
    return;
  }
  if (step_id >= program.steps.size()) {
    ROS_ERROR(
        "Unable to get action from step %ld from program \"%s\", which has "
        "%ld steps",
        step_id, db_id.c_str(), program.steps.size());
    return;
  }
  DeleteScene(program.steps[step_id].scene_id);
  msgs::Step* step = &program.steps[step_id];
  World initial_world;
  GetWorld(robot_config_, program, step_id, &initial_world);

  msgs::Condition action_condition = step->actions[action_id].condition;
  cond_gen_.AssignLandmarkCondition(initial_world, landmark_name,
                                    &action_condition);

  step->actions[action_id].condition = action_condition;
  // publish condition markers
  db_.Update(db_id, program);
  if (last_viewed_.find(db_id) != last_viewed_.end()) {
    World world;
    GetWorld(robot_config_, program, last_viewed_[db_id], &world);
    viz_.PublishConditionMarkers(db_id, world, action_condition);
  } else {
    ROS_ERROR("Unable to publish condition visualization: unknown step");
  }
}

void Editor::ViewConditions(const std::string& db_id, size_t step_id,
                            size_t action_id) {
  msgs::Program program;
  bool success = db_.Get(db_id, &program);
  if (!success) {
    ROS_ERROR("Unable to submit program \"%s\"", db_id.c_str());
    return;
  }
  if (step_id >= program.steps.size()) {
    ROS_ERROR(
        "Unable to get action from step %ld from program \"%s\", which has "
        "%ld steps",
        step_id, db_id.c_str(), program.steps.size());
    return;
  }
  msgs::Step* step = &program.steps[step_id];
  msgs::Condition action_condition = step->actions[action_id].condition;

  // std::vector<geometry_msgs::PoseArray> grid;
  // cond_gen_.GenerateGrid(&action_condition, &grid, obj_num);
  // step->grid = grid;

  db_.Update(db_id, program);
  if (last_viewed_.find(db_id) != last_viewed_.end()) {
    World world;
    GetWorld(robot_config_, program, last_viewed_[db_id], &world);
    viz_.PublishConditionMarkers(db_id, world, action_condition);
  } else {
    ROS_ERROR("Unable to publish condition visualization: unknown step");
  }
}

void Editor::UpdateConditions(const std::string& db_id, size_t step_id,
                              size_t action_id,
                              const msgs::Landmark& reference) {
  msgs::Program program;
  bool success = db_.Get(db_id, &program);
  if (!success) {
    ROS_ERROR("Unable to submit program \"%s\"", db_id.c_str());
    return;
  }
  if (step_id >= program.steps.size()) {
    ROS_ERROR(
        "Unable to get action from step %ld from program \"%s\", which has "
        "%ld steps",
        step_id, db_id.c_str(), program.steps.size());
    return;
  }
  msgs::Step* step = &program.steps[step_id];

  World world;
  GetWorld(robot_config_, program, step_id, &world);
  msgs::Condition action_condition = step->actions[action_id].condition;
  if (action_condition.referenceRelevant && reference.name != "") {
    cond_gen_.UpdateReferenceLandmark(world, &action_condition, reference);
    step->actions[action_id].condition = action_condition;
  } else {
    step->actions[action_id].condition.referenceRelevant = false;
  }

  db_.Update(db_id, program);
  if (last_viewed_.find(db_id) != last_viewed_.end()) {
    World world;
    GetWorld(robot_config_, program, last_viewed_[db_id], &world);
    viz_.PublishConditionMarkers(db_id, world,
                                 step->actions[action_id].condition);
  } else {
    ROS_ERROR("Unable to publish condition visualization: unknown step");
  }
}

void Editor::InferSpecification(const std::string& db_id, size_t step_id,
                                size_t action_id,
                                const msgs::Landmark& landmark) {
  // Infers specification after the last landmark has been placed
  msgs::Program program;
  bool success = db_.Get(db_id, &program);
  if (!success) {
    ROS_ERROR("Unable to submit program \"%s\"", db_id.c_str());
    return;
  }
  if (step_id >= program.steps.size()) {
    ROS_ERROR(
        "Unable to get action from step %ld from program \"%s\", which has "
        "%ld steps",
        step_id, db_id.c_str(), program.steps.size());
    return;
  }
  msgs::Step* step = &program.steps[step_id];

  World world;
  GetWorld(robot_config_, program, step_id, &world);

  // if there is only one landmark, initialise specs
  // assumes that if there are more, then it is already initialised
  std::vector<msgs::Specification> specs;
  std::vector<float> posteriors;
  if (program.template_specs.size() < 1 ||
      world.surface_box_landmarks.size() <= 1 ||
      program.posteriors.data.size() < 0) {
    std::cout << "Re-initalising specs with landmark: "
              << landmark.surface_box_dims.x << "\n";
    spec_inf_.InitSpecs(&specs, landmark);
    program.template_specs = specs;
    // new specification for this program
    msgs::Specification spec = program.spec;
    spec_inf_.InitSpec(&spec);
    program.spec = spec;
    program.spec.avg_dx =
        fmin(landmark.surface_box_dims.x, landmark.surface_box_dims.y) + 0.02;
    program.spec.avg_dy =
        fmax(landmark.surface_box_dims.x, landmark.surface_box_dims.y) + 0.02;
    program.spec.landmark = landmark;
    program.spec.obj_num = 100;

    // get posteriors from program if already exists, if not, take initialised
    // ones
    posteriors = spec_inf_.posteriors_;
    std::cout << "program posteriors didnt exist, initialise"
              << "\n";
  } else {
    std::cout << "program posteriors already exist, use old ones"
              << "\n";
    for (size_t i = 0; i < program.posteriors.data.size(); ++i) {
      posteriors.push_back(program.posteriors.data[i]);
    }
  }
  spec_inf_.UpdatePosteriors(world, landmark, &posteriors, &program.spec);
  // replace old posteriors with new ones
  program.posteriors.data.clear();
  for (size_t i = 0; i < posteriors.size(); ++i) {
    program.posteriors.data.push_back(posteriors[i]);
  }
  std::cout << "new posteriors are " << program.posteriors.data.size() << " \n";

  db_.Update(db_id, program);
}

void Editor::ViewSpecification(const std::string& db_id, size_t step_id,
                               const msgs::Specification& temp_spec) {
  msgs::Program program;
  bool success = db_.Get(db_id, &program);
  if (!success) {
    ROS_ERROR("Unable to submit program \"%s\"", db_id.c_str());
    return;
  }
  if (step_id >= program.steps.size()) {
    ROS_ERROR(
        "Unable to get action from step %ld from program \"%s\", which has "
        "%ld steps",
        step_id, db_id.c_str(), program.steps.size());
    return;
  }
  msgs::Step* step = &program.steps[step_id];

  World world;
  GetWorld(robot_config_, program, last_viewed_[db_id], &world);
  msgs::Specification spec = temp_spec;
  spec_inf_.GetOffset(temp_spec, &spec.offset);
  // spec.offset.x = spec.offset.x * spec.avg_dx * 0.5;
  // spec.offset.y = spec.offset.y * spec.avg_dy * 0.5;
  std::vector<geometry_msgs::PoseArray> grid;
  spec_inf_.GenerateGrid(spec, world.surface, &grid);
  program.grid = grid;
  program.spec = spec;

  db_.Update(db_id, program);
  if (last_viewed_.find(db_id) != last_viewed_.end()) {
    viz_.PublishSpecMarkers(db_id, world, grid, spec.landmark);
  } else {
    ROS_ERROR("Unable to publish spec visualization: unknown step");
  }
}

void Editor::RunProgram(const std::string& program_name) {
  msgs::ExecuteProgramGoal goal;
  goal.name = program_name;
  action_clients_->program_client.sendGoal(goal);
  bool finished_before_timeout =
      action_clients_->program_client.waitForResult(ros::Duration(30));
  if (!finished_before_timeout) {
    ROS_INFO("Program %s did not finish before the time out.",
             program_name.c_str());
    // return;
  }
  msgs::ExecuteProgramResult::ConstPtr result =
      action_clients_->program_client.getResult();
}

void Editor::SelectSpecification(const std::string& db_id, size_t step_id,
                                 const msgs::Specification& temp_spec) {
  // Assumes that this is called in the 'Place & Infer' program
  // TO DO: assume that spec.landmark is the last added landmark, might not be
  // true
  // get the program
  msgs::Program main_program;
  bool success = db_.Get(db_id, &main_program);
  if (!success) {
    ROS_ERROR("Unable to submit program \"%s\"", db_id.c_str());
    return;
  }
  if (step_id >= main_program.steps.size()) {
    ROS_ERROR(
        "Unable to get action from step %ld from program \"%s\", which has "
        "%ld steps",
        step_id, db_id.c_str(), main_program.steps.size());
    return;
  }

  msgs::Specification spec = temp_spec;

  // Create new program that will be modified and run for each grid position
  msgs::Program new_program = main_program;

  World world;  // needed for surface box landmarks
  GetWorld(robot_config_, main_program, last_viewed_[db_id], &world);
  std::vector<geometry_msgs::PoseArray> grid = main_program.grid;

  // 1.1 save 'move to cart pose' actions in an array
  // 1.2 get demo reference position by looking for 'open gripper action' and
  // taking the latest 'move to cart pose'
  // Note: reference position in demo always aligns with 1st object in grid

  // 2. Loop through grid positions
  // 2.1 if landmark on position, skip
  // 2.2 if position empty, modify poses and run program
  double dx, dy, dz;  // track transformation from 0th to current position
  int count = 1;
  std::cout << "Grid positions = " << grid.size() * grid[0].poses.size()
            << "\n";
  std::cout << "Height = " << spec.height_num << "\n";
  for (size_t height = 0; height < spec.height_num; ++height) {
    // update z coordinate to stack
    std::cout << "Current height: " << height + 1 << "\n";
    dz = (main_program.spec.landmark.surface_box_dims.z + 0.005) * height;
    for (size_t row = 0; row < grid.size(); ++row) {
      for (size_t col = 0; col < grid[row].poses.size(); ++col) {
        geometry_msgs::Pose pose = grid[row].poses[col];
        std::cout << "-- #" << count << " grid position is \n"
                  << pose.position << "\n";

        dx = pose.position.x - grid[0].poses[0].position.x;
        dy = pose.position.y - grid[0].poses[0].position.y;

        std::cout << "dx is " << dx << ",";
        std::cout << "dy is " << dy << ",";
        std::cout << "dz is " << dz << "\n";
        if (CheckGridPositionFree(world.surface_box_landmarks, pose.position) ||
            height > 0) {
          // Check current Place program has enough space
          msgs::Program alt_program = main_program;
          bool gripperSpace = true;
          if (alt_program.gripper_box.name != "") {
            ROS_INFO("Checking gripper space for %s", alt_program.name.c_str());
            gripperSpace =
                CheckGripperSpaceEnough(world.surface_box_landmarks,
                                        pose.position, alt_program.gripper_box);
          }

          if (!gripperSpace) {
            std::cout << "Gripper space not enough, checking other programs\n";
            std::vector<std::string> names;
            bool listSuccess = db_.GetList(&names);
            if (!success) {
              ROS_ERROR("Unable to get list of programs.");
              return;
            }
            std::cout << "List of other programs is: " << names.size() << "\n";
            for (size_t p_id; p_id < names.size(); ++p_id) {
              bool programSuccess = db_.GetByName(names[p_id], &alt_program);
              if (!programSuccess || names[p_id] == main_program.name) {
                continue;
              }
              ROS_INFO("Checking gripper space for %s",
                       alt_program.name.c_str());
              gripperSpace = CheckGripperSpaceEnough(
                  world.surface_box_landmarks, pose.position,
                  alt_program.gripper_box);
              if (gripperSpace) {
                ROS_INFO("Alternative Place program found: %s",
                         alt_program.name.c_str());
                // new_program.steps = alt_program.steps;
                break;
              }
            }
          }

          std::vector<std::pair<int, int> > cart_pose_actions;
          new_program.steps.clear();
          bool newProgramSuccess =
              GetCartActions(&cart_pose_actions, alt_program, &new_program);
          if (!newProgramSuccess) {
            ROS_ERROR("No cartesian actions found in program \"%s\"",
                      alt_program.name.c_str());
            return;
          }
          // use this grid pose, update cart_pose_actions with new grid pose
          // find transform from grid pos to ref_pose
          for (size_t id = 0; id < cart_pose_actions.size(); ++id) {
            int s_id = cart_pose_actions[id].first;
            int a_id = cart_pose_actions[id].second;

            // reset pose to original demo pose
            new_program.steps[s_id].actions[a_id].pose.position.x =
                alt_program.steps[s_id].actions[a_id].pose.position.x;
            new_program.steps[s_id].actions[a_id].pose.position.y =
                alt_program.steps[s_id].actions[a_id].pose.position.y;
            new_program.steps[s_id].actions[a_id].pose.position.z =
                alt_program.steps[s_id].actions[a_id].pose.position.z;

            std::cout << "Reset to original x pose for step/action = " << s_id
                      << "," << a_id << ": x = "
                      << new_program.steps[s_id].actions[a_id].pose.position.x
                      << ", y = "
                      << new_program.steps[s_id].actions[a_id].pose.position.y
                      << ", z = "
                      << new_program.steps[s_id].actions[a_id].pose.position.z
                      << "\n";
            // transform pose to new grid position
            new_program.steps[s_id].actions[a_id].pose.position.x += dx;
            new_program.steps[s_id].actions[a_id].pose.position.y += dy;
            new_program.steps[s_id].actions[a_id].pose.position.z += dz;

            std::cout << "New x pose for step/action = " << s_id << "," << a_id
                      << ": x = "
                      << new_program.steps[s_id].actions[a_id].pose.position.x
                      << ", y = "
                      << new_program.steps[s_id].actions[a_id].pose.position.y
                      << ", z = "
                      << new_program.steps[s_id].actions[a_id].pose.position.z
                      << "\n";
          }
          std::cout << "Running program..." << spec.pick_program << "\n";
          if (spec.pick_program != "") {
            RunProgram(spec.pick_program);
            ROS_INFO("%s done. Press enter to proceed to Place",
                     spec.pick_program.c_str());
            std::cin.ignore();
          }

          std::cout << "Running program..." << alt_program.name << "\n";
          msgs::ExecuteProgramGoal goal;
          goal.program = new_program;
          action_clients_->program_client.sendGoal(goal);
          bool finished_before_timeout =
              action_clients_->program_client.waitForResult(
                  ros::Duration(30.0));
          if (!finished_before_timeout) {
            ROS_INFO("Program did not finish before the time out.");
          }
          msgs::ExecuteProgramResult::ConstPtr result =
              action_clients_->program_client.getResult();
          ROS_INFO("Place done. Press enter to continue");
          std::cin.ignore();
        }
        ++count;
      }
    }
  }
}
bool Editor::GetCartActions(
    std::vector<std::pair<int, int> >* cart_pose_actions,
    const msgs::Program& program, msgs::Program* new_program) {
  // 1. Extracts step and action pairs with 'move-to-cart-pose'-actions
  // 2. Creates new program cut off at INF SPEC
  for (size_t step_id = 0; step_id < program.steps.size(); ++step_id) {
    msgs::Step step = program.steps[step_id];
    for (size_t action_id = 0; action_id < step.actions.size(); ++action_id) {
      msgs::Action action = step.actions[action_id];
      if (action.type == Action::INFER_SPECIFICATION) {
        std::cout << "Infer action detected, end of demo steps: "
                  << new_program->steps.size() << "\n";
        return new_program->steps.size() > 0;
      }
      if (action.type == Action::MOVE_TO_CARTESIAN_GOAL &&
          action.landmark.name != robot_config_.torso_link()) {
        cart_pose_actions->push_back(std::make_pair(step_id, action_id));
      }
      // if (action.type == Action::ACTUATE_GRIPPER && release_step < 0) {
      //   // save the latest added step-action pair
      //   reference_pose = cart_pose_actions->back();
      //   release_step = step_id;
      //   release_action = action_id;
      //   std::cout << "* Pushed out Reference pose: " <<
      //   reference_pose.first
      //             << "," << reference_pose.second << "\n";
      // }
    }
    new_program->steps.push_back(step);
  }
  std::cout << "End of demo steps: " << new_program->steps.size()
            << " Cart actions found: " << cart_pose_actions->size() << "\n";
  return new_program->steps.size() > 0;
}

geometry_msgs::Vector3 Editor::QuaternionToRPY(
    // same as in ConditionGenerator
    const geometry_msgs::Quaternion& msg) {
  tf::Quaternion quat(msg.x, msg.y, msg.z, msg.w);
  // the tf::Quaternion has a method to access roll pitch and yaw
  double roll, pitch, yaw;
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  ROS_INFO("quaternion: %f,%f,%f,%f", msg.w, msg.x, msg.y, msg.z);
  // the found angles are written in a geometry_msgs::Vector3
  geometry_msgs::Vector3 rpy;
  rpy.x = roll * 180.0 / M_PI;
  rpy.y = pitch * 180.0 / M_PI;
  rpy.z = yaw * 180.0 / M_PI;

  ROS_INFO("euler: %f,%f,%f", rpy.x, rpy.y, rpy.z);
  return rpy;
}

bool Editor::CheckObjectCollision(const std::vector<msgs::Landmark>& landmarks,
                                  const msgs::Landmark& bounding_box) {
  // TO DO: Check if bounding box collides with any landmarks
}
bool Editor::CheckGripperSpaceEnough(
    const std::vector<msgs::Landmark>& landmarks,
    const geometry_msgs::Point& position, const msgs::Landmark& bounding_box) {
  // Checks if the target pose will not collide with existing landmarks
  // TO DO: Use Separating Axis Theorem for more accurate results
  // 1. check orientation of gripper, if > 45deg then swap dims.x and dims.y
  if (bounding_box.name == "") {
    std::cout << "No bounding box specified. \n";
    return false;
  }

  geometry_msgs::Vector3 bb_orientation =
      QuaternionToRPY(bounding_box.pose_stamped.pose.orientation);
  geometry_msgs::Vector3 bb_dims = bounding_box.surface_box_dims;
  if (fabs(fmod(bb_orientation.z, 180)) > 85) {
    std::cout << "Swapping bb_dims.x and y \n";
    float temp = bb_dims.x;
    bb_dims.x = bb_dims.y;
    bb_dims.y = temp;
  }
  std::cout << "Target Position: " << position << " \n";

  for (size_t i = 0; i < landmarks.size(); ++i) {
    msgs::Landmark lm = landmarks[i];
    // ABB TEST
    ROS_INFO("**Running AABB test...");
    msgs::Landmark gripper_box = bounding_box;
    gripper_box.pose_stamped.pose.position = position;
    bool aabbResult = AABBintersect(lm, gripper_box);
    std::cout << "AABB result is: " << aabbResult << " for Landmark " << lm.name
              << ". Press Enter to continue. \n";
    std::cin.ignore();

    geometry_msgs::Vector3 lm_orientation =
        QuaternionToRPY(lm.pose_stamped.pose.orientation);
    geometry_msgs::Vector3 lm_dims = lm.surface_box_dims;
    if (fabs(fmod(lm_orientation.z, 180)) > 85) {
      std::cout << "Swapping lm_dims.x and y \n";
      float temp = lm_dims.x;
      lm_dims.x = lm_dims.y;
      lm_dims.y = temp;
    }

    double dx = position.x - lm.pose_stamped.pose.position.x;
    double dy = position.y - lm.pose_stamped.pose.position.y;
    // double dz = position.z - lm.pose_stamped.pose.position.z;

    if (dx < (bb_dims.x + lm_dims.x) * 0.5 &&
        dy < (bb_dims.y + lm_dims.y) * 0.5) {
      std::cout << "Not enough gripper space, because of " << lm.name << "\n";
      return false;
    }
  }
  std::cout << "GripperSpaceEnough returned true. \n";
  return true;
}

bool Editor::AABBintersect(const msgs::Landmark& lm1,
                           const msgs::Landmark& lm2) {
  // AABB = axis aligned bounding boxes
  geometry_msgs::Vector3 min1, max1;
  geometry_msgs::Vector3 min2, max2;
  GetMinMax(lm1, &min1, &max1);
  GetMinMax(lm2, &min2, &max2);
  return (min1.x <= max2.x && max1.x >= min2.x) &&
         (min1.y <= max2.y && max1.y >= min2.y) &&
         (min1.z <= max2.z && max1.z >= min2.z);
}

void Editor::GetMinMax(const msgs::Landmark& lm, geometry_msgs::Vector3* min,
                       geometry_msgs::Vector3* max) {
  min->x = lm.pose_stamped.pose.position.x - lm.surface_box_dims.x * 0.5;
  min->y = lm.pose_stamped.pose.position.y - lm.surface_box_dims.y * 0.5;
  min->z = lm.pose_stamped.pose.position.z - lm.surface_box_dims.z * 0.5;
  max->x = lm.pose_stamped.pose.position.x - lm.surface_box_dims.x * 0.5;
  max->y = lm.pose_stamped.pose.position.y - lm.surface_box_dims.y * 0.5;
  max->z = lm.pose_stamped.pose.position.z - lm.surface_box_dims.z * 0.5;
}

bool Editor::CheckGridPositionFree(const std::vector<msgs::Landmark>& landmarks,
                                   const geometry_msgs::Point& position) {
  // TO DO: include object dims to check space properly

  // Checks if position is inside a landmark
  for (size_t i = 0; i < landmarks.size(); ++i) {
    msgs::Landmark lm = landmarks[i];
    double dx = position.x - lm.pose_stamped.pose.position.x;
    double dy = position.y - lm.pose_stamped.pose.position.y;
    // double dz = position.z - lm.pose_stamped.pose.position.z;
    double squared_distance = dx * dx + dy * dy;  // + dz * dz;

    double lm_diameter = fmin(lm.surface_box_dims.x, lm.surface_box_dims.y);
    if (squared_distance < lm_diameter * lm_diameter) {
      std::cout << "Position not free, blocked by " << lm.name << "\n";
      std::cout << "squ distance = " << squared_distance << " < ";
      std::cout << "lm_diameter = " << lm_diameter * lm_diameter << "\n";
      return false;
    }
  }
  return true;
}

void Editor::GetDemonstrationSteps(const msgs::Program& program,
                                   std::vector<msgs::Step>* demo_steps) {
  // Assume that demonstrations are separated by a DetectTTObject action
  for (size_t i = 0; i < program.steps.size(); ++i) {
    if (program.steps[i].actions.size() == 1) {
      if (program.steps[i].actions[0].type == Action::INFER_SPECIFICATION) {
        break;
      }
    }
    demo_steps->push_back(program.steps[i]);
  }
  std::cout << "new_program.steps.size = " << demo_steps->size() << "\n";
}

void Editor::AddStep(const std::string& db_id) {
  msgs::Program program;
  bool success = db_.Get(db_id, &program);
  if (!success) {
    ROS_ERROR("Unable to add step to program ID \"%s\"", db_id.c_str());
    return;
  }
  msgs::Step step;
  program.steps.push_back(step);
  Update(db_id, program);
}

void Editor::DeleteStep(const std::string& db_id, size_t step_id) {
  msgs::Program program;
  bool success = db_.Get(db_id, &program);
  if (!success) {
    ROS_ERROR("Unable to delete step from program ID \"%s\"", db_id.c_str());
    return;
  }
  if (step_id >= program.steps.size()) {
    ROS_ERROR(
        "Unable to delete step %ld from program \"%s\", which has %ld steps",
        step_id, db_id.c_str(), program.steps.size());
    return;
  }
  DeleteScene(program.steps[step_id].scene_id);
  program.steps.erase(program.steps.begin() + step_id);
  if (last_viewed_.find(db_id) != last_viewed_.end()) {
    if (last_viewed_[db_id] >= program.steps.size()) {
      last_viewed_[db_id] = program.steps.size() - 1;
    }
  }
  Update(db_id, program);
}

void Editor::AddAction(const std::string& db_id, size_t step_id,
                       msgs::Action action) {
  msgs::Program program;
  bool success = db_.Get(db_id, &program);
  if (!success) {
    ROS_ERROR("Unable to add action to program ID \"%s\"", db_id.c_str());
    return;
  }
  if (step_id >= program.steps.size()) {
    ROS_ERROR(
        "Unable to delete action from step %ld from program \"%s\", which "
        "has "
        "%ld steps",
        step_id, db_id.c_str(), program.steps.size());
    return;
  }
  msgs::Step* step = &program.steps[step_id];
  step->actions.insert(step->actions.begin(), action);
  if (action.type == Action::ACTUATE_GRIPPER && step_id > 0) {
    // if the gripper_box has not been assigned yet
    msgs::Step* cart_step = &program.steps[step_id - 1];
    // AssignGripperBoundingBox(cart_step, &program.gripper_box);

    World world;
    GetWorld(robot_config_, program, last_viewed_[db_id], &world);
    world.surface_box_landmarks.push_back(program.gripper_box);
    viz_.Publish(db_id, world);
  }
  Update(db_id, program);
}
void Editor::AssignGripperBoundingBox(msgs::Step* cart_step,
                                      msgs::Landmark* gripper_box) {
  // Assigns a landmark representing the gripper bounding box
  // Find move_to_cart action
  // TO DO: needs to be translated into torso tf
  ROS_INFO("Assigning gripper bounding box to program");
  for (size_t action_id = 0; action_id < cart_step->actions.size();
       ++action_id) {
    msgs::Action action = cart_step->actions[action_id];
    if (action.type == Action::MOVE_TO_CARTESIAN_GOAL) {
      ROS_INFO("Cart poses detected...");
      gripper_box->type = msgs::Landmark::SURFACE_BOX;
      gripper_box->name = "Gripper Box";
      gripper_box->surface_box_dims.x = 0.05;
      gripper_box->surface_box_dims.y = 0.1;
      gripper_box->surface_box_dims.z = 0.05;
      // needs to be in base_link frame, same as other world landmarks
      // TO DO: does it matter if it's base_link or torso_link?
      if (action.landmark.name != robot_config_.base_link()) {
        msgs::Landmark landmark;
        landmark.type = msgs::Landmark::TF_FRAME;
        landmark.name = robot_config_.base_link();
        ReinterpretPose(landmark, &action);
      }
      gripper_box->pose_stamped.header.frame_id = robot_config_.base_link();
      gripper_box->pose_stamped.pose = action.pose;
      return;
    }
  }
  ROS_INFO("No cart poses found for bounding box");
}

void Editor::CopyPDDLAction(const std::string& domain_id,
                            const std::string& action_name) {
  ROS_INFO("Trying to get domain id '%s' from db", domain_id.c_str());
  msgs::PDDLDomain domain;
  bool success = domain_db_.Get(domain_id, &domain);
  if (!success) {
    ROS_ERROR("Unable to get domain from \"%s\"", domain_id.c_str());
    return;
  }

  msgs::PDDLAction action;
  int index = FindPDDLAction(action_name, pddl_domain_.domain_.actions);
  if (index >= 0) {
    action = pddl_domain_.domain_.actions[index];
    ROS_INFO("Action found...");
    action.name += "-copy";

    // copy program

    msgs::Program program;
    success = db_.Get(action.program_id.c_str(), &program);
    if (!success) {
      ROS_ERROR("Unable to get program with ID \"%s\"",
                action.program_id.c_str());
      return;
    }
    program.name += "-copy";
    std::string new_id = db_.Insert(program);
    db_.StartPublishingProgramById(new_id);
    action.program_id = new_id;
    domain.actions.push_back(action);
    UpdatePDDLDomain(domain_id, domain);
  } else {
    ROS_ERROR("Could not save PDDL action named %s because it does not exist",
              action_name.c_str());
  }
}

void Editor::DeleteAction(const std::string& db_id, size_t step_id,
                          size_t action_id) {
  msgs::Program program;
  bool success = db_.Get(db_id, &program);
  if (!success) {
    ROS_ERROR("Unable to delete action from program ID \"%s\"", db_id.c_str());
    return;
  }
  if (step_id >= program.steps.size()) {
    ROS_ERROR(
        "Unable to delete action from step %ld from program \"%s\", which "
        "has "
        "%ld steps",
        step_id, db_id.c_str(), program.steps.size());
    return;
  }
  msgs::Step* step = &program.steps[step_id];
  if (action_id >= step->actions.size()) {
    ROS_ERROR(
        "Unable to delete action %ld from step %ld of program \"%s\", which "
        "has %ld actions",
        action_id, step_id, db_id.c_str(), step->actions.size());
    return;
  }

  // Clean up the scene and landmarks
  const msgs::Action& action = step->actions[action_id];
  if (action.type == msgs::Action::DETECT_TABLETOP_OBJECTS) {
    DeleteScene(step->scene_id);
    step->scene_id = "";
    DeleteLandmarks(msgs::Landmark::SURFACE_BOX, step);
  } else if (action.type == msgs::Action::FIND_CUSTOM_LANDMARK) {
    DeleteScene(step->scene_id);
    step->scene_id = "";
    DeleteLandmarks(msgs::Landmark::CUSTOM_LANDMARK, step);
  } else if (action.type == msgs::Action::CHECK_CONDITIONS) {
  } else if (action.type == msgs::Action::INFER_SPECIFICATION) {
    program.template_specs.clear();
    program.posteriors.data.clear();
    std::cout << "template_spec size " << program.template_specs.size() << "\n";
    std::cout << "posteriors size " << program.posteriors.data.size() << "\n";
    msgs::Specification spec;
    program.spec = spec;
  }
  step->actions.erase(step->actions.begin() + action_id);
  Update(db_id, program);
}

void Editor::ViewStep(const std::string& db_id, size_t step_id) {
  db_.StartPublishingProgramById(db_id);
  last_viewed_[db_id] = step_id;

  msgs::Program program;
  bool success = db_.Get(db_id, &program);
  if (!success) {
    ROS_ERROR("Unable to view program \"%s\"", db_id.c_str());
    return;
  }
  World world;
  GetWorld(robot_config_, program, last_viewed_[db_id], &world);
  viz_.Publish(db_id, world);
}

void Editor::DetectSurfaceObjects(const std::string& db_id, size_t step_id) {
  msgs::SegmentSurfacesGoal goal;
  goal.save_cloud = true;
  action_clients_->surface_segmentation_client.sendGoal(goal);
  bool success = action_clients_->surface_segmentation_client.waitForResult(
      ros::Duration(50));
  if (!success) {
    ROS_ERROR("Failed to segment surface.");
    return;
  }
  msgs::SegmentSurfacesResult::ConstPtr result =
      action_clients_->surface_segmentation_client.getResult();

  msgs::Program program;
  success = db_.Get(db_id, &program);
  if (!success) {
    ROS_ERROR("Unable to update scene for program ID \"%s\"", db_id.c_str());
    return;
  }
  if (step_id >= program.steps.size()) {
    ROS_ERROR(
        "DetectSurfaceObjects: Unable to update scene for step %ld, program "
        "\"%s\", which has %ld "
        "steps",
        step_id, db_id.c_str(), program.steps.size());
    return;
  }
  DeleteScene(program.steps[step_id].scene_id);
  program.steps[step_id].scene_id = result->cloud_db_id;
  program.steps[step_id].surface = result->surface;
  DeleteLandmarks(msgs::Landmark::SURFACE_BOX, &program.steps[step_id]);

  for (size_t i = 0; i < result->landmarks.size(); ++i) {
    msgs::Landmark landmark;
    ProcessSurfaceBox(result->landmarks[i], &landmark);
    program.steps[step_id].landmarks.push_back(landmark);
    // std::cout << i << "th landmark: " << landmark.pose_stamped.pose << "\n";
  }
  Update(db_id, program);
}

void Editor::SaveOnExit(const std::string& db_id,
                        const std::string& action_name) {
  msgs::Program program;
  bool success = db_.Get(db_id, &program);
  if (!success) {
    ROS_ERROR("Unable to delete program ID \"%s\"", db_id.c_str());
    return;
  }
  if (program.steps.size() > 0) {
    msgs::PDDLAction action;
    int index = FindPDDLAction(action_name, pddl_domain_.domain_.actions);
    if (index >= 0) {
      action = pddl_domain_.domain_.actions[index];
      ROS_INFO("Action found...");
    } else {
      ROS_ERROR("Could not save PDDL action named %s because it does not exist",
                action_name.c_str());
    }
  }
  Update(db_id, program);
}

void Editor::UpdatePDDLDomain(const std::string& domain_id,
                              const msgs::PDDLDomain& domain) {
  domain_db_.Update(domain_id, domain);
  pddl_domain_.PublishPDDLDomain(domain);
}

void Editor::DetectActionConditions(const std::string& domain_id,
                                    const std::string& action_name,
                                    const std::string& state_name) {
  ROS_INFO("DetectActionConditions... '%s'", action_name.c_str());
  // look for pddl domain
  msgs::PDDLDomain domain;
  bool success = domain_db_.Get(domain_id, &domain);
  if (!success) {
    ROS_ERROR("Unable to get domain from \"%s\"", domain_id.c_str());
    return;
  }

  // look for pddl action
  int index = FindPDDLAction(action_name, domain.actions);
  if (index < 0) {
    ROS_ERROR("Pddl action called %s not found", action_name.c_str());
    return;
  } else {
    msgs::PDDLAction new_action = domain.actions[index];
    // detect surface landmarks
    msgs::SegmentSurfacesGoal goal;
    goal.save_cloud = true;

    action_clients_->surface_segmentation_client.sendGoal(goal);
    success = action_clients_->surface_segmentation_client.waitForResult(
        ros::Duration(50));
    if (!success) {
      ROS_ERROR("Failed to segment surface.");
      return;
    }
    msgs::SegmentSurfacesResult::ConstPtr result =
        action_clients_->surface_segmentation_client.getResult();

    // save scene_id and surface for later
    new_action.scene_id = result->cloud_db_id;
    new_action.surface = result->surface;
    new_action.landmarks.clear();
    ROS_INFO("Saved action scene_id '%s'", new_action.scene_id.c_str());
    // std::vector<msgs::Landmark> world_landmarks;
    for (size_t i = 0; i < result->landmarks.size(); ++i) {
      msgs::Landmark landmark;
      ProcessSurfaceBox(result->landmarks[i], &landmark);
      new_action.landmarks.push_back(landmark);
    }

    WorldState world_state;
    GetWorldState(new_action.landmarks, &world_state);
    new_action.params = world_state.objects_;
    if (state_name == "Precondition") {
      new_action.preconditions = world_state.predicates_;
      ROS_INFO("Updated action %s", state_name.c_str());
    } else if (state_name == "Effect") {
      new_action.effects = world_state.predicates_;
      ROS_INFO("Updated action %s", state_name.c_str());
    } else {
      ROS_ERROR("Unknown condition type: %s", state_name.c_str());
    }

    PrintAllPredicates(world_state.predicates_, "");
    UpdatePDDLAction(domain_id, new_action, "");

    msgs::Program program;
    success = db_.Get(new_action.program_id, &program);
    if (!success) {
      ROS_ERROR("Unable to update scene for program ID \"%s\"",
                new_action.program_id.c_str());
      return;
    }
    Update(new_action.program_id, program);
  }
}

void Editor::AssignSurfaceObjects(const std::string& db_id,
                                  const msgs::PDDLAction& action,
                                  const std::string& state_name,
                                  size_t step_id) {
  // Assigns detected surface objects to step
  msgs::Program program;
  bool success = db_.Get(db_id, &program);
  if (!success) {
    ROS_ERROR("Unable to update scene for program ID \"%s\"", db_id.c_str());
    return;
  }
  if (state_name == "Effect") {
    step_id = program.steps.size() - 1;
    ROS_INFO("Updating effect, so set step_id to last step: %ld ", step_id);
  }

  if (step_id >= program.steps.size()) {
    ROS_ERROR(
        "AssignSurfaceObjects: Unable to update scene for step %ld, program "
        "\"%s\", which has %ld "
        "steps",
        step_id, db_id.c_str(), program.steps.size());
    return;
  }
  DeleteScene(program.steps[step_id].scene_id);
  program.steps[step_id].scene_id = action.scene_id;
  program.steps[step_id].surface = action.surface;
  DeleteLandmarks(msgs::Landmark::SURFACE_BOX, &program.steps[step_id]);

  for (size_t i = 0; i < action.landmarks.size(); ++i) {
    msgs::Landmark landmark;
    ProcessSurfaceBox(action.landmarks[i], &landmark);
    program.steps[step_id].landmarks.push_back(landmark);
  }
  Update(db_id, program);
}
// PDDL Action functions
void Editor::AddPDDLAction(const std::string& domain_id,
                           const std::string& action_name) {
  ROS_INFO("Adding pddl action: %s", action_name.c_str());
  domain_db_.StartPublishingPDDLDomainById(domain_id);

  ROS_INFO("Trying to get domain id '%s' from db", domain_id.c_str());
  msgs::PDDLDomain domain;
  bool success = domain_db_.Get(domain_id, &domain);
  if (!success) {
    ROS_ERROR("Unable to get domain from \"%s\"", domain_id.c_str());
    return;
  }
  int index = FindPDDLAction(action_name, domain.actions);
  if (index >= 0) {
    ROS_INFO("Pddl action '%s' already exists", action_name.c_str());
  }
  ROS_INFO("Creating new pddl action '%s'", action_name.c_str());
  msgs::PDDLAction action;
  action.name = action_name;
  domain.actions.push_back(action);
  UpdatePDDLDomain(domain_id, domain);
}

void Editor::UpdatePDDLAction(const std::string& domain_id,
                              const msgs::PDDLAction& action,
                              const std::string& action_name) {
  ROS_INFO("UpdatePDDLAction... '%s'", action_name.c_str());
  msgs::PDDLDomain domain;
  bool success = domain_db_.Get(domain_id, &domain);
  if (!success) {
    ROS_ERROR("Unable to get domain from \"%s\"", domain_id.c_str());
    return;
  }

  int index = FindPDDLAction(action.name, domain.actions);
  if (index < 0) {
    ROS_INFO("Pddl action %s does not exist but will be added",
             action.name.c_str());
    domain.actions.push_back(action);
  } else {
    domain.actions.at(index) = action;
  }
  // Update action name
  if (action_name != "") {
    domain.actions[index].name = action_name;
  }

  UpdatePDDLDomain(domain_id, domain);
}

void Editor::DeletePDDLAction(const std::string& domain_id,
                              const std::string& action_name) {
  msgs::PDDLDomain domain;
  bool success = domain_db_.Get(domain_id, &domain);
  if (!success) {
    ROS_ERROR("Unable to get domain from \"%s\"", domain_id.c_str());
    return;
  }
  msgs::PDDLAction action;
  int index = FindPDDLAction(action_name, domain.actions);
  if (index < 0) {
    ROS_ERROR("Unable to find pddl action %s for domain %s",
              action_name.c_str(), domain_id.c_str());
  } else {
  }
  // deleting the program linked to the action
  Delete(domain.actions[index].program_id);

  domain.actions.erase(domain.actions.begin() + index);

  UpdatePDDLDomain(domain_id, domain);
}

int Editor::FindPDDLAction(const std::string name,
                           const std::vector<msgs::PDDLAction>& actions) {
  ROS_INFO("Start looking for action '%s' out of %zu actions", name.c_str(),
           actions.size());
  for (int i = 0; i < actions.size(); ++i) {
    if (strcasecmp(actions[i].name.c_str(), name.c_str()) == 0) {
      return i;
    }
  }
  return -1;
}

// PDDL Problems functions

void Editor::DetectWorldState(const std::string& domain_id,
                              const std::string& problem_name,
                              const std::string& state_name) {
  // Detect TT objects and assigns them to the PDDL problem initial/goal state
  // look for pddl domain
  msgs::PDDLDomain domain;
  bool success = domain_db_.Get(domain_id, &domain);
  if (!success) {
    ROS_ERROR("Unable to get domain from \"%s\"", domain_id.c_str());
    return;
  }

  // look for pddl problem
  int index = FindPDDLProblem(problem_name, domain.problems);
  if (index < 0) {
    ROS_ERROR("Pddl problem called %s not found", problem_name.c_str());
    return;
  } else {
    msgs::PDDLProblem new_problem = domain.problems[index];
    // detect surface landmarks
    msgs::SegmentSurfacesGoal goal;
    goal.save_cloud = true;
    action_clients_->surface_segmentation_client.sendGoal(goal);
    success = action_clients_->surface_segmentation_client.waitForResult(
        ros::Duration(50));
    if (!success) {
      ROS_ERROR("Failed to segment surface.");
      return;
    }
    msgs::SegmentSurfacesResult::ConstPtr result =
        action_clients_->surface_segmentation_client.getResult();

    // save scene_id and surface for later
    new_problem.scene_id = result->cloud_db_id;
    new_problem.surface = result->surface;
    new_problem.landmarks.clear();
    for (size_t i = 0; i < result->landmarks.size(); ++i) {
      msgs::Landmark landmark;
      ProcessSurfaceBox(result->landmarks[i], &landmark);
      new_problem.landmarks.push_back(landmark);
    }

    WorldState world_state;
    GetWorldState(new_problem.landmarks, &world_state);
    new_problem.objects = world_state.objects_;
    if (state_name == "initial") {
      new_problem.initial_states = world_state.predicates_;
      ROS_INFO("Updated problem %s", state_name.c_str());
    } else if (state_name == "goal") {
      new_problem.goal_states = world_state.predicates_;
      ROS_INFO("Updated problem %s", state_name.c_str());
    } else {
      ROS_ERROR("Unknown condition type: %s", state_name.c_str());
    }

    PrintAllPredicates(world_state.predicates_, "");
    UpdatePDDLProblem(domain_id, new_problem, "");
  }
}

void Editor::AddPDDLProblem(const std::string& domain_id,
                            const std::string& problem_name) {
  ROS_INFO("Start add pddl problem: %s", problem_name.c_str());
  domain_db_.StartPublishingPDDLDomainById(domain_id);

  ROS_INFO("Trying to get %s from db", domain_id.c_str());
  msgs::PDDLDomain domain;
  bool success = domain_db_.Get(domain_id, &domain);
  if (!success) {
    ROS_ERROR("Unable to get domain from \"%s\"", domain_id.c_str());
    return;
  }
  int index = FindPDDLProblem(problem_name, domain.problems);
  if (index >= 0) {
    ROS_INFO("Pddl problem '%s' already exists", problem_name.c_str());
  }
  ROS_INFO("Creating new pddl problem '%s'", problem_name.c_str());
  msgs::PDDLProblem problem;
  problem.name = problem_name;
  domain.problems.push_back(problem);
  UpdatePDDLDomain(domain_id, domain);
}

void Editor::UpdatePDDLProblem(const std::string& domain_id,
                               const msgs::PDDLProblem& problem,
                               const std::string& problem_name) {
  msgs::PDDLDomain domain;
  bool success = domain_db_.Get(domain_id, &domain);
  if (!success) {
    ROS_ERROR("Unable to get domain from \"%s\"", domain_id.c_str());
    return;
  }

  int index = FindPDDLProblem(problem.name, domain.problems);
  if (index < 0) {
    ROS_INFO("Pddl problem %s does not exist but will be added",
             problem.name.c_str());
    domain.problems.push_back(problem);
  } else {
    domain.problems.at(index) = problem;
  }
  // Update problem name
  if (problem_name != "") {
    domain.problems[index].name = problem_name;
  }

  UpdatePDDLDomain(domain_id, domain);
}
void Editor::DeletePDDLProblem(const std::string& domain_id,
                               const std::string& problem_name) {
  msgs::PDDLDomain domain;
  bool success = domain_db_.Get(domain_id, &domain);
  if (!success) {
    ROS_ERROR("Unable to get domain from \"%s\"", domain_id.c_str());
    return;
  }
  msgs::PDDLProblem problem;
  int index = FindPDDLProblem(problem_name, domain.problems);
  if (index < 0) {
    ROS_ERROR("Unable to find pddl problem %s for domain %s",
              problem_name.c_str(), domain_id.c_str());
  } else {
  }
  domain.problems.erase(domain.problems.begin() + index);

  UpdatePDDLDomain(domain_id, domain);
}

int Editor::FindPDDLProblem(const std::string name,
                            const std::vector<msgs::PDDLProblem>& problems) {
  ROS_INFO("Start looking for problem %s out of %zu problems", name.c_str(),
           problems.size());
  for (int i = 0; i < problems.size(); ++i) {
    if (problems[i].name == name) {
      return i;
    }
  }
  return -1;
}

void Editor::SolvePDDLProblem(const std::string domain_id,
                              const msgs::PDDLProblem& problem,
                              const std::string planner) {
  ROS_INFO("Solve problem %s with planner %s", problem.name.c_str(),
           planner.c_str());
  // look for pddl domain
  msgs::PDDLDomain domain;
  bool success = domain_db_.Get(domain_id, &domain);
  if (!success) {
    ROS_ERROR("Unable to get domain from \"%s\"", domain_id.c_str());
    return;
  }
  // create pddl_msgs::PDDLDomain element
  pddl_msgs::PDDLDomain planner_domain;
  planner_domain.name = domain.name;
  planner_domain.requirements = domain.requirements;

  ROS_INFO("Got domain %s", planner_domain.name.c_str());
  // get types
  for (size_t i = 0; i < domain.types.size(); ++i) {
    std::string new_type = domain.types[i].name;
    if (domain.types[i].parent != "")
      new_type += " - " + domain.types[i].parent;
    planner_domain.types.push_back(new_type);
    ROS_INFO("Added type: %s", planner_domain.types[i].c_str());
  }

  // get predicates
  for (size_t i = 0; i < domain.predicates.size(); ++i) {
    planner_domain.predicates.push_back(
        PrintPDDLPredicate(domain.predicates[i], "predicate"));
    ROS_INFO("Added predicate: %s", planner_domain.predicates[i].c_str());
  }
  // get actions
  for (size_t i = 0; i < domain.actions.size(); ++i) {
    msgs::PDDLAction action = domain.actions[i];

    pddl_msgs::PDDLAction planner_action;
    planner_action.name = action.name;
    ROS_INFO("Adding action: %s", planner_action.name.c_str());
    // get parameters as single string
    std::stringstream param_ss;
    for (size_t j = 0; j < action.params.size(); ++j) {
      param_ss << "?" << action.params[j].name << " - "
               << action.params[j].type.name << " ";
    }
    planner_action.parameters = "(" + param_ss.str() + ")";

    ROS_INFO("action params: %s", planner_action.parameters.c_str());
    // get preconditions as single string
    planner_action.precondition =
        PrintAllPredicates(action.preconditions, "precondition");

    ROS_INFO("action pre: %s", planner_action.precondition.c_str());
    planner_action.effect = PrintAllPredicates(action.effects, "effect");

    ROS_INFO("action eff: %s", planner_action.effect.c_str());
    planner_domain.actions.push_back(planner_action);
  }

  // create pddl_msgs::PDDLProblem element
  pddl_msgs::PDDLProblem planner_problem;
  planner_problem.name = problem.name;
  planner_problem.domain = domain.name;
  ROS_INFO("Get Problem: %s for Domain '%s'", planner_problem.name.c_str(),
           planner_problem.domain.c_str());
  // get objects
  for (size_t i = 0; i < problem.objects.size(); ++i) {
    pddl_msgs::PDDLObject planner_object;
    std::string str = problem.objects[i].name;
    str.erase(remove_if(str.begin(), str.end(), isspace), str.end());

    planner_object.name = str;
    planner_object.type = problem.objects[i].type.name;
    planner_problem.objects.push_back(planner_object);

    ROS_INFO("added object: %s", str.c_str());
  }
  // get initial states
  for (size_t i = 0; i < problem.initial_states.size(); ++i) {
    planner_problem.initial.push_back(
        PrintPDDLPredicate(problem.initial_states[i], "init"));
    ROS_INFO("added initial state: %s", planner_problem.initial[i].c_str());
  }
  planner_problem.goal = PrintAllPredicates(problem.goal_states, "goal");

  ROS_INFO("added goal: %s", planner_problem.goal.c_str());
  pddl_msgs::PDDLPlannerGoal goal;
  goal.domain = planner_domain;
  goal.problem = planner_problem;

  ROS_INFO("sending goal to solver: %s", planner_problem.goal.c_str());
  action_clients_->pddl_solver_client.sendGoal(goal);
  success =
      action_clients_->pddl_solver_client.waitForResult(ros::Duration(30));
  if (!success) {
    ROS_INFO("No plan found for problem %s", problem.name.c_str());
  }
  pddl_msgs::PDDLPlannerResult::ConstPtr result =
      action_clients_->pddl_solver_client.getResult();
  ROS_INFO("Plan found for problem \"%s\" with %zu data", problem.name.c_str(),
           result->data.size());
  ROS_INFO("...with %zu steps", result->sequence.size());
  msgs::PDDLProblem new_problem = problem;
  new_problem.sequence.clear();
  for (size_t i = 0; i < result->sequence.size(); ++i) {
    pddl_msgs::PDDLStep step = result->sequence[i];
    new_problem.sequence.push_back(step);
    ROS_INFO("...%s", step.action.c_str());
  }
  UpdatePDDLProblem(domain_id, new_problem, "");
}
void Editor::RunPDDLPlan(const std::string domain_id,
                         const msgs::PDDLProblem& problem,
                         const std::string planner) {
  ROS_INFO("Execute (%zu steps) plan for problem %s with planner %s",
           problem.sequence.size(), problem.name.c_str(), planner.c_str());
  // look for pddl domain
  msgs::PDDLDomain domain;
  bool success = domain_db_.Get(domain_id, &domain);
  if (!success) {
    ROS_ERROR("Unable to get domain from \"%s\"", domain_id.c_str());
    return;
  }
  for (size_t i = 0; i < problem.sequence.size(); ++i) {
    pddl_msgs::PDDLStep step = problem.sequence[i];
    ROS_INFO("Step %zu: Action '%s'", i, step.action.c_str());
    // Look for action in PDDL domain
    std::string action_name = step.action;
    msgs::PDDLAction action;
    int index = FindPDDLAction(action_name, pddl_domain_.domain_.actions);
    if (index < 0) {
      ROS_ERROR("Could not save PDDL action named %s because it does not exist",
                action_name.c_str());
    }
    ROS_INFO("Action found...");
    action = pddl_domain_.domain_.actions[index];
    // 1. Look for associated program_id (db_id)
    std::string db_id;
    db_id = action.program_id;
    msgs::Program main_program;
    bool success = db_.Get(db_id, &main_program);
    if (!success) {
      ROS_ERROR("Unable to submit program \"%s\"", db_id.c_str());
      return;
    }

    // 3. Run associated program for this step
    // 3.1 Create new program that will be modified and run for this step
    msgs::Program new_program = main_program;
    msgs::Program alt_program = main_program;

    // 3.2 save 'move to cart pose' action/step no. in an array that are
    // relative to a landmark (not torso)
    std::vector<std::pair<int, int> > cart_pose_actions;
    new_program.steps.clear();
    bool newProgramSuccess =
        GetCartActions(&cart_pose_actions, alt_program, &new_program);
    if (!newProgramSuccess) {
      ROS_ERROR("No cartesian actions found in program \"%s\"",
                alt_program.name.c_str());
      return;
    }
    // 3.3 update the cart action's relative landmark's dimensions according to
    // the matching PDDLaction parameter dimension
    for (size_t id = 0; id < cart_pose_actions.size(); ++id) {
      int s_id = cart_pose_actions[id].first;
      int a_id = cart_pose_actions[id].second;
      std::string lm_name = new_program.steps[s_id].actions[a_id].landmark.name;

      // find argument in action that corresponds to the relative lm name
      std::cout << "Looking for matching lm: " << lm_name << "\n";
      for (size_t z = 0; z < action.params.size(); ++z) {
        std::cout << "action param: " << action.params[z].name << "\n";
        std::cout << "# problems landmarks: " << problem.landmarks.size()
                  << "\n";

        if (lm_name == action.params[z].name) {
          std::cout << "action param: matched ! " << action.params[z].name
                    << "\n";
          // find landmark object in list of detected landmarks in problem
          for (size_t l = 0; l < problem.landmarks.size(); ++l) {
            msgs::Landmark match = problem.landmarks[l];
            if (strcasecmp(match.name.c_str(), step.args[z].c_str()) == 0) {
              std::cout << "step param: matched ! " << step.args[z] << "\n";
              new_program.steps[s_id]
                  .actions[a_id]
                  .landmark.surface_box_dims.x = match.surface_box_dims.x;
              new_program.steps[s_id]
                  .actions[a_id]
                  .landmark.surface_box_dims.y = match.surface_box_dims.y;
              new_program.steps[s_id]
                  .actions[a_id]
                  .landmark.surface_box_dims.z = match.surface_box_dims.z;
              new_program.steps[s_id].actions[a_id].landmark.pose_stamped =
                  match.pose_stamped;

              std::cout << "Updated dimension of step/action = " << s_id << ","
                        << a_id << "  and landmark " << lm_name << " ("
                        << new_program.steps[s_id]
                               .actions[a_id]
                               .landmark.surface_box_dims.x
                        << " ,"
                        << new_program.steps[s_id]
                               .actions[a_id]
                               .landmark.surface_box_dims.y
                        << " ,"
                        << new_program.steps[s_id]
                               .actions[a_id]
                               .landmark.surface_box_dims.z
                        << ") \n";
            }
          }
        }
      }
    }

  // run program
  RUN:
    std::cout << "Running program for action ..." << action_name << "\n";
    msgs::ExecuteProgramGoal goal;
    goal.program = new_program;
    action_clients_->program_client.sendGoal(goal);
    bool finished_before_timeout =
        action_clients_->program_client.waitForResult(ros::Duration(30.0));
    if (!finished_before_timeout) {
      ROS_INFO("Program did not finish before the time out.");
    }
    msgs::ExecuteProgramResult::ConstPtr result =
        action_clients_->program_client.getResult();

    ROS_INFO(
        "%s done with: %s \n Press enter to r to try again or c to continue.",
        action_name.c_str(), result->error.c_str());
    std::string n;
    std::cin >> n;
    if (n == "r") goto RUN;

    // 4. Check action effects after executing action
    // TO DO: if effects are not satisfied stop action execution
  }
}

// *** adding check conditions action after table top detection

// msgs::Program new_program = main_program;
// int step_id = 0;
// for (size_t k = 0; k < program.steps.size(); ++k) {
//   msgs::Step step = program.steps[step_id];
//   for (size_t action_id = 0; action_id < step.actions.size(); ++action_id) {
//     msgs::Action action = step.actions[action_id];

//     if (action.type == Action::MOVE_TO_CARTESIAN_GOAL &&
//         action.landmark.name != robot_config_.torso_link()) {
//       cart_pose_actions->push_back(std::make_pair(step_id, action_id));
//     }
//     if (action.type == Action::DETECT_TABLETOP_OBJECTS) {
//       new_program->steps.push_back(step);
//       ++step_id;
//       step.actions.clear();
//       action_id = 0;
//       // 3.2. Add check conditions action before executing action
//       if (action_name == msgs::PDDLPredicate::IS_CLEAR &&
//           action.params.size() == 1) {
//         // check if position is clear
//       } else if (action_name == msgs::PDDLPredicate::IS_ON &&
//                  action.params.size() == 2) {
//         // check if object is on position
//         AddCheckConditionsAction(db_id, step_id);
//         msgs::Condition action_condition =
//             step->actions[action_id].condition;
//         cond_gen_.AssignLandmarkCondition(initial_world, landmark_name,
//                                           &action_condition);
//         program.steps[step_id].actions[action_id].condition =
//             action_condition;
//       }
//     }
//   }
//   new_program->steps.push_back(step);
//   ++step_id;
// }

// rapid_pbd functions
void Editor::GetJointValues(const std::string& db_id, size_t step_id,
                            size_t action_id,
                            const std::string& actuator_group) {
  msgs::Program program;
  bool success = db_.Get(db_id, &program);
  if (!success) {
    ROS_ERROR("Unable to update action from program ID \"%s\"", db_id.c_str());
    return;
  }
  if (step_id >= program.steps.size()) {
    ROS_ERROR(
        "Unable to update action from step %ld from program \"%s\", which "
        "has "
        "%ld steps",
        step_id, db_id.c_str(), program.steps.size());
    return;
  }
  msgs::Step* step = &program.steps[step_id];
  if (action_id >= step->actions.size()) {
    ROS_ERROR(
        "Unable to update action %ld from step %ld of program \"%s\", which "
        "has %ld actions",
        action_id, step_id, db_id.c_str(), step->actions.size());
    return;
  }

  msgs::Action* action = &step->actions[action_id];
  action->actuator_group = actuator_group;

  std::vector<std::string> joint_names;
  robot_config_.joints_for_group(actuator_group, &joint_names);
  if (joint_names.size() == 0) {
    ROS_ERROR("Can't get joint angles for actuator group \"%s\"",
              action->actuator_group.c_str());
    return;
  }

  std::vector<double> joint_positions;
  for (size_t i = 0; i < joint_names.size(); ++i) {
    const std::string& name = joint_names[i];
    double pos = joint_state_reader_.get_position(name);
    if (pos == kNoJointValue) {
      ROS_ERROR("Could not get angle for joint \"%s\"", name.c_str());
      joint_positions.push_back(0);
    } else {
      joint_positions.push_back(pos);
    }
  }

  SetJointPositions(joint_names, joint_positions, action);

  // Fill in default time
  if (action->joint_trajectory.points[0].time_from_start.isZero()) {
    action->joint_trajectory.points[0].time_from_start.sec = 3;
  }

  // Clear any previous landmark.
  msgs::Landmark blank_landmark;
  action->landmark = blank_landmark;

  Update(db_id, program);
}

void Editor::GetPose(const std::string& db_id, size_t step_id, size_t action_id,
                     const std::string& actuator_group,
                     const msgs::Landmark& landmark) {
  msgs::Program program;
  bool success = db_.Get(db_id, &program);
  if (!success) {
    ROS_ERROR("Unable to get action from program ID \"%s\"", db_id.c_str());
    return;
  }
  if (step_id >= program.steps.size()) {
    ROS_ERROR(
        "Unable to get action from step %ld from program \"%s\", which has "
        "%ld steps",
        step_id, db_id.c_str(), program.steps.size());
    return;
  }
  msgs::Step* step = &program.steps[step_id];
  if (action_id >= step->actions.size()) {
    ROS_ERROR(
        "Unable to get action %ld from step %ld of program \"%s\", which "
        "has %ld actions",
        action_id, step_id, db_id.c_str(), step->actions.size());
    return;
  }

  msgs::Action* action = &step->actions[action_id];
  action->actuator_group = actuator_group;

  // If the landmark is empty or the same as before, then update the action's
  // pose.
  // If the landmark has changed, then reinterpret the action's pose in the
  // new landmark frame.
  if (action->landmark.type == "" || landmark.type == "" ||
      action->landmark.type == landmark.type) {
    size_t prev_step_id = 0;
    if (step_id > 0) {
      prev_step_id = step_id - 1;
    }
    World world;
    GetWorld(robot_config_, program, prev_step_id, &world);
    ROS_INFO("%lu Actuator group: %s", prev_step_id, actuator_group.c_str());
    GetNewPose(landmark, world, actuator_group, action);
  } else {
    ReinterpretPose(landmark, action);
  }
  Update(db_id, program);
}

// Gets the current pose of the end-effector relative to the given landmark.
// action.pose and action.landmark are mutated.
void Editor::GetNewPose(const msgs::Landmark& landmark, const World& world,
                        const std::string& actuator_group,
                        msgs::Action* action) {
  // Get transform from landmark to end-effector.
  transform_graph::Graph graph;

  // Get transform of end-effector relative to base.
  tf::StampedTransform transform;
  try {
    std::string ee_frame = robot_config_.ee_frame_for_group(actuator_group);
    if (ee_frame == "") {
      ROS_ERROR("Unable to get pose for actuator group: \"%s\"",
                actuator_group.c_str());
    }
    tf_listener_.lookupTransform(robot_config_.base_link(), ee_frame,
                                 ros::Time(0), transform);
  } catch (tf::TransformException ex) {
    ROS_ERROR("GetNewPose tf: %s", ex.what());
    return;
  }
  graph.Add("end effector",
            transform_graph::RefFrame(robot_config_.base_link()), transform);

  // If the action does not have a pre-existing landmark, then find the
  // closest landmark. Otherwise, use the provided landmark.
  if (action->landmark.type == "" || landmark.type == "") {
    double distance_cutoff = 0.4;
    ros::param::param("distance_cutoff", distance_cutoff, 0.4);
    double squared_cutoff = distance_cutoff * distance_cutoff;

    geometry_msgs::Vector3 ee_position;
    ee_position.x = transform.getOrigin().x();
    ee_position.y = transform.getOrigin().y();
    ee_position.z = transform.getOrigin().z();
    msgs::Landmark closest;

    if (ClosestLandmark(ee_position, world, squared_cutoff, &closest)) {
      action->landmark = closest;
    } else {
      action->landmark.type = msgs::Landmark::TF_FRAME;
      action->landmark.name = robot_config_.torso_link();
    }
  } else {
    action->landmark = landmark;
  }

  // Get transform of landmark relative to base.
  if (action->landmark.type == msgs::Landmark::TF_FRAME) {
    tf::StampedTransform landmark_transform;
    try {
      tf_listener_.lookupTransform(robot_config_.base_link(),
                                   action->landmark.name, ros::Time(0),
                                   landmark_transform);
    } catch (tf::TransformException ex) {
      ROS_ERROR("GetNewPose tf2: %s", ex.what());
      return;
    }
    graph.Add("landmark", transform_graph::RefFrame(robot_config_.base_link()),
              landmark_transform);
    action->landmark.pose_stamped.header.frame_id = robot_config_.base_link();
    transform_graph::Transform landmark_tf(landmark_transform);
    landmark_tf.ToPose(&action->landmark.pose_stamped.pose);
  } else if (action->landmark.type == msgs::Landmark::SURFACE_BOX) {
    std::string landmark_frame(action->landmark.pose_stamped.header.frame_id);
    if (landmark_frame != robot_config_.base_link()) {
      ROS_WARN("Landmark not in base frame.");
    }
    graph.Add("landmark", transform_graph::RefFrame(landmark_frame),
              action->landmark.pose_stamped.pose);
  } else {
    ROS_ERROR("Unsupported landmark type \"%s\"",
              action->landmark.type.c_str());
    return;
  }

  transform_graph::Transform ee_in_landmark;
  bool success = graph.ComputeDescription(
      transform_graph::LocalFrame("end effector"),
      transform_graph::RefFrame("landmark"), &ee_in_landmark);
  if (!success) {
    ROS_ERROR("Unable to transform end-effector pose into landmark!");
  }
  ee_in_landmark.ToPose(&action->pose);

  // Set joint angles as a seed.
  std::vector<std::string> joint_names;
  robot_config_.joints_for_group(actuator_group, &joint_names);
  if (joint_names.size() == 0) {
    ROS_ERROR("Can't get joint angles for actuator group \"%s\"",
              action->actuator_group.c_str());
    return;
  }

  std::vector<double> joint_positions;
  for (size_t i = 0; i < joint_names.size(); ++i) {
    const std::string& name = joint_names[i];
    double pos = joint_state_reader_.get_position(name);
    if (pos == kNoJointValue) {
      ROS_ERROR("Could not get angle for joint \"%s\"", name.c_str());
      joint_positions.push_back(0);
    } else {
      joint_positions.push_back(pos);
    }
  }
  SetJointPositions(joint_names, joint_positions, action);
}

// Reinterpret the existing pose to be relative to the given landmark.
// Assumes as a precondition that action->pose is not empty.
// action->pose and action->landmark are mutated.
void Editor::ReinterpretPose(const msgs::Landmark& new_landmark,
                             msgs::Action* action) {
  transform_graph::Graph graph;
  graph.Add("end effector", transform_graph::RefFrame("old landmark"),
            action->pose);

  if (action->landmark.type == msgs::Landmark::TF_FRAME) {
    tf::StampedTransform landmark_transform;
    try {
      tf_listener_.lookupTransform(robot_config_.base_link(),
                                   action->landmark.name, ros::Time(0),
                                   landmark_transform);
    } catch (tf::TransformException ex) {
      ROS_ERROR("ReinterpretPose tf: %s", ex.what());
      return;
    }
    graph.Add("old landmark",
              transform_graph::RefFrame(robot_config_.base_link()),
              landmark_transform);
  } else if (action->landmark.type == msgs::Landmark::SURFACE_BOX) {
    std::string landmark_frame(action->landmark.pose_stamped.header.frame_id);
    if (landmark_frame != robot_config_.base_link()) {
      ROS_WARN("Landmark not in base frame.");
    }
    graph.Add("old landmark", transform_graph::RefFrame(landmark_frame),
              action->landmark.pose_stamped.pose);

  } else {
    ROS_ERROR("Unsupported landmark type \"%s\"",
              action->landmark.type.c_str());
    return;
  }

  // Add the new landmark
  action->landmark = new_landmark;

  if (action->landmark.type == msgs::Landmark::TF_FRAME) {
    tf::StampedTransform landmark_transform;
    try {
      tf_listener_.lookupTransform(robot_config_.base_link(),
                                   action->landmark.name, ros::Time(0),
                                   landmark_transform);
    } catch (tf::TransformException ex) {
      ROS_ERROR("ReinterpretPose tf: %s", ex.what());
      return;
    }
    graph.Add("new landmark",
              transform_graph::RefFrame(robot_config_.base_link()),
              landmark_transform);
    action->landmark.pose_stamped.header.frame_id = robot_config_.base_link();
    transform_graph::Transform landmark_tf(landmark_transform);
    landmark_tf.ToPose(&action->landmark.pose_stamped.pose);
  } else if (action->landmark.type == msgs::Landmark::SURFACE_BOX) {
    std::string landmark_frame(action->landmark.pose_stamped.header.frame_id);
    if (landmark_frame != robot_config_.base_link()) {
      ROS_WARN("Landmark not in base frame.");
    }
    graph.Add("new landmark", transform_graph::RefFrame(landmark_frame),
              action->landmark.pose_stamped.pose);

  } else {
    ROS_ERROR("Unsupported landmark type \"%s\"",
              action->landmark.type.c_str());
    return;
  }

  // Update the pose
  transform_graph::Transform ee_in_new_landmark;
  bool success = graph.ComputeDescription(
      transform_graph::LocalFrame("end effector"),
      transform_graph::RefFrame("new landmark"), &ee_in_new_landmark);

  if (!success) {
    ROS_ERROR("Unable to transform end-effector pose into new landmark!");
    return;
  }
  ee_in_new_landmark.ToPose(&action->pose);
}

bool Editor::ClosestLandmark(const geometry_msgs::Vector3& ee_position,
                             const World& world,
                             const double squared_distance_cutoff,
                             msgs::Landmark* landmark) {
  bool success = false;
  double closest_distance = std::numeric_limits<double>::max();
  for (size_t i = 0; i < world.surface_box_landmarks.size(); ++i) {
    const msgs::Landmark& world_landmark = world.surface_box_landmarks[i];
    geometry_msgs::Vector3 world_pos;
    world_pos.x = world_landmark.pose_stamped.pose.position.x;
    world_pos.y = world_landmark.pose_stamped.pose.position.y;
    world_pos.z = world_landmark.pose_stamped.pose.position.z;
    double dx = world_pos.x - ee_position.x;
    double dy = world_pos.y - ee_position.y;
    double dz = world_pos.z - ee_position.z;
    double squared_distance = dx * dx + dy * dy + dz * dz;
    if (squared_distance < closest_distance &&
        squared_distance <= squared_distance_cutoff) {
      *landmark = world_landmark;
      closest_distance = squared_distance;
      success = true;
    }
  }
  return success;
}

void Editor::DeleteScene(const std::string& scene_id) {
  if (scene_id == "") {
    return;
  }
  bool success = scene_db_.Delete(scene_id);
  if (!success) {
    ROS_ERROR("Failed to delete scene ID: \"%s\"", scene_id.c_str());
  }
}

void Editor::DeleteLandmarks(const std::string& landmark_type,
                             msgs::Step* step) {
  std::vector<msgs::Landmark> cleaned;
  for (size_t i = 0; i < step->landmarks.size(); ++i) {
    const msgs::Landmark& landmark = step->landmarks[i];
    if (landmark.type != landmark_type) {
      cleaned.push_back(landmark);
    }
  }
  step->landmarks = cleaned;
}
}  // namespace pbd
}  // namespace rapid
