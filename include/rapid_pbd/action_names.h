// Constants for the names of our actions and the names of the actions in the
// robot APIs.

#ifndef _RAPID_PBD_ACTION_NAMES_H_
#define _RAPID_PBD_ACTION_NAMES_H_

namespace rapid {
namespace pbd {
// Names without a leading slash will be prepended with /rapid_pbd/ in the
// launch files.
static const char kProgramActionName[] = "execute_program_action";

static const char kGripperActionName[] = "gripper_action";
static const char kLeftGripperActionName[] = "l_gripper_action";
static const char kRightGripperActionName[] = "r_gripper_action";

static const char kArmJointActionName[] = "arm_joint_action";
static const char kLeftArmJointActionName[] = "l_arm_joint_action";
static const char kRightArmJointActionName[] = "r_arm_joint_action";

static const char kArmControllerStateTopic[] = "arm_controller_state";
static const char kFreezeArmService[] = "freeze_arm";
static const char kRelaxArmService[] = "relax_arm";

static const char kHeadActionName[] = "head_action";

static const char kMoveGroupActionName[] = "move_group";

static const char kSurfaceSegmentationActionName[] = "segment_surfaces_action";
static const char kPDDLSolverActionName[] = "pddl_solver_action";

namespace pr2 {
static const char kLeftGripperActionName[] =
    "/l_gripper_controller/gripper_action";
// rostopics for l_gripper controller - same as head
// Type: pr2_controllers_msgs/Pr2GripperCommandActionGoal
// subscribers: /l_gripper_controller/gripper_action_node
// node publishes:  /l_gripper_controller/command
// [pr2_controllers_msgs/Pr2GripperCommand]

static const char kRightGripperActionName[] =
    "/r_gripper_controller/gripper_action";
static const char kListControllersService[] =
    "/pr2_controller_manager/list_controllers";
static const char kSwitchControllerService[] =
    "/pr2_controller_manager/switch_controller";
static const char kHeadActionName[] =
    "/head_traj_controller/follow_joint_trajectory";
// rostopics for head traj when loading sim
// type: control_msgs/FollowJointTrajectoryActionGoal

// /head_traj_controller/follow_joint_trajectory/cancel
// /head_traj_controller/follow_joint_trajectory/feedback
// /head_traj_controller/follow_joint_trajectory/goal
// /head_traj_controller/follow_joint_trajectory/result
// /head_traj_controller/follow_joint_trajectory/status
}  // namespace pr2

namespace fetch {
static const char kControllerActionName[] = "/query_controller_states";
}  // namespace fetch

namespace baxter {
static const char kLeftGripperActionName[] =
    "/robot/end_effector/left_gripper/gripper_action";
static const char kRightGripperActionName[] =
    "/robot/end_effector/right_gripper/gripper_action";
static const char kHeadActionName[] = "/robot/head/head_action";
}  // namespace baxter

namespace ur {
static const char kControllerActionName[] = "/arm_controller/query_state";
}  // namespace ur

}  // namespace pbd
}  // namespace rapid

#endif  // _RAPID_PBD_ACTION_NAMES_H_
