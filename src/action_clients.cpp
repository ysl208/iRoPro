#include "rapid_pbd/action_clients.h"

#include "rapid_pbd/action_names.h"

namespace rapid {
namespace pbd {
ActionClients::ActionClients()
    : gripper_client(kGripperActionName, true),
      l_gripper_client(kLeftGripperActionName, true),
      r_gripper_client(kRightGripperActionName, true),
      head_client(kHeadActionName, true),
      surface_segmentation_client(kSurfaceSegmentationActionName, true),
      pddl_solver_client(kPDDLSolverActionName, true),
      moveit_client(kMoveGroupActionName, true),
      program_client(kProgramActionName, true) {}
}  // namespace pbd
}  // namespace rapid