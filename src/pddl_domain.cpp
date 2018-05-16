#include "rapid_pbd/pddl_domain.h"

#include <math.h>
#include <set>
#include <string>
#include <vector>

#include "geometry_msgs/Pose.h"
#include "moveit_msgs/GetPositionIK.h"
#include "rapid_pbd_msgs/Action.h"
#include "rapid_pbd_msgs/Landmark.h"
#include "rapid_pbd_msgs/Program.h"
#include "rapid_pbd_msgs/Step.h"
#include "transform_graph/graph.h"

#include "rapid_pbd/action_utils.h"
#include "rapid_pbd/joint_state.h"
#include "rapid_pbd/landmarks.h"
#include "rapid_pbd/robot_config.h"

#include "ros/ros.h"

namespace msgs = rapid_pbd_msgs;

namespace rapid {
namespace pbd {
void GetPDDLDomain(const RobotConfig& robot_config, const msgs::Program& program,
               size_t step_id, PDDLDomain* domain) {}

}  // namespace pbd
}  // namespace rapid
