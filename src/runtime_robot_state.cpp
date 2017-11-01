#include "rapid_pbd/runtime_robot_state.h"

#include "tf/transform_listener.h"

#include "rapid_pbd/joint_state_reader.h"
#include "rapid_pbd/robot_config.h"

namespace rapid {
namespace pbd {
RuntimeRobotState::RuntimeRobotState(const RobotConfig& config,
                                     const tf::TransformListener& tf_listener,
                                     const JointStateReader& js_reader)
    : config(config), tf_listener(tf_listener), js_reader(js_reader) {}
}  // namespace pbd
}  // namespace rapid
