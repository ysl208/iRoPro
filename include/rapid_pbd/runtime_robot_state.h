#ifndef _RAPID_PBD_RUNTIME_ROBOT_STATE_H_
#define _RAPID_PBD_RUNTIME_ROBOT_STATE_H_

#include "tf/transform_listener.h"

#include "rapid_pbd/joint_state_reader.h"
#include "rapid_pbd/robot_config.h"

namespace rapid {
namespace pbd {
// RuntimeRobotState contains objects for getting information about the robot at
// runtime. This is designed to be a convenience class that can be passed
// around, since these objects tend to be grouped together.
struct RuntimeRobotState {
 public:
  RuntimeRobotState(const RobotConfig& config,
                    const tf::TransformListener& tf_listener,
                    const JointStateReader& js_reader);
  const RobotConfig& config;
  const tf::TransformListener& tf_listener;
  const JointStateReader& js_reader;
};
}  // namespace pbd
}  // namespace rapid

#endif  // _RAPID_PBD_RUNTIME_ROBOT_STATE_H_
