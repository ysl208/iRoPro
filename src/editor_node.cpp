#include "mongodb_store/message_store.h"
#include "rapid_pbd/action_clients.h"
#include "rapid_pbd/condition_generator.h"
#include "rapid_pbd/db_names.h"
#include "rapid_pbd/editor.h"
#include "rapid_pbd/joint_state_reader.h"
#include "rapid_pbd/program_db.h"
#include "rapid_pbd/robot_config.h"
#include "rapid_pbd/visualizer.h"
#include "rapid_pbd_msgs/PDDLDomain.h"
#include "rapid_pbd_msgs/PDDLDomainInfoList.h"
#include "rapid_pbd_msgs/ProgramInfoList.h"
#include "robot_markers/builder.h"
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "urdf/model.h"
#include "visualization_msgs/MarkerArray.h"

namespace pbd = rapid::pbd;
namespace msgs = rapid_pbd_msgs;

int main(int argc, char** argv) {
  ros::init(argc, argv, "rapid_pbd_editor_node");
  ros::NodeHandle nh;

  // Build robot config.
  std::string robot("");
  bool is_robot_specified = ros::param::get("robot", robot);
  if (!is_robot_specified) {
    ROS_ERROR("robot param must be specified.");
    return 1;
  }

  pbd::RobotConfig* robot_config;
  if (robot == "pr2") {
    robot_config = new pbd::Pr2RobotConfig();
  } else if (robot == "fetch") {
    robot_config = new pbd::FetchRobotConfig();
  } else if (robot == "baxter") {
    robot_config = new pbd::BaxterRobotConfig();
  } else if (robot == "ur5") {
    robot_config = new pbd::UniversalRobotConfig();
  } else {
    ROS_ERROR("Unsupported robot \"%s\"", robot.c_str());
    return 1;
  }

  // Build program DB.
  mongodb_store::MessageStoreProxy proxy(nh, pbd::kMongoProgramCollectionName,
                                         pbd::kMongoDbName);
  mongodb_store::MessageStoreProxy pddl_proxy(nh, pbd::kMongoPddlCollectionName,
                                              pbd::kMongoDbName);
  mongodb_store::MessageStoreProxy* scene_proxy =
      new mongodb_store::MessageStoreProxy(nh, pbd::kMongoSceneCollectionName,
                                           pbd::kMongoDbName);
  ros::Publisher program_list_pub =
      nh.advertise<msgs::ProgramInfoList>(pbd::kProgramListTopic, 1, true);
  ros::Publisher pddl_list_pub = nh.advertise<msgs::PDDLDomainInfoList>(
      pbd::kPDDLDomainListTopic, 1, true);
  // Build DBs.
  pbd::ProgramDb db(nh, &proxy, &program_list_pub);
  pbd::SceneDb scene_db(scene_proxy);
  pbd::PDDLDomainDb domain_db(nh, &pddl_proxy, &pddl_list_pub);

  // Build action clients.
  pbd::ActionClients action_clients;
  while (!action_clients.surface_segmentation_client.waitForServer(
             ros::Duration(5)) &&
         ros::ok()) {
    ROS_WARN("Waiting for surface segmentation server.");
  }
  // while (!action_clients.surface_segmentation_client.waitForServer(
  //            ros::Duration(5)) &&
  //        ros::ok()) {
  //   ROS_WARN("Waiting for pddl planner server.");
  // }
  // PDDL domain publisher publishes domain_id that is currently used
  ros::Publisher pddl_domain_pub =
      nh.advertise<std_msgs::String>("pddl_domain", 5, true);

  // Build visualizer
  urdf::Model model;
  model.initParam("robot_description");
  robot_markers::Builder marker_builder(model);
  pbd::Visualizer visualizer(scene_db, marker_builder, *robot_config);
  visualizer.Init();

  pbd::ConditionGenerator cond_gen(*robot_config);

  pbd::SpecInference spec_inf(*robot_config);
  // Build editor.
  pbd::JointStateReader joint_state_reader(robot_config->joint_states_topic());
  pbd::Editor editor(db, scene_db, domain_db, joint_state_reader, visualizer,
                     &action_clients, cond_gen, spec_inf, pddl_domain_pub,
                     *robot_config);
  editor.Start();

  ros::Subscriber editor_sub = nh.subscribe(pbd::kEditorEventsTopic, 10,
                                            &pbd::Editor::HandleEvent, &editor);

  ros::ServiceServer editor_server = nh.advertiseService(
      "create_program", &pbd::Editor::HandleCreateProgram, &editor);

  ros::ServiceServer editor_server_domain = nh.advertiseService(
      "create_domain", &pbd::Editor::HandleCreatePDDLDomain, &editor);

  ROS_INFO("RapidPBD editor ready.");
  ros::spin();
  if (robot_config) {
    delete robot_config;
  }
  return 0;
}
