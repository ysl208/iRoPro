#ifndef _RAPID_PBD_PDDL_DOMAIN_H_
#define _RAPID_PBD_PDDL_DOMAIN_H_
#include <string>
#include <vector>

#include "rapid_pbd_msgs/Landmark.h"
#include "rapid_pbd_msgs/PDDLAction.h"
#include "rapid_pbd_msgs/PDDLDomain.h"
#include "rapid_pbd_msgs/PDDLObject.h"
#include "rapid_pbd_msgs/PDDLPredicate.h"
#include "rapid_pbd_msgs/Program.h"

#include "rapid_pbd/world.h"
#include "ros/ros.h"

namespace msgs = rapid_pbd_msgs;
namespace rapid {
namespace pbd {
struct WorldState {
 public:
  // positions, objects, grippers
  std::vector<msgs::PDDLObject> objects_;
  // predicates describing object relations
  std::vector<msgs::PDDLPredicate> predicates_;

  std::vector<msgs::PDDLObject> positions_;
};

class PDDLDomain {
 public:
  msgs::PDDLDomain domain_;
  std::string domain_id;
  ros::Publisher pddl_domain_pub_;

  PDDLDomain(const ros::Publisher& pub);
  void Init(msgs::PDDLDomain* domain, const std::string& name);
  // void GetDomain(Domain* domain);
  void PublishPDDLDomain(const msgs::PDDLDomain& domain);
};

void GetWorldState(const std::vector<msgs::Landmark>& world_landmarks,
                   WorldState* world_state);
void AddType(std::vector<msgs::PDDLType>* types,
             const msgs::PDDLType& new_type);
void AddObject(std::vector<msgs::PDDLObject>* objects,
               const msgs::PDDLObject& new_obj);
bool ObjectExists(std::vector<msgs::PDDLObject>* objects,
                  const std::string& name);
bool PredicateExists(std::vector<msgs::PDDLPredicate>* predicates,
                     const std::string& predicate,
                     const std::vector<msgs::PDDLObject>& args);
void AddPredicate(std::vector<msgs::PDDLPredicate>* predicates,
                  const std::string& predicate,
                  const std::vector<msgs::PDDLObject>& args, bool negate);
void GetTypeFromDims(const geometry_msgs::Vector3& dims,
                     msgs::PDDLType* obj_type);
bool GetObjectTablePosition(const msgs::PDDLType& obj, WorldState* world_state,
                            const double squared_distance_cutoff,
                            msgs::PDDLObject* found_position);
std::string PrintAllPredicates(std::vector<msgs::PDDLPredicate> predicates,
                               std::string type);
std::string PrintPredicate(msgs::PDDLPredicate predicate);
std::string PrintPDDLPredicate(msgs::PDDLPredicate predicate,
                               std::string pred_type);
}  // namespace pbd
}  // namespace rapid

#endif  // _RAPID_PBD_WORLD_H_
