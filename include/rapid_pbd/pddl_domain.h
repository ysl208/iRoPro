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

namespace msgs = rapid_pbd_msgs;
namespace rapid {
namespace pbd {
struct WorldState {
 public:
  // positions, objects, grippers
  std::vector<rapid_pbd_msgs::PDDLObject> objects_;
  // predicates describing object relations
  std::vector<rapid_pbd_msgs::PDDLPredicate> predicates_;

  std::vector<rapid_pbd_msgs::PDDLObject> positions_;
};

struct Domain {
 public:
  std::string domain_name;
  std::vector<rapid_pbd_msgs::PDDLDomain> domain_;
};

// void GetDomain(Domain* domain);
void InitDomain(Domain* domain);
void GetWorldState(const World& world, WorldState* world_state);
bool PredicateExists(std::vector<rapid_pbd_msgs::PDDLPredicate> predicates,
                     const std::string& predicate,
                     const std::vector<rapid_pbd_msgs::PDDLObject>& args);
void AddPredicate(std::vector<rapid_pbd_msgs::PDDLPredicate> predicates,
                  const std::string& predicate,
                  const std::vector<rapid_pbd_msgs::PDDLObject>& args,
                  bool negate);
void GetTypeFromDims(const geometry_msgs::Vector3& dims,
                     rapid_pbd_msgs::PDDLType* obj_type);
void GetFixedPositions(std::vector<rapid_pbd_msgs::PDDLObject>* objects);
bool GetObjectTablePosition(const rapid_pbd_msgs::PDDLType& obj,
                            WorldState* world_state,
                            const double squared_distance_cutoff,
                            rapid_pbd_msgs::PDDLObject* found_position);

}  // namespace pbd
}  // namespace rapid

#endif  // _RAPID_PBD_WORLD_H_
