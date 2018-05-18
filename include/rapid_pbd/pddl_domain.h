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

namespace rapid {
namespace pbd {
struct WorldState {
 public:
  // positions, objects, grippers
  std::vector<rapid_pbd_msgs::PDDLObject> objects_;
  // predicates describing object relations
  std::vector<rapid_pbd_msgs::PDDLPredicate> predicates_;
};

struct Domain {
 public:
  std::string domain_name;
  std::vector<rapid_pbd_msgs::PDDLDomain> domain_;

 private:
  std::vector<geometry_msgs::Vector3> positions_;
};

// void GetDomain(Domain* domain);
void InitDomain(Domain* domain);
void GetWorldState(const World& world, WorldState* world_state);
void GetTypeFromDims(const geometry_msgs::Vector3& dims, std::string type);
void GetFixedPositions(std::vector<rapid_pbd_msgs::PDDLObject>* objects);

}  // namespace pbd
}  // namespace rapid

#endif  // _RAPID_PBD_WORLD_H_
