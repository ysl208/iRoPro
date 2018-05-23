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
  std::vector<msgs::PDDLObject> objects_;
  // predicates describing object relations
  std::vector<msgs::PDDLPredicate> predicates_;

  std::vector<msgs::PDDLObject> positions_;
};

struct Domain {
 public:
  std::string domain_name;
  std::vector<msgs::PDDLDomain> domain_;
};

// void GetDomain(Domain* domain);
void InitDomain(Domain* domain);
void GetWorldState(const World& world, WorldState* world_state);
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
void GetFixedPositions(std::vector<msgs::PDDLObject>* objects);
bool GetObjectTablePosition(const msgs::PDDLType& obj, WorldState* world_state,
                            const double squared_distance_cutoff,
                            msgs::PDDLObject* found_position);
void PrintAllPredicates(std::vector<msgs::PDDLPredicate> predicates,
                        std::string type);
std::string PrintPredicate(msgs::PDDLPredicate predicate);
std::string PrintPDDLPredicate(msgs::PDDLPredicate predicate);
}  // namespace pbd
}  // namespace rapid

#endif  // _RAPID_PBD_WORLD_H_
