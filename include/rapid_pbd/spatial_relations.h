// Constants for the names of our actions and the names of the actions in the
// robot APIs.

#ifndef _RAPID_PBD_SPATIAL_RELATIONS_H_
#define _RAPID_PBD_SPATIAL_RELATIONS_H_

namespace rapid {
namespace pbd {

static const char leftSpatialRelation[] = "is_left_of";
static const char rightSpatialRelation[] = "is_right_of";
static const char frontSpatialRelation[] = "is_in_front_of";
static const char behindSpatialRelation[] = "is_behind_of";

static const char aboveSpatialRelation[] = "is_above";
static const char belowSpatialRelation[] = "is_below";
static const char insideSpatialRelation[] = "is_inside_of";

static const char touchSpatialRelation[] = "is_touching";

}  // namespace pbd
}  // namespace rapid
#endif  // _RAPID_PBD_SPATIAL_RELATIONS_H_
