#ifndef YARNPATH_SURFACE_CONSTRAINT_HPP
#define YARNPATH_SURFACE_CONSTRAINT_HPP

#include "surface_node.hpp"
#include <cstdint>

namespace yarnpath {

using ConstraintId = uint32_t;

// Type of constraint
enum class ConstraintType {
    MaxStretch,   // Yarn cannot stretch beyond (1 + elasticity) * rest_length
    MinDistance   // Non-adjacent segments maintain minimum separation
};

// A constraint in the surface relaxation graph.
// Constraints are projected after integration to maintain physical validity.
struct SurfaceConstraint {
    ConstraintId id = 0;
    ConstraintType type = ConstraintType::MaxStretch;
    NodeId node_a = 0;
    NodeId node_b = 0;
    float limit = 1.0f;           // Maximum stretch ratio or minimum distance
};

}  // namespace yarnpath

#endif // YARNPATH_SURFACE_CONSTRAINT_HPP
