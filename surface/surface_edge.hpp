#ifndef YARNPATH_SURFACE_EDGE_HPP
#define YARNPATH_SURFACE_EDGE_HPP

#include "surface_node.hpp"
#include <cstdint>

namespace yarnpath {

using EdgeId = uint32_t;

// Type of edge connection
enum class EdgeType {
    YarnContinuity,  // Between consecutive segments (i -> i+1), represents yarn backbone
    PassThrough      // Between segment and loop in its 'through' vector
};

// An edge in the surface relaxation graph.
// Edges represent spring connections between nodes.
struct SurfaceEdge {
    EdgeId id = 0;
    NodeId node_a = 0;
    NodeId node_b = 0;

    EdgeType type = EdgeType::YarnContinuity;
    float rest_length = 1.0f;     // Natural length of the spring
    float stiffness = 1.0f;       // Spring constant (higher = stiffer)
};

}  // namespace yarnpath

#endif // YARNPATH_SURFACE_EDGE_HPP
