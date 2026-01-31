#ifndef YARNPATH_SURFACE_NODE_HPP
#define YARNPATH_SURFACE_NODE_HPP

#include <math/vec3.hpp>
#include <yarn_path.hpp>
#include <cstdint>

namespace yarnpath {

using NodeId = uint32_t;

// A node in the surface relaxation graph.
// Each node corresponds to one YarnSegment from the YarnPath.
struct SurfaceNode {
    NodeId id = 0;
    SegmentId segment_id = 0;     // Back-reference to YarnPath

    Vec3 position;                // Current position
    Vec3 velocity;                // For Verlet integration
    Vec3 force;                   // Accumulated forces

    float mass = 1.0f;            // Node mass (grams) for gravity calculation

    bool is_pinned = false;       // Fixed position (won't move during relaxation)
    bool forms_loop = false;      // Cached from YarnSegment

    // Reset forces for a new iteration
    void clear_force() {
        force = Vec3::zero();
    }

    // Add a force to the accumulated force
    void add_force(const Vec3& f) {
        force += f;
    }
};

}  // namespace yarnpath

#endif // YARNPATH_SURFACE_NODE_HPP
