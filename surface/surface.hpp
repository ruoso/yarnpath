#ifndef YARNPATH_SURFACE_HPP
#define YARNPATH_SURFACE_HPP

// Surface layer public API
// Physics-based relaxation of yarn path topology to equilibrium positions

#include "surface_graph.hpp"
#include "surface_node.hpp"
#include "surface_edge.hpp"
#include "surface_constraint.hpp"
#include "surface_builder.hpp"
#include "surface_forces.hpp"
#include "surface_solver.hpp"

#include <math/vec3.hpp>
#include <yarn/yarn_properties.hpp>
#include <yarn/gauge.hpp>
#include <yarn_path.hpp>
#include <vector>
#include <string>

namespace yarnpath {

// Result of surface relaxation: equilibrium positions for each segment
class RelaxedSurface {
public:
    RelaxedSurface() = default;

    // Extract from a solved graph
    static RelaxedSurface from_graph(const SurfaceGraph& graph);

    // Access position by segment ID
    Vec3 position(SegmentId id) const;

    // Access all positions (indexed by SegmentId)
    const std::vector<Vec3>& positions() const { return positions_; }

    // Number of segments
    size_t segment_count() const { return positions_.size(); }

    // Check if segment exists
    bool has_segment(SegmentId id) const {
        return id < positions_.size();
    }

    // Bounding box of all positions
    std::pair<Vec3, Vec3> bounding_box() const;

    // Center of mass
    Vec3 center_of_mass() const;

private:
    std::vector<Vec3> positions_;  // Indexed by SegmentId
};

// High-level convenience function: relax a yarn path to equilibrium
// This is the main entry point for the surface module
//
// Usage:
//   YarnPath yarn_path = YarnPath::from_stitch_graph(graph);
//   YarnProperties yarn = YarnProperties::worsted();
//   Gauge gauge = Gauge::worsted();
//
//   RelaxedSurface surface = relax_surface(yarn_path, yarn, gauge);
//
//   // Get position of a specific segment
//   Vec3 pos = surface.position(segment_id);
//
RelaxedSurface relax_surface(
    const YarnPath& path,
    const YarnProperties& yarn,
    const Gauge& gauge,
    const SurfaceBuildConfig& build_config = SurfaceBuildConfig{},
    const SolveConfig& solve_config = SolveConfig{}
);

// Advanced: build and solve separately for more control
//
// Usage:
//   SurfaceGraph graph = SurfaceBuilder::from_yarn_path(path, yarn, gauge, build_config);
//   SolveResult result = SurfaceSolver::solve(graph, yarn, solve_config);
//   RelaxedSurface surface = RelaxedSurface::from_graph(graph);
//

// Configuration for OBJ export
struct SurfaceObjConfig {
    float node_radius = 0.5f;        // Radius of spheres representing nodes
    int sphere_subdivisions = 1;      // Icosphere subdivisions (0=20 faces, 1=80, 2=320)
    bool show_continuity = true;      // Show yarn continuity edges as lines
    bool show_passthrough = true;     // Show passthrough edges as lines
    bool color_by_type = true;        // Different colors for loop vs non-loop nodes
};

// Export surface graph to OBJ format for visualization
// - Nodes are rendered as small spheres (icospheres)
// - Edges are rendered as lines
std::string surface_to_obj(
    const SurfaceGraph& graph,
    const SurfaceObjConfig& config = SurfaceObjConfig{}
);

}  // namespace yarnpath

#endif // YARNPATH_SURFACE_HPP
