#ifndef YARNPATH_SURFACE_BUILDER_HPP
#define YARNPATH_SURFACE_BUILDER_HPP

#include "surface_graph.hpp"
#include <yarn/yarn_properties.hpp>
#include <yarn/gauge.hpp>
#include <yarn_path.hpp>
#include <cstdint>
#include <map>
#include <vector>

namespace yarnpath {

// Configuration for building a surface graph
struct SurfaceBuildConfig {
    // Random seed for initial positions
    uint32_t random_seed = 42;

    // Noise amplitude added to grid positions (fraction of stitch_width)
    float position_noise = 0.1f;

    // Rest length multipliers (applied to gauge-derived values)
    float continuity_rest_length_factor = 1.0f;
    float passthrough_rest_length_factor = 1.0f;

    // Stiffness multipliers (applied to yarn.stiffness-derived values)
    // Base stiffness comes from YarnProperties::stiffness
    float continuity_stiffness_factor = 1.0f;
    float passthrough_stiffness_factor = 0.5f;  // Passthroughs are typically less stiff
};

// Builder for creating a SurfaceGraph from YarnPath topology
class SurfaceBuilder {
public:
    // Build a surface graph from yarn path topology
    static SurfaceGraph from_yarn_path(
        const YarnPath& path,
        const YarnProperties& yarn,
        const Gauge& gauge,
        const SurfaceBuildConfig& config = SurfaceBuildConfig{}
    );

private:
    SurfaceBuilder(const YarnPath& path, const YarnProperties& yarn,
                   const Gauge& gauge, const SurfaceBuildConfig& config);

    void create_nodes();
    void create_continuity_edges();
    void create_passthrough_edges();
    void create_constraints();
    void initialize_positions();

    // Helper methods for incremental position initialization
    bool check_node_constraints(
        NodeId node_id,
        const std::map<std::pair<NodeId, NodeId>, float>& continuity_lengths,
        const std::map<NodeId, std::vector<std::pair<NodeId, float>>>& passthrough_info);

    void relax_partial(
        NodeId up_to_node,
        const std::map<std::pair<NodeId, NodeId>, float>& continuity_lengths,
        const std::map<NodeId, std::vector<std::pair<NodeId, float>>>& passthrough_info);

    void resolve_collisions(
        NodeId node_id,
        const std::map<NodeId, std::vector<std::pair<NodeId, float>>>& passthrough_info);

    // Compute dominant plane normal from placed nodes (0 to up_to_node inclusive)
    Vec3 compute_plane_normal(NodeId up_to_node) const;

    // Check and fix angular constraint for node triplet (prev2 -> prev -> current)
    // Ensures the bend angle doesn't exceed max_bend_angle relative to plane normal
    // Returns true if angle was acceptable, false if correction was needed
    bool check_and_fix_angle(NodeId node_id, const Vec3& plane_normal, float max_bend_angle);

    const YarnPath& path_;
    const YarnProperties& yarn_;
    const Gauge& gauge_;
    SurfaceBuildConfig config_;
    SurfaceGraph graph_;
};

}  // namespace yarnpath

#endif // YARNPATH_SURFACE_BUILDER_HPP
