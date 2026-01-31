#ifndef YARNPATH_SURFACE_GRAPH_HPP
#define YARNPATH_SURFACE_GRAPH_HPP

#include "surface_node.hpp"
#include "surface_edge.hpp"
#include "surface_constraint.hpp"
#include <vector>
#include <unordered_map>

namespace yarnpath {

// Container for the complete surface relaxation graph.
// Holds all nodes, edges, and constraints.
class SurfaceGraph {
public:
    SurfaceGraph() = default;

    // Node management
    NodeId add_node(const SurfaceNode& node);
    SurfaceNode& node(NodeId id);
    const SurfaceNode& node(NodeId id) const;
    size_t node_count() const { return nodes_.size(); }
    std::vector<SurfaceNode>& nodes() { return nodes_; }
    const std::vector<SurfaceNode>& nodes() const { return nodes_; }

    // Edge management
    EdgeId add_edge(const SurfaceEdge& edge);
    SurfaceEdge& edge(EdgeId id);
    const SurfaceEdge& edge(EdgeId id) const;
    size_t edge_count() const { return edges_.size(); }
    std::vector<SurfaceEdge>& edges() { return edges_; }
    const std::vector<SurfaceEdge>& edges() const { return edges_; }

    // Constraint management
    ConstraintId add_constraint(const SurfaceConstraint& constraint);
    SurfaceConstraint& constraint(ConstraintId id);
    const SurfaceConstraint& constraint(ConstraintId id) const;
    size_t constraint_count() const { return constraints_.size(); }
    std::vector<SurfaceConstraint>& constraints() { return constraints_; }
    const std::vector<SurfaceConstraint>& constraints() const { return constraints_; }

    // Lookup node by segment ID
    NodeId node_for_segment(SegmentId seg_id) const;
    bool has_segment(SegmentId seg_id) const;

    // Get edges connected to a node
    std::vector<EdgeId> edges_for_node(NodeId node_id) const;

    // Compute total system energy (for convergence checking)
    float compute_energy() const;

    // Clear all forces on all nodes
    void clear_all_forces();

    // Build adjacency index for fast neighbor lookup (O(1) instead of O(E))
    void build_adjacency_index();

    // Get continuity neighbors (prev/next along yarn path)
    // Returns {prev_node_id, next_node_id}, where -1 indicates no neighbor
    std::pair<NodeId, NodeId> get_continuity_neighbors(NodeId node) const;

    // Build constraint color sets for parallel projection
    // Partitions constraints into independent sets using graph coloring
    void build_constraint_colors();

    // Get constraints grouped by color (each color is an independent set)
    const std::vector<std::vector<ConstraintId>>& constraint_colors() const {
        return constraint_colors_;
    }

    // Check if constraint coloring has been built
    bool has_constraint_colors() const { return !constraint_colors_.empty(); }

private:
    std::vector<SurfaceNode> nodes_;
    std::vector<SurfaceEdge> edges_;
    std::vector<SurfaceConstraint> constraints_;

    // Mapping from SegmentId to NodeId for quick lookup
    std::unordered_map<SegmentId, NodeId> segment_to_node_;

    // Adjacency index: node_id -> (prev_node_id, next_node_id)
    // -1 indicates no neighbor in that direction
    std::vector<std::pair<NodeId, NodeId>> continuity_neighbors_;
    bool adjacency_built_ = false;

    // Constraints grouped by color (each color is an independent set)
    // Constraints in the same color don't share any nodes
    std::vector<std::vector<ConstraintId>> constraint_colors_;
};

}  // namespace yarnpath

#endif // YARNPATH_SURFACE_GRAPH_HPP
