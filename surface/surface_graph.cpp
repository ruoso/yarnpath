#include "surface_graph.hpp"
#include "logging.hpp"
#include <stdexcept>
#include <algorithm>
#include <set>
#ifdef _OPENMP
#include <omp.h>
#endif

namespace yarnpath {

NodeId SurfaceGraph::add_node(const SurfaceNode& node) {
    NodeId id = static_cast<NodeId>(nodes_.size());
    SurfaceNode new_node = node;
    new_node.id = id;
    nodes_.push_back(new_node);
    segment_to_node_[new_node.segment_id] = id;
    return id;
}

SurfaceNode& SurfaceGraph::node(NodeId id) {
    if (id >= nodes_.size()) {
        throw std::out_of_range("SurfaceGraph::node: invalid node id");
    }
    return nodes_[id];
}

const SurfaceNode& SurfaceGraph::node(NodeId id) const {
    if (id >= nodes_.size()) {
        throw std::out_of_range("SurfaceGraph::node: invalid node id");
    }
    return nodes_[id];
}

EdgeId SurfaceGraph::add_edge(const SurfaceEdge& edge) {
    EdgeId id = static_cast<EdgeId>(edges_.size());
    SurfaceEdge new_edge = edge;
    new_edge.id = id;
    edges_.push_back(new_edge);

    // Add to filtered collections by type for efficient iteration
    if (edge.type == EdgeType::YarnContinuity) {
        continuity_edge_ids_.push_back(id);
    } else if (edge.type == EdgeType::PassThrough) {
        passthrough_edge_ids_.push_back(id);
    }

    return id;
}

SurfaceEdge& SurfaceGraph::edge(EdgeId id) {
    if (id >= edges_.size()) {
        throw std::out_of_range("SurfaceGraph::edge: invalid edge id");
    }
    return edges_[id];
}

const SurfaceEdge& SurfaceGraph::edge(EdgeId id) const {
    if (id >= edges_.size()) {
        throw std::out_of_range("SurfaceGraph::edge: invalid edge id");
    }
    return edges_[id];
}

NodeId SurfaceGraph::node_for_segment(SegmentId seg_id) const {
    auto it = segment_to_node_.find(seg_id);
    if (it == segment_to_node_.end()) {
        throw std::out_of_range("SurfaceGraph::node_for_segment: segment not found");
    }
    return it->second;
}

bool SurfaceGraph::has_segment(SegmentId seg_id) const {
    return segment_to_node_.find(seg_id) != segment_to_node_.end();
}

std::vector<EdgeId> SurfaceGraph::edges_for_node(NodeId node_id) const {
    std::vector<EdgeId> result;
    for (const auto& edge : edges_) {
        if (edge.node_a == node_id || edge.node_b == node_id) {
            result.push_back(edge.id);
        }
    }
    return result;
}

float SurfaceGraph::compute_energy() const {
    float energy = 0.0f;

    // Spring potential energy: 0.5 * k * (x - rest_length)^2
    // Parallelize over edges - each edge computes energy independently
    #pragma omp parallel for schedule(static) reduction(+:energy) if(edges_.size() > 50)
    for (size_t i = 0; i < edges_.size(); ++i) {
        const auto& edge = edges_[i];
        const auto& a = nodes_[edge.node_a];
        const auto& b = nodes_[edge.node_b];
        float dist = a.position.distance_to(b.position);
        float delta = dist - edge.rest_length;
        energy += 0.5f * edge.stiffness * delta * delta;
    }

    return energy;
}

void SurfaceGraph::clear_all_forces() {
    for (auto& node : nodes_) {
        node.clear_force();
    }
}

void SurfaceGraph::build_adjacency_index() {
    // Initialize with no neighbors (-1 indicates no neighbor)
    continuity_neighbors_.clear();
    continuity_neighbors_.resize(nodes_.size(), {static_cast<NodeId>(-1), static_cast<NodeId>(-1)});

    // Scan edges once to build index
    for (const auto& edge : edges_) {
        if (edge.type != EdgeType::YarnContinuity) continue;

        // edge.node_a -> edge.node_b means:
        // - node_b.prev = node_a
        // - node_a.next = node_b
        continuity_neighbors_[edge.node_b].first = edge.node_a;
        continuity_neighbors_[edge.node_a].second = edge.node_b;
    }

    adjacency_built_ = true;
}

std::pair<NodeId, NodeId> SurfaceGraph::get_continuity_neighbors(NodeId node) const {
    if (!adjacency_built_) {
        throw std::runtime_error("Adjacency index not built. Call build_adjacency_index() first.");
    }
    if (node >= continuity_neighbors_.size()) {
        throw std::out_of_range("Node ID out of range");
    }
    return continuity_neighbors_[node];
}

void SurfaceGraph::build_passthrough_pairs() {
    passthrough_pairs_.clear();
    for (const auto& edge : edges_) {
        if (edge.type != EdgeType::PassThrough) continue;
        passthrough_pairs_.insert({edge.node_a, edge.node_b});
        passthrough_pairs_.insert({edge.node_b, edge.node_a});
    }
    passthrough_pairs_built_ = true;
}

bool SurfaceGraph::is_cross_row_neighbor(NodeId a, NodeId b) const {
    if (!passthrough_pairs_built_) {
        throw std::runtime_error("Passthrough pairs not built. Call build_passthrough_pairs() first.");
    }
    return passthrough_pairs_.count({a, b}) > 0;
}

}  // namespace yarnpath
