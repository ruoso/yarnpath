#include "surface_graph.hpp"
#include "logging.hpp"
#include <stdexcept>

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

ConstraintId SurfaceGraph::add_constraint(const SurfaceConstraint& constraint) {
    ConstraintId id = static_cast<ConstraintId>(constraints_.size());
    SurfaceConstraint new_constraint = constraint;
    new_constraint.id = id;
    constraints_.push_back(new_constraint);
    return id;
}

SurfaceConstraint& SurfaceGraph::constraint(ConstraintId id) {
    if (id >= constraints_.size()) {
        throw std::out_of_range("SurfaceGraph::constraint: invalid constraint id");
    }
    return constraints_[id];
}

const SurfaceConstraint& SurfaceGraph::constraint(ConstraintId id) const {
    if (id >= constraints_.size()) {
        throw std::out_of_range("SurfaceGraph::constraint: invalid constraint id");
    }
    return constraints_[id];
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
    for (const auto& edge : edges_) {
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

}  // namespace yarnpath
