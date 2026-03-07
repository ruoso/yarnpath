#ifndef YARNPATH_SERIALIZATION_SURFACE_GRAPH_JSON_HPP
#define YARNPATH_SERIALIZATION_SURFACE_GRAPH_JSON_HPP

#include <nlohmann/json.hpp>
#include <surface/surface_graph.hpp>
#include <surface/surface_node.hpp>
#include <surface/surface_edge.hpp>
#include <surface/surface_constraint.hpp>
#include <stitch_shape/stitch_shape.hpp>
#include "config_json.hpp"

namespace yarnpath {

// EdgeType enum serialization
NLOHMANN_JSON_SERIALIZE_ENUM(EdgeType, {
    {EdgeType::YarnContinuity, "YarnContinuity"},
    {EdgeType::PassThrough, "PassThrough"},
})

// ConstraintType enum serialization
NLOHMANN_JSON_SERIALIZE_ENUM(ConstraintType, {
    {ConstraintType::MaxStretch, "MaxStretch"},
    {ConstraintType::MinDistance, "MinDistance"},
})

// StitchShapeParams serialization
inline void to_json(nlohmann::json& j, const StitchShapeParams& shape) {
    j["z_bulge"] = shape.z_bulge;
    j["loop_height"] = shape.loop_height;
    j["loop_width"] = shape.loop_width;
    j["z_bulge_factor"] = shape.z_bulge_factor;
    j["apex_lean_x"] = shape.apex_lean_x;
    j["apex_height_factor"] = shape.apex_height_factor;
    j["symmetric_exit"] = shape.symmetric_exit;
    j["entry_tangent_scale"] = shape.entry_tangent_scale;
    j["width_multiplier"] = shape.width_multiplier;
    j["height_multiplier"] = shape.height_multiplier;
}

inline void from_json(const nlohmann::json& j, StitchShapeParams& shape) {
    shape.z_bulge = j.value("z_bulge", 0.0f);
    shape.loop_height = j.value("loop_height", 0.0f);
    shape.loop_width = j.value("loop_width", 0.0f);
    shape.z_bulge_factor = j.value("z_bulge_factor", 1.0f);
    shape.apex_lean_x = j.value("apex_lean_x", 0.0f);
    shape.apex_height_factor = j.value("apex_height_factor", 1.0f);
    shape.symmetric_exit = j.value("symmetric_exit", true);
    shape.entry_tangent_scale = j.value("entry_tangent_scale", 1.0f);
    shape.width_multiplier = j.value("width_multiplier", 1.0f);
    shape.height_multiplier = j.value("height_multiplier", 1.0f);
}

// SurfaceNode serialization
inline void to_json(nlohmann::json& j, const SurfaceNode& node) {
    j["id"] = node.id;
    j["segment_id"] = node.segment_id;
    j["position"] = node.position;
    j["velocity"] = node.velocity;
    j["force"] = node.force;
    j["mass"] = node.mass;
    j["is_pinned"] = node.is_pinned;
    j["forms_loop"] = node.forms_loop;
    j["stitch_axis"] = node.stitch_axis;
    j["shape"] = node.shape;
}

inline void from_json(const nlohmann::json& j, SurfaceNode& node) {
    node.id = j["id"].get<NodeId>();
    node.segment_id = j["segment_id"].get<SegmentId>();
    node.position = j["position"].get<Vec3>();
    node.velocity = j["velocity"].get<Vec3>();
    node.force = j["force"].get<Vec3>();
    node.mass = j["mass"].get<float>();
    node.is_pinned = j["is_pinned"].get<bool>();
    node.forms_loop = j["forms_loop"].get<bool>();
    node.stitch_axis = j.value("stitch_axis", Vec3::unit_x());
    node.shape = j.value("shape", StitchShapeParams{});
}

// SurfaceEdge serialization
inline void to_json(nlohmann::json& j, const SurfaceEdge& edge) {
    j["id"] = edge.id;
    j["node_a"] = edge.node_a;
    j["node_b"] = edge.node_b;
    j["type"] = edge.type;
    j["rest_length"] = edge.rest_length;
    j["stiffness"] = edge.stiffness;
}

inline void from_json(const nlohmann::json& j, SurfaceEdge& edge) {
    edge.id = j["id"].get<EdgeId>();
    edge.node_a = j["node_a"].get<NodeId>();
    edge.node_b = j["node_b"].get<NodeId>();
    edge.type = j["type"].get<EdgeType>();
    edge.rest_length = j["rest_length"].get<float>();
    edge.stiffness = j["stiffness"].get<float>();
}

// SurfaceConstraint serialization
inline void to_json(nlohmann::json& j, const SurfaceConstraint& constraint) {
    j["id"] = constraint.id;
    j["type"] = constraint.type;
    j["node_a"] = constraint.node_a;
    j["node_b"] = constraint.node_b;
    j["limit"] = constraint.limit;
}

inline void from_json(const nlohmann::json& j, SurfaceConstraint& constraint) {
    constraint.id = j["id"].get<ConstraintId>();
    constraint.type = j["type"].get<ConstraintType>();
    constraint.node_a = j["node_a"].get<NodeId>();
    constraint.node_b = j["node_b"].get<NodeId>();
    constraint.limit = j["limit"].get<float>();
}

// SurfaceGraph serialization
inline nlohmann::json surface_graph_to_json(const SurfaceGraph& graph) {
    nlohmann::json j;

    // Serialize core data
    j["nodes"] = graph.nodes();
    j["edges"] = graph.edges();
    j["constraints"] = graph.constraints();

    // Serialize segment_to_node mapping
    nlohmann::json segment_map = nlohmann::json::object();
    for (const auto& node : graph.nodes()) {
        segment_map[std::to_string(node.segment_id)] = node.id;
    }
    j["segment_to_node"] = segment_map;

    return j;
}

// SurfaceGraph deserialization
// Note: Reconstructs the graph from serialized data and rebuilds optimization structures
inline SurfaceGraph surface_graph_from_json(const nlohmann::json& j) {
    SurfaceGraph graph;

    // Deserialize nodes
    for (const auto& node_j : j["nodes"]) {
        graph.add_node(node_j.get<SurfaceNode>());
    }

    // Deserialize edges
    for (const auto& edge_j : j["edges"]) {
        graph.add_edge(edge_j.get<SurfaceEdge>());
    }

    // Deserialize constraints
    for (const auto& constraint_j : j["constraints"]) {
        graph.add_constraint(constraint_j.get<SurfaceConstraint>());
    }

    // Note: segment_to_node_ is rebuilt from nodes automatically via add_node
    // Note: adjacency index and constraint colors are NOT serialized
    // They must be rebuilt by calling:
    //   - graph.build_adjacency_index()
    //   - graph.build_constraint_colors()
    // This is typically done by the solver

    return graph;
}

}  // namespace yarnpath

#endif // YARNPATH_SERIALIZATION_SURFACE_GRAPH_JSON_HPP
