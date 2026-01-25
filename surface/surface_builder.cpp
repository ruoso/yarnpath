#include "surface_builder.hpp"
#include "logging.hpp"
#include <random>
#include <cmath>
#include <map>
#include <set>

namespace yarnpath {

SurfaceGraph SurfaceBuilder::from_yarn_path(
    const YarnPath& path,
    const YarnProperties& yarn,
    const Gauge& gauge,
    const SurfaceBuildConfig& config) {

    SurfaceBuilder builder(path, yarn, gauge, config);
    builder.create_nodes();
    builder.create_continuity_edges();
    builder.create_passthrough_edges();
    builder.create_constraints();
    builder.initialize_positions();

    auto log = yarnpath::logging::get_logger();
    log->info("SurfaceBuilder: created graph with {} nodes, {} edges, {} constraints",
              builder.graph_.node_count(),
              builder.graph_.edge_count(),
              builder.graph_.constraint_count());

    return std::move(builder.graph_);
}

SurfaceBuilder::SurfaceBuilder(const YarnPath& path, const YarnProperties& yarn,
                               const Gauge& gauge, const SurfaceBuildConfig& config)
    : path_(path), yarn_(yarn), gauge_(gauge), config_(config) {
}

void SurfaceBuilder::create_nodes() {
    auto log = yarnpath::logging::get_logger();
    const auto& segments = path_.segments();

    // Estimate loop width from gauge (stitch width gives approximate loop size)
    float loop_width = gauge_.needle_diameter;  // Loop wraps around needle

    // Connector length (yarn between adjacent loops)
    float connector_length = yarn_.radius * 3.0f;  // Same as continuity rest length base

    for (size_t i = 0; i < segments.size(); ++i) {
        SurfaceNode node;
        node.segment_id = static_cast<SegmentId>(i);
        node.forms_loop = segments[i].forms_loop;
        node.is_pinned = false;
        node.position = vec3::zero();  // Will be set in initialize_positions
        node.velocity = vec3::zero();
        node.force = vec3::zero();

        // Compute mass based on yarn length in this segment
        if (node.forms_loop) {
            node.mass = yarn_.loop_mass(loop_width);
        } else {
            node.mass = yarn_.connector_mass(connector_length);
        }

        graph_.add_node(node);
    }

    log->debug("SurfaceBuilder: created {} nodes (loop_mass={:.4f}g, connector_mass={:.4f}g)",
               graph_.node_count(),
               yarn_.loop_mass(loop_width),
               yarn_.connector_mass(connector_length));
}

void SurfaceBuilder::create_continuity_edges() {
    auto log = yarnpath::logging::get_logger();

    // Rest length for continuity edges based on yarn properties
    // This is the yarn length between loop centers - typically 2-4x the yarn radius
    // For tight knitting (high tension), use smaller value; for loose, use larger
    float base_rest_length = yarn_.radius * 3.0f * (1.0f + (1.0f - yarn_.tension) * 0.5f);
    // For worsted (radius=1.0, tension=0.5): rest_length ≈ 3.0 * 1.25 = 3.75mm

    // Stiffness derived from yarn properties
    // Base stiffness scales with yarn stiffness (0=flexible, 1=stiff)
    // Using 100.0 as a base constant that works well for the physics simulation
    float base_stiffness = 100.0f * (0.5f + yarn_.stiffness * 0.5f);  // Range: 50-100
    float stiffness = base_stiffness * config_.continuity_stiffness_factor;

    // Connect consecutive segments
    for (size_t i = 0; i + 1 < path_.segment_count(); ++i) {
        SurfaceEdge edge;
        edge.node_a = static_cast<NodeId>(i);
        edge.node_b = static_cast<NodeId>(i + 1);
        edge.type = EdgeType::YarnContinuity;
        edge.rest_length = base_rest_length * config_.continuity_rest_length_factor;
        edge.stiffness = stiffness;

        graph_.add_edge(edge);
    }

    log->debug("SurfaceBuilder: created {} continuity edges with rest_length={}, stiffness={}",
               graph_.edge_count(), base_rest_length, stiffness);
}

void SurfaceBuilder::create_passthrough_edges() {
    auto log = yarnpath::logging::get_logger();
    const auto& segments = path_.segments();
    size_t passthrough_count = 0;

    // Passthrough rest length: distance from child loop center to parent loop center
    // This is based on the loop size (determined by needle diameter and yarn)
    // The child loop passes through the parent and hangs below it
    // Distance is approximately: parent_loop_half_height + child_loop_half_height + clearance
    float loop_height = gauge_.loop_height(yarn_.radius, yarn_.loop_aspect_ratio);
    float base_rest_length = loop_height + yarn_.min_clearance();

    // Stiffness for passthrough edges - derived from yarn stiffness
    // Passthroughs are less stiff than continuity (yarn can slide through loops)
    float base_stiffness = 100.0f * (0.5f + yarn_.stiffness * 0.5f);
    float stiffness = base_stiffness * config_.passthrough_stiffness_factor;

    for (size_t i = 0; i < segments.size(); ++i) {
        const auto& segment = segments[i];

        // Connect this segment to each loop it passes through
        for (SegmentId parent_seg : segment.through) {
            // Make sure the parent segment exists
            if (parent_seg >= segments.size()) {
                log->warn("SurfaceBuilder: segment {} references invalid parent {}",
                          i, parent_seg);
                continue;
            }

            SurfaceEdge edge;
            edge.node_a = static_cast<NodeId>(i);
            edge.node_b = static_cast<NodeId>(parent_seg);
            edge.type = EdgeType::PassThrough;
            edge.rest_length = base_rest_length * config_.passthrough_rest_length_factor;
            edge.stiffness = stiffness;

            graph_.add_edge(edge);
            ++passthrough_count;
        }
    }

    log->debug("SurfaceBuilder: created {} passthrough edges with rest_length={}, stiffness={}",
               passthrough_count, base_rest_length, stiffness);
}

void SurfaceBuilder::create_constraints() {
    auto log = yarnpath::logging::get_logger();

    // MaxStretch constraints on continuity edges
    // Yarn can stretch by elasticity factor
    float max_stretch = 1.0f + yarn_.elasticity;

    for (const auto& edge : graph_.edges()) {
        if (edge.type == EdgeType::YarnContinuity) {
            SurfaceConstraint constraint;
            constraint.type = ConstraintType::MaxStretch;
            constraint.node_a = edge.node_a;
            constraint.node_b = edge.node_b;
            constraint.limit = edge.rest_length * max_stretch;

            graph_.add_constraint(constraint);
        }
    }

    // MinDistance constraints for passthrough edges
    // The child loop can't be closer than the yarn clearance from parent
    float min_clearance = yarn_.min_clearance();

    for (const auto& edge : graph_.edges()) {
        if (edge.type == EdgeType::PassThrough) {
            SurfaceConstraint constraint;
            constraint.type = ConstraintType::MinDistance;
            constraint.node_a = edge.node_a;
            constraint.node_b = edge.node_b;
            constraint.limit = min_clearance;

            graph_.add_constraint(constraint);
        }
    }

    log->debug("SurfaceBuilder: created {} constraints", graph_.constraint_count());
}

void SurfaceBuilder::initialize_positions() {
    auto log = yarnpath::logging::get_logger();

    // Initialize positions incrementally:
    // 1. Add nodes one at a time
    // 2. Position each based on edge constraints
    // 3. If constraints can't be satisfied, run local relaxation

    std::mt19937 rng(config_.random_seed);
    float noise_amplitude = gauge_.stitch_width() * config_.position_noise;
    std::uniform_real_distribution<float> noise_dist(-noise_amplitude, noise_amplitude);

    const auto& segments = path_.segments();
    const auto& edges = graph_.edges();

    // Build lookup for continuity edge rest lengths
    std::map<std::pair<NodeId, NodeId>, float> continuity_lengths;
    for (const auto& edge : edges) {
        if (edge.type == EdgeType::YarnContinuity) {
            continuity_lengths[{edge.node_a, edge.node_b}] = edge.rest_length;
            continuity_lengths[{edge.node_b, edge.node_a}] = edge.rest_length;
        }
    }

    // Build lookup for passthrough edges: child -> [(parent, rest_length), ...]
    std::map<NodeId, std::vector<std::pair<NodeId, float>>> passthrough_info;
    for (const auto& edge : edges) {
        if (edge.type == EdgeType::PassThrough) {
            passthrough_info[edge.node_a].push_back({edge.node_b, edge.rest_length});
        }
    }

    // Position node 0 at origin
    if (graph_.node_count() > 0) {
        graph_.node(0).position = Vec3(
            noise_dist(rng),
            noise_dist(rng),
            noise_dist(rng) * 0.5f
        );
    }

    // Track current Y position (always increasing - going "up" in fabric terms means down/+Y)
    float current_row_y = 0.0f;
    // Track X position within row (always go right/positive X)
    float current_x = 0.0f;

    // Position each subsequent node incrementally
    for (size_t i = 1; i < segments.size(); ++i) {
        NodeId node_id = static_cast<NodeId>(i);
        NodeId prev_id = static_cast<NodeId>(i - 1);
        auto& node = graph_.node(node_id);
        const auto& prev_node = graph_.node(prev_id);

        // Check if this segment has passthrough edges to parents
        auto pt_it = passthrough_info.find(node_id);
        bool has_passthrough = (pt_it != passthrough_info.end() && !pt_it->second.empty());

        if (has_passthrough) {
            // This node passes through parent loop(s)
            // Use first parent for initial positioning
            NodeId parent_id = pt_it->second[0].first;
            float passthrough_length = pt_it->second[0].second;
            const auto& parent_node = graph_.node(parent_id);

            // Position below the parent at passthrough_length distance (positive Y = down)
            node.position = Vec3(
                parent_node.position.x + noise_dist(rng),
                parent_node.position.y + passthrough_length + noise_dist(rng),
                parent_node.position.z + noise_dist(rng) * 0.5f
            );

            // Update row tracking - new row starts at this Y
            current_row_y = node.position.y;
            current_x = node.position.x;
        } else {
            // Use continuity edge to position relative to previous node
            auto cont_it = continuity_lengths.find({prev_id, node_id});
            float continuity_length = (cont_it != continuity_lengths.end())
                ? cont_it->second
                : gauge_.stitch_width();

            // Always move right (positive X direction) within a row
            current_x += continuity_length;
            node.position = Vec3(
                current_x + noise_dist(rng),
                current_row_y + noise_dist(rng),
                prev_node.position.z + noise_dist(rng) * 0.5f
            );
        }

        // Check for collisions with non-connected existing nodes
        // and push away if too close
        resolve_collisions(node_id, passthrough_info);

        // Check if all constraints involving this node and earlier nodes are satisfied
        bool constraints_ok = check_node_constraints(node_id, continuity_lengths, passthrough_info);

        if (!constraints_ok) {
            // Run local relaxation with nodes 0..i to satisfy constraints
            relax_partial(node_id, continuity_lengths, passthrough_info);
        }
    }

    log->debug("SurfaceBuilder: initialized {} node positions incrementally",
               graph_.node_count());
}

bool SurfaceBuilder::check_node_constraints(
    NodeId node_id,
    const std::map<std::pair<NodeId, NodeId>, float>& continuity_lengths,
    const std::map<NodeId, std::vector<std::pair<NodeId, float>>>& passthrough_info) {

    const float tolerance = 0.01f;
    const auto& node = graph_.node(node_id);

    // Check continuity constraint to previous node
    if (node_id > 0) {
        NodeId prev_id = node_id - 1;
        auto cont_it = continuity_lengths.find({prev_id, node_id});
        if (cont_it != continuity_lengths.end()) {
            const auto& prev_node = graph_.node(prev_id);
            float dist = (node.position - prev_node.position).length();
            float target = cont_it->second;
            // Allow some stretch (MaxStretch constraint limit)
            float max_allowed = target * (1.0f + yarn_.elasticity);
            if (dist > max_allowed + tolerance) {
                return false;
            }
        }
    }

    // Check passthrough constraints
    auto pt_it = passthrough_info.find(node_id);
    if (pt_it != passthrough_info.end()) {
        for (const auto& [parent_id, rest_length] : pt_it->second) {
            if (parent_id < node_id) {  // Only check already-placed parents
                const auto& parent_node = graph_.node(parent_id);
                float dist = (node.position - parent_node.position).length();
                float min_allowed = yarn_.min_clearance();
                if (dist < min_allowed - tolerance) {
                    return false;
                }
            }
        }
    }

    return true;
}

void SurfaceBuilder::relax_partial(
    NodeId up_to_node,
    const std::map<std::pair<NodeId, NodeId>, float>& continuity_lengths,
    const std::map<NodeId, std::vector<std::pair<NodeId, float>>>& passthrough_info) {

    auto log = yarnpath::logging::get_logger();
    log->debug("SurfaceBuilder: running partial relaxation for nodes 0-{}", up_to_node);

    // Simple iterative constraint projection for nodes 0..up_to_node
    const int max_iters = 100;
    const float tolerance = 0.001f;

    for (int iter = 0; iter < max_iters; ++iter) {
        bool any_correction = false;

        // Project continuity constraints (MaxStretch)
        for (NodeId i = 1; i <= up_to_node; ++i) {
            NodeId prev_id = i - 1;
            auto cont_it = continuity_lengths.find({prev_id, i});
            if (cont_it == continuity_lengths.end()) continue;

            auto& node_a = graph_.node(prev_id);
            auto& node_b = graph_.node(i);
            Vec3 delta = node_b.position - node_a.position;
            float dist = delta.length();
            if (dist < 1e-6f) continue;

            float target = cont_it->second;
            float max_allowed = target * (1.0f + yarn_.elasticity);

            if (dist > max_allowed) {
                // Pull together
                float correction = (dist - max_allowed) / dist * 0.5f;
                Vec3 corr_vec = delta * correction;
                node_a.position += corr_vec;
                node_b.position -= corr_vec;
                any_correction = true;
            }
        }

        // Project passthrough constraints (MinDistance)
        for (NodeId i = 0; i <= up_to_node; ++i) {
            auto pt_it = passthrough_info.find(i);
            if (pt_it == passthrough_info.end()) continue;

            for (const auto& [parent_id, rest_length] : pt_it->second) {
                if (parent_id > up_to_node) continue;  // Parent not yet placed

                auto& child = graph_.node(i);
                auto& parent = graph_.node(parent_id);
                Vec3 delta = child.position - parent.position;
                float dist = delta.length();
                if (dist < 1e-6f) continue;

                float min_allowed = yarn_.min_clearance();
                if (dist < min_allowed) {
                    // Push apart
                    float correction = (min_allowed - dist) / dist * 0.5f;
                    Vec3 corr_vec = delta * correction;
                    parent.position -= corr_vec;
                    child.position += corr_vec;
                    any_correction = true;
                }
            }
        }

        if (!any_correction) {
            break;  // Converged
        }
    }
}

void SurfaceBuilder::resolve_collisions(
    NodeId node_id,
    const std::map<NodeId, std::vector<std::pair<NodeId, float>>>& passthrough_info) {

    auto log = yarnpath::logging::get_logger();
    auto& node = graph_.node(node_id);

    // Check if this node and its connections form triangles with consistent orientation
    // In knitting, adjacent stitches passing through adjacent parent loops form quads
    // These quads have implicit triangles that should have consistent normals

    // For this node, check if it forms a quad with the previous node
    // A quad exists when:
    // - node_id has a passthrough to parent_id
    // - node_id - 1 has a passthrough to parent_id - 1 (or adjacent parent)
    // The quad is: [node_id, node_id-1, parent_of_(node_id-1), parent_id]

    auto pt_it = passthrough_info.find(node_id);
    if (pt_it == passthrough_info.end() || pt_it->second.empty() || node_id == 0) {
        return;  // No passthrough or first node - no quad to check
    }

    NodeId parent_id = pt_it->second[0].first;
    NodeId prev_id = node_id - 1;

    // Check if previous node also has a passthrough
    auto prev_pt_it = passthrough_info.find(prev_id);
    if (prev_pt_it == passthrough_info.end() || prev_pt_it->second.empty()) {
        return;  // No quad formed
    }

    NodeId prev_parent_id = prev_pt_it->second[0].first;

    // Now we have a quad: [node_id, prev_id, prev_parent_id, parent_id]
    // Check that the triangles have consistent winding by computing normals
    const Vec3& p0 = node.position;
    const Vec3& p1 = graph_.node(prev_id).position;
    const Vec3& p2 = graph_.node(prev_parent_id).position;
    const Vec3& p3 = graph_.node(parent_id).position;

    // Triangle 1: p0, p1, p2
    Vec3 edge1_t1 = p1 - p0;
    Vec3 edge2_t1 = p2 - p0;
    Vec3 normal1 = edge1_t1.cross(edge2_t1);

    // Triangle 2: p0, p2, p3
    Vec3 edge1_t2 = p2 - p0;
    Vec3 edge2_t2 = p3 - p0;
    Vec3 normal2 = edge1_t2.cross(edge2_t2);

    // Both normals should point in roughly the same direction (positive Z for front face)
    // If they don't, we need to adjust the node position

    // Check if normal1 and normal2 are consistent
    float dot_normals = normal1.dot(normal2);

    if (dot_normals < 0) {
        // Normals are opposite - the quad is twisted
        // Push the node in Z to fix orientation
        log->debug("SurfaceBuilder: fixing twisted quad at node {}", node_id);

        // Average the other three points' Z and ensure we're on the correct side
        float avg_z = (p1.z + p2.z + p3.z) / 3.0f;

        // The normal should point in positive Z (front-facing)
        // Adjust our Z to be consistent
        if (normal1.z < 0) {
            // Normal is pointing backward, we need to flip the quad
            // Move node to the other side of the average Z
            float delta_z = avg_z - node.position.z;
            node.position.z = avg_z + delta_z + yarn_.min_clearance();
        }
    }

    // Also check that the quad doesn't fold back on itself
    // The node should be "below" its parent (larger Y in our coordinate system)
    if (node.position.y < graph_.node(parent_id).position.y) {
        // Node is above its parent - this is wrong for knitting
        // Swap Y positions
        log->debug("SurfaceBuilder: fixing inverted Y at node {}", node_id);
        node.position.y = graph_.node(parent_id).position.y + yarn_.min_clearance();
    }
}

}  // namespace yarnpath
