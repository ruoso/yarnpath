#include "surface_forces.hpp"
#include "logging.hpp"
#include <cmath>
#include <limits>
#include <set>
#include <vector>
#ifdef _OPENMP
#include <omp.h>
#endif

namespace yarnpath {

void compute_forces(SurfaceGraph& graph,
                    const YarnProperties& yarn,
                    const ForceConfig& config) {
    // Clear all forces first
    graph.clear_all_forces();

    // Compute spring forces from all edges
    compute_spring_forces(graph);

    // Add passthrough tension based on yarn properties
    compute_passthrough_tension(graph, yarn, config.passthrough_tension_factor);

    // Add loop curvature forces
    compute_loop_curvature_forces(graph, yarn, config.loop_curvature_strength);

    // Add collision repulsion forces
    if (config.enable_collision && config.collision_strength > 0) {
        compute_collision_forces(graph, yarn.min_clearance(), config.collision_strength);
    }

    // Add gravity if enabled
    if (config.enable_gravity && config.gravity_strength > 0) {
        compute_gravity_force(graph, config.gravity_strength, config.gravity_direction);
    }

    // Apply velocity damping
    apply_damping(graph, config.damping);
}

void compute_spring_forces(SurfaceGraph& graph) {
    auto& nodes = graph.nodes();
    const auto& edges = graph.edges();
    const size_t num_nodes = nodes.size();

    // Thread-local force accumulation - eliminates atomic operations
    #pragma omp parallel if(edges.size() > 50)
    {
        // Each thread has its own force buffer
        std::vector<Vec3> thread_forces(num_nodes, Vec3::zero());

        #pragma omp for schedule(static)
        for (size_t i = 0; i < edges.size(); ++i) {
            const auto& edge = edges[i];

            // Read positions (const access, thread-safe)
            const Vec3& pos_a = nodes[edge.node_a].position;
            const Vec3& pos_b = nodes[edge.node_b].position;

            Vec3 delta = pos_b - pos_a;
            float length = delta.length();

            // Avoid division by zero
            if (length < 1e-6f) {
                continue;
            }

            // Spring force: F = -k * (length - rest_length) * direction
            float displacement = length - edge.rest_length;
            Vec3 force = (delta / length) * (edge.stiffness * displacement);

            // Write to thread-local buffer (no atomics needed!)
            thread_forces[edge.node_a] += force;
            thread_forces[edge.node_b] -= force;
        }

        // Merge thread-local forces into global forces (single critical section)
        #pragma omp critical
        {
            for (size_t i = 0; i < num_nodes; ++i) {
                nodes[i].force += thread_forces[i];
            }
        }
    }
}

void compute_passthrough_tension(SurfaceGraph& graph,
                                  const YarnProperties& yarn,
                                  float tension_factor) {
    // Additional tension force on passthrough edges to straighten yarn
    // Higher yarn tension = stronger straightening force

    float tension_strength = yarn.tension * tension_factor;
    auto& nodes = graph.nodes();
    const auto& edges = graph.edges();
    const auto& passthrough_ids = graph.passthrough_edge_ids();
    const size_t num_nodes = nodes.size();

    // Skip if no passthrough edges
    if (passthrough_ids.empty()) {
        return;
    }

    // Thread-local force accumulation - eliminates atomic operations
    #pragma omp parallel if(passthrough_ids.size() > 50)
    {
        // Each thread has its own force buffer
        std::vector<Vec3> thread_forces(num_nodes, Vec3::zero());

        #pragma omp for schedule(static)
        for (size_t i = 0; i < passthrough_ids.size(); ++i) {
            const auto& edge = edges[passthrough_ids[i]];

            // Read positions (const access, thread-safe)
            const Vec3& pos_a = nodes[edge.node_a].position;
            const Vec3& pos_b = nodes[edge.node_b].position;

            Vec3 delta = pos_b - pos_a;
            float length = delta.length();

            if (length < 1e-6f) {
                continue;
            }

            // Extra force pulling nodes together based on tension
            // This simulates the yarn being pulled taut through the loop
            Vec3 force = (delta / length) * tension_strength;

            // Write to thread-local buffer (no atomics needed!)
            thread_forces[edge.node_a] += force;
            thread_forces[edge.node_b] -= force;
        }

        // Merge thread-local forces into global forces (single critical section)
        #pragma omp critical
        {
            for (size_t i = 0; i < num_nodes; ++i) {
                nodes[i].force += thread_forces[i];
            }
        }
    }
}

void compute_loop_curvature_forces(SurfaceGraph& graph,
                                    const YarnProperties& yarn,
                                    float strength) {
    // Loop curvature force encourages loops to maintain natural shape
    // For nodes that form loops, apply a gentle force based on aspect ratio

    if (strength < 1e-6f) {
        return;  // Skip if strength is negligible
    }

    // Build adjacency index once if not already built (thread-safe)
    // This eliminates O(N×E) nested loop by providing O(1) neighbor lookup
    #pragma omp single
    {
        if (!graph.has_adjacency_index()) {
            graph.build_adjacency_index();
        }
    }

    auto& nodes = graph.nodes();

    // NOW PARALLELIZABLE - no nested edge search!
    // Each loop node independently looks up its neighbors in O(1)
    #pragma omp parallel for schedule(static) if(nodes.size() > 50)
    for (size_t i = 0; i < nodes.size(); ++i) {
        auto& node = nodes[i];
        if (!node.forms_loop) {
            continue;
        }

        // Fast O(1) neighbor lookup instead of O(E) edge search
        auto [prev_id, next_id] = graph.get_continuity_neighbors(node.id);

        // If we have both neighbors, apply curvature force
        if (prev_id != static_cast<NodeId>(-1) && next_id != static_cast<NodeId>(-1)) {
            const auto& prev_node = graph.node(prev_id);
            const auto& next_node = graph.node(next_id);

            // Vector from prev through current to next
            Vec3 to_prev = prev_node.position - node.position;
            Vec3 to_next = next_node.position - node.position;

            float len_prev = to_prev.length();
            float len_next = to_next.length();

            if (len_prev < 1e-6f || len_next < 1e-6f) {
                continue;
            }

            // Normalize
            Vec3 dir_prev = to_prev / len_prev;
            Vec3 dir_next = to_next / len_next;

            // The "natural" direction is influenced by loop aspect ratio
            // Higher aspect ratio = more vertical loops
            // We apply a force perpendicular to the chord (prev to next)
            // pushing the loop node "outward" to create the loop shape

            Vec3 chord = next_node.position - prev_node.position;
            float chord_len = chord.length();

            if (chord_len < 1e-6f) {
                continue;
            }

            // Find perpendicular direction (simplified: use Y-up)
            Vec3 chord_dir = chord / chord_len;
            Vec3 up = Vec3::unit_y();

            // Perpendicular to chord in the vertical plane
            Vec3 perp = up - chord_dir * chord_dir.dot(up);
            float perp_len = perp.length();

            if (perp_len < 1e-6f) {
                // Chord is vertical, use X instead
                up = Vec3::unit_x();
                perp = up - chord_dir * chord_dir.dot(up);
                perp_len = perp.length();
            }

            if (perp_len > 1e-6f) {
                perp = perp / perp_len;

                // Force magnitude based on desired loop height
                // aspect_ratio = height / width, where width ~ chord_len
                float desired_height = chord_len * yarn.loop_aspect_ratio * 0.5f;

                // Current height of the loop node above the chord midpoint
                Vec3 chord_mid = (prev_node.position + next_node.position) * 0.5f;
                Vec3 to_mid = node.position - chord_mid;
                float current_height = to_mid.dot(perp);

                // Force to reach desired height
                float height_diff = desired_height - current_height;
                Vec3 curvature_force = perp * (height_diff * strength);

                node.add_force(curvature_force);
            }
        }
    }
}

void apply_damping(SurfaceGraph& graph, float damping) {
    auto& nodes = graph.nodes();

    // Parallelize over nodes - each node is independent
    #pragma omp parallel for schedule(static) if(nodes.size() > 50)
    for (size_t i = 0; i < nodes.size(); ++i) {
        nodes[i].force -= nodes[i].velocity * damping;
    }
}

void compute_bending_forces(SurfaceGraph& graph,
                            float stiffness,
                            float min_angle) {
    // Look at triplets of consecutive nodes along continuity edges
    // and apply forces to prevent sharp bends/folds

    const auto& edges = graph.edges();

    // Find continuity chains: sequences of nodes connected by YarnContinuity edges
    // Since continuity edges connect consecutive segments (i -> i+1),
    // we can iterate through nodes in order

    // For each triplet (prev, curr, next) along continuity path
    for (NodeId curr = 1; curr + 1 < graph.node_count(); ++curr) {
        NodeId prev = curr - 1;
        NodeId next = curr + 1;

        // Verify these are connected by continuity edges
        bool has_prev_edge = false;
        bool has_next_edge = false;
        for (const auto& edge : edges) {
            if (edge.type != EdgeType::YarnContinuity) continue;
            if ((edge.node_a == prev && edge.node_b == curr) ||
                (edge.node_a == curr && edge.node_b == prev)) {
                has_prev_edge = true;
            }
            if ((edge.node_a == curr && edge.node_b == next) ||
                (edge.node_a == next && edge.node_b == curr)) {
                has_next_edge = true;
            }
        }

        if (!has_prev_edge || !has_next_edge) continue;

        // Get positions
        const Vec3& pos_prev = graph.node(prev).position;
        const Vec3& pos_curr = graph.node(curr).position;
        const Vec3& pos_next = graph.node(next).position;

        // Vectors along the chain
        Vec3 v1 = pos_curr - pos_prev;  // prev -> curr
        Vec3 v2 = pos_next - pos_curr;  // curr -> next

        float len1 = v1.length();
        float len2 = v2.length();

        if (len1 < 1e-6f || len2 < 1e-6f) continue;

        Vec3 dir1 = v1 / len1;
        Vec3 dir2 = v2 / len2;

        // Compute the bend angle
        // cos(angle) = dir1 . dir2
        // angle = 0 means straight, angle = PI means folded back
        float cos_angle = dir1.dot(dir2);
        cos_angle = std::max(-1.0f, std::min(1.0f, cos_angle));

        // We want the angle to be close to 0 (straight) or at least > min_angle
        // If cos_angle < cos(PI - min_angle), the bend is too sharp
        float max_cos = std::cos(3.14159f - min_angle);  // cos of max allowed bend

        if (cos_angle < max_cos) {
            // Bend is too sharp - apply straightening force
            float bend_severity = max_cos - cos_angle;
            float force_mag = stiffness * bend_severity;

            // The straightening direction: push next node to align with dir1
            // and push prev node to align with -dir2
            // The ideal position for next would be: pos_curr + dir1 * len2
            Vec3 ideal_next = pos_curr + dir1 * len2;
            Vec3 correction_next = ideal_next - pos_next;

            // Similarly for prev
            Vec3 ideal_prev = pos_curr - dir2 * len1;
            Vec3 correction_prev = ideal_prev - pos_prev;

            // Apply scaled forces
            float corr_next_len = correction_next.length();
            float corr_prev_len = correction_prev.length();

            if (corr_next_len > 1e-6f) {
                Vec3 force_next = correction_next / corr_next_len * force_mag;
                graph.node(next).add_force(force_next);
                graph.node(curr).add_force(-force_next * 0.5f);
            }

            if (corr_prev_len > 1e-6f) {
                Vec3 force_prev = correction_prev / corr_prev_len * force_mag;
                graph.node(prev).add_force(force_prev);
                graph.node(curr).add_force(-force_prev * 0.5f);
            }
        }
    }
}

void compute_planar_forces(SurfaceGraph& graph,
                           float stiffness,
                           float max_deviation) {
    if (graph.node_count() < 3) return;

    // Compute centroid
    Vec3 centroid = Vec3::zero();
    for (const auto& node : graph.nodes()) {
        centroid += node.position;
    }
    centroid = centroid / static_cast<float>(graph.node_count());

    // Compute dominant plane normal using the helper function
    Vec3 plane_normal = compute_dominant_plane_normal(graph);

    // For each node, compute signed distance to plane and apply force if needed
    for (auto& node : graph.nodes()) {
        if (node.is_pinned) continue;

        // Signed distance from node to plane
        Vec3 to_node = node.position - centroid;
        float signed_dist = to_node.dot(plane_normal);

        // If deviation exceeds max, apply restoring force
        if (std::abs(signed_dist) > max_deviation) {
            float excess = std::abs(signed_dist) - max_deviation;
            float force_mag = stiffness * excess;

            // Force direction: push back toward plane
            Vec3 force = -plane_normal * (signed_dist > 0 ? force_mag : -force_mag);
            node.add_force(force);
        }
    }
}

void compute_gravity_force(SurfaceGraph& graph,
                           float strength,
                           const Vec3& direction) {
    // F = m * g * direction
    Vec3 gravity_dir = direction.normalized();
    auto& nodes = graph.nodes();

    // Parallelize over nodes - each node is independent
    // Removed reduction overhead for total_force calculation
    #pragma omp parallel for schedule(static) if(nodes.size() > 50)
    for (size_t i = 0; i < nodes.size(); ++i) {
        auto& node = nodes[i];
        if (node.is_pinned) {
            continue;
        }
        // Convert gravity from m/s² to mm/s² since positions are in mm
        // strength is in m/s² (e.g., 9.8), mass is in grams
        // Force units should be compatible with spring forces (which use mm distances)
        // Numerically treat the value as mm/s² instead of m/s² for unit consistency
        float gravity_in_mm = strength;
        Vec3 gravity_force = gravity_dir * (node.mass * gravity_in_mm);
        node.add_force(gravity_force);
    }
}

void apply_floor_constraint(SurfaceGraph& graph, float floor_dist, const Vec3& direction) {
    auto log = yarnpath::logging::get_logger();

    static int log_counter = 0;
    bool should_log = (log_counter++ % 1000 == 0);

    Vec3 floor_dir = direction.normalized();
    int nodes_on_floor = 0;
    float min_dist = std::numeric_limits<float>::max();
    float max_dist = std::numeric_limits<float>::lowest();

    auto& nodes = graph.nodes();

    // Parallelize over nodes - each node checked independently
    // Use reduction for statistics
    #pragma omp parallel for schedule(static) reduction(+:nodes_on_floor) \
            reduction(min:min_dist) reduction(max:max_dist) if(nodes.size() > 50)
    for (size_t i = 0; i < nodes.size(); ++i) {
        auto& node = nodes[i];
        // Compute signed distance along floor direction
        float dist = node.position.dot(floor_dir);

        min_dist = std::min(min_dist, dist);
        max_dist = std::max(max_dist, dist);

        if (dist > floor_dist) {
            // Project node back to floor plane
            float excess = dist - floor_dist;
            node.position -= floor_dir * excess;

            // Zero out velocity component along floor direction
            float vel_along = node.velocity.dot(floor_dir);
            if (vel_along > 0) {
                node.velocity -= floor_dir * vel_along;
            }
            nodes_on_floor++;
        }
    }

    if (should_log) {
        log->debug("Floor: dist={}, nodes_on_floor={}/{}, dist_range=[{}, {}]",
                   floor_dist, nodes_on_floor, graph.node_count(), min_dist, max_dist);
    }
}

void compute_collision_forces(SurfaceGraph& graph, float min_distance, float strength) {
    // Build a set of connected pairs (edges) to skip during collision check
    std::set<std::pair<NodeId, NodeId>> connected;
    for (const auto& edge : graph.edges()) {
        NodeId a = std::min(edge.node_a, edge.node_b);
        NodeId b = std::max(edge.node_a, edge.node_b);
        connected.insert({a, b});
    }

    // Also skip adjacent nodes along the yarn path (i, i+1)
    for (NodeId i = 0; i + 1 < graph.node_count(); ++i) {
        connected.insert({i, i + 1});
    }

    auto& nodes = graph.nodes();

    // Check all pairs of non-connected nodes for potential collisions
    for (size_t i = 0; i < nodes.size(); ++i) {
        for (size_t j = i + 1; j < nodes.size(); ++j) {
            // Skip if connected
            NodeId a = static_cast<NodeId>(i);
            NodeId b = static_cast<NodeId>(j);
            if (connected.count({a, b})) continue;

            // Check distance using squared distance first (avoids sqrt)
            Vec3 delta = nodes[j].position - nodes[i].position;
            float dist_sq = delta.length_squared();
            float min_dist_sq = min_distance * min_distance;

            // Apply repulsion force when closer than min_distance
            // Force falls off with distance: F = strength * (1 - dist/min_distance)^2
            if (dist_sq < min_dist_sq && dist_sq > 1e-12f) {
                // Only compute sqrt when we know we need it
                float dist = std::sqrt(dist_sq);
                float overlap_ratio = 1.0f - dist / min_distance;
                float force_magnitude = strength * overlap_ratio * overlap_ratio;

                Vec3 direction = delta / dist;  // Points from i to j
                Vec3 force = direction * force_magnitude;

                // Push nodes apart
                nodes[i].add_force(-force);
                nodes[j].add_force(force);
            }
        }
    }
}

Vec3 compute_dominant_plane_normal(const SurfaceGraph& graph) {
    // Compute the dominant plane normal using covariance matrix analysis
    // The normal is the eigenvector corresponding to the smallest eigenvalue
    // (direction of minimum variance = perpendicular to the plane)

    if (graph.node_count() < 3) {
        return Vec3::unit_z();  // Default to Z-up
    }

    // Compute centroid
    Vec3 centroid = Vec3::zero();
    for (const auto& node : graph.nodes()) {
        centroid += node.position;
    }
    centroid = centroid / static_cast<float>(graph.node_count());

    // Compute 3x3 covariance matrix
    // cov[i][j] = sum((pos[i] - centroid[i]) * (pos[j] - centroid[j])) / n
    float cov[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

    for (const auto& node : graph.nodes()) {
        Vec3 d = node.position - centroid;
        float coords[3] = {d.x, d.y, d.z};
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                cov[i][j] += coords[i] * coords[j];
            }
        }
    }

    float n = static_cast<float>(graph.node_count());
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            cov[i][j] /= n;
        }
    }

    // Use power iteration to find the eigenvector with the largest eigenvalue
    // Then find the one with the smallest by trying all three axis-aligned starts
    // and keeping the one that converges to the smallest eigenvalue

    auto power_iteration = [&](Vec3 start, int iterations) -> std::pair<Vec3, float> {
        Vec3 v = start.normalized();
        float eigenvalue = 0;
        for (int iter = 0; iter < iterations; ++iter) {
            // Multiply by covariance matrix
            Vec3 Av;
            Av.x = cov[0][0] * v.x + cov[0][1] * v.y + cov[0][2] * v.z;
            Av.y = cov[1][0] * v.x + cov[1][1] * v.y + cov[1][2] * v.z;
            Av.z = cov[2][0] * v.x + cov[2][1] * v.y + cov[2][2] * v.z;

            float len = Av.length();
            if (len < 1e-10f) {
                // This direction has near-zero variance - it's the normal!
                return {v, 0.0f};
            }
            eigenvalue = len;
            v = Av / len;
        }
        return {v, eigenvalue};
    };

    // Find the largest eigenvector first (principal component)
    auto [v1, e1] = power_iteration(Vec3(1, 0, 0), 50);

    // Deflate the matrix to find the second largest
    // cov' = cov - e1 * v1 * v1^T
    float cov2[3][3];
    float v1_coords[3] = {v1.x, v1.y, v1.z};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            cov2[i][j] = cov[i][j] - e1 * v1_coords[i] * v1_coords[j];
        }
    }

    // Find second eigenvector (start perpendicular to v1)
    Vec3 start2 = (std::abs(v1.x) < 0.9f) ? Vec3(1, 0, 0) : Vec3(0, 1, 0);
    start2 = start2 - v1 * start2.dot(v1);
    if (start2.length() > 1e-6f) {
        start2 = start2.normalized();
    }

    auto power_iteration2 = [&](Vec3 start, int iterations) -> std::pair<Vec3, float> {
        Vec3 v = start.normalized();
        float eigenvalue = 0;
        for (int iter = 0; iter < iterations; ++iter) {
            Vec3 Av;
            Av.x = cov2[0][0] * v.x + cov2[0][1] * v.y + cov2[0][2] * v.z;
            Av.y = cov2[1][0] * v.x + cov2[1][1] * v.y + cov2[1][2] * v.z;
            Av.z = cov2[2][0] * v.x + cov2[2][1] * v.y + cov2[2][2] * v.z;

            float len = Av.length();
            if (len < 1e-10f) {
                return {v, 0.0f};
            }
            eigenvalue = len;
            v = Av / len;
        }
        return {v, eigenvalue};
    };

    auto [v2, e2] = power_iteration2(start2, 50);

    // The normal is perpendicular to both v1 and v2
    Vec3 normal = v1.cross(v2);
    if (normal.length() < 1e-6f) {
        // Degenerate case - use default
        return Vec3::unit_z();
    }
    normal = normal.normalized();

    // Ensure consistent orientation (prefer positive Z component)
    if (normal.z < 0) {
        normal = -normal;
    }

    auto log = yarnpath::logging::get_logger();
    log->debug("Dominant plane: v1=({:.3f},{:.3f},{:.3f}) e1={:.3f}, v2=({:.3f},{:.3f},{:.3f}) e2={:.3f}, normal=({:.3f},{:.3f},{:.3f})",
               v1.x, v1.y, v1.z, e1, v2.x, v2.y, v2.z, e2, normal.x, normal.y, normal.z);

    return normal;
}

}  // namespace yarnpath
