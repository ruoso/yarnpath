#include "surface_forces.hpp"
#include "logging.hpp"
#include <cmath>
#include <limits>
#include <set>
#include <vector>
#include <unordered_map>
#include <algorithm>
#include <chrono>
#ifdef _OPENMP
#include <omp.h>
#endif

namespace yarnpath {

void compute_forces(SurfaceGraph& graph,
                    const YarnProperties& yarn,
                    const Gauge& gauge,
                    const ForceConfig& config,
                    const std::vector<std::vector<NodeId>>& collision_skip_list) {
    using Clock = std::chrono::steady_clock;
    static int force_call_count = 0;
    bool should_time = (force_call_count < 5 || force_call_count % 100 == 0);
    auto log = yarnpath::logging::get_logger();

    // Clear all forces first
    graph.clear_all_forces();

    auto t0 = Clock::now();
    // Compute spring forces from all edges
    compute_spring_forces(graph);
    auto t1 = Clock::now();

    // Add passthrough tension based on yarn properties
    compute_passthrough_tension(graph, yarn, gauge, config.passthrough_tension_factor);
    auto t2 = Clock::now();

    // Add loop curvature forces
    compute_loop_curvature_forces(graph, yarn, gauge, config.loop_curvature_strength);
    auto t3 = Clock::now();

    // Add sigmoid barrier forces on edges (max stretch + min distance)
    if (config.barrier_strength > 0) {
        compute_barrier_forces(graph, yarn, config.barrier_strength, config.barrier_ramp);
    }
    auto t3b = Clock::now();

    // Add collision repulsion forces
    if (config.enable_collision && config.barrier_strength > 0) {
        compute_collision_forces(graph, yarn.min_clearance(), config.barrier_strength,
                                 config.barrier_ramp, collision_skip_list);
    }
    auto t4 = Clock::now();

    // Add bending resistance to prevent sharp folds
    if (config.enable_bending_resistance && config.bending_stiffness > 0) {
        compute_bending_forces(graph, config.bending_stiffness, config.min_bend_angle);
    }
    auto t5 = Clock::now();

    // Add gravity if enabled
    if (config.enable_gravity && config.gravity_strength > 0) {
        compute_gravity_force(graph, config.gravity_strength, config.gravity_direction);
    }

    // Damping is not applied: with gradient descent (no velocity accumulation),
    // velocity is zero at the start of each step, so damping would be a no-op.
    auto t6 = Clock::now();

    if (should_time) {
        float spring_ms = std::chrono::duration<float, std::milli>(t1 - t0).count();
        float passthrough_ms = std::chrono::duration<float, std::milli>(t2 - t1).count();
        float curvature_ms = std::chrono::duration<float, std::milli>(t3 - t2).count();
        float barrier_ms = std::chrono::duration<float, std::milli>(t3b - t3).count();
        float collision_ms = std::chrono::duration<float, std::milli>(t4 - t3b).count();
        float bending_ms = std::chrono::duration<float, std::milli>(t5 - t4).count();
        float rest_ms = std::chrono::duration<float, std::milli>(t6 - t5).count();
        log->debug("    forces: spring={:.1f}ms passthrough={:.1f}ms curvature={:.1f}ms "
                   "barrier={:.1f}ms collision={:.1f}ms bending={:.1f}ms rest={:.1f}ms",
                   spring_ms, passthrough_ms, curvature_ms, barrier_ms,
                   collision_ms, bending_ms, rest_ms);
    }
    ++force_call_count;
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
                                  const Gauge&, // gauge not used here
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
                                    const Gauge& gauge,
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
                float desired_height = gauge.loop_height(yarn.compressed_radius);

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

    if (stiffness < 1e-6f) {
        return;  // Skip if stiffness is negligible
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

    // For each node with both neighbors, check bend angle
    #pragma omp parallel for schedule(static) if(nodes.size() > 50)
    for (size_t i = 0; i < nodes.size(); ++i) {
        NodeId curr = static_cast<NodeId>(i);

        // Fast O(1) neighbor lookup instead of O(E) edge search
        auto [prev, next] = graph.get_continuity_neighbors(curr);

        // Skip if we don't have both neighbors
        if (prev == static_cast<NodeId>(-1) || next == static_cast<NodeId>(-1)) {
            continue;
        }

        // Get positions (read-only access, thread-safe)
        const Vec3& pos_prev = graph.node(prev).position;
        const Vec3& pos_curr = nodes[i].position;
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

void compute_barrier_forces(SurfaceGraph& graph, const YarnProperties& yarn,
                            float strength, float ramp_fraction) {
    auto& nodes = graph.nodes();
    const auto& edges = graph.edges();
    const size_t num_nodes = nodes.size();

    float max_stretch_factor = 1.0f + yarn.elasticity;
    float min_dist = yarn.min_clearance();

    // Thread-local force accumulation
    #pragma omp parallel if(edges.size() > 50)
    {
        std::vector<Vec3> thread_forces(num_nodes, Vec3::zero());

        #pragma omp for schedule(static)
        for (size_t i = 0; i < edges.size(); ++i) {
            const auto& edge = edges[i];
            const Vec3& pos_a = nodes[edge.node_a].position;
            const Vec3& pos_b = nodes[edge.node_b].position;

            Vec3 delta = pos_b - pos_a;
            float dist = delta.length();
            if (dist < 1e-6f) continue;
            Vec3 dir = delta / dist;

            // Max distance barrier: pull together when approaching max
            float max_dist = edge.rest_length * max_stretch_factor;
            float max_margin = max_dist * ramp_fraction;
            float dist_past_ramp_start = dist - (max_dist - max_margin);

            if (dist_past_ramp_start > 0) {
                float force_mag;
                if (dist_past_ramp_start >= max_margin) {
                    force_mag = -strength;  // at or past limit: full force
                } else {
                    float progress = dist_past_ramp_start / max_margin;
                    float t = progress * 4.0f;
                    force_mag = -strength * t / std::sqrt(1.0f + t * t);
                }
                Vec3 force = dir * force_mag;
                thread_forces[edge.node_a] -= force;  // pull A toward B
                thread_forces[edge.node_b] += force;  // pull B toward A
            }

            // Min distance barrier: push apart when approaching min
            float min_margin = min_dist * ramp_fraction;
            float dist_into_ramp = (min_dist + min_margin) - dist;

            if (dist_into_ramp > 0) {
                float force_mag;
                if (dist_into_ramp >= min_margin) {
                    force_mag = strength;  // at or past limit: full force
                } else {
                    float progress = dist_into_ramp / min_margin;
                    float t = progress * 4.0f;
                    force_mag = strength * t / std::sqrt(1.0f + t * t);
                }
                Vec3 force = dir * force_mag;
                thread_forces[edge.node_a] -= force;  // push A away from B
                thread_forces[edge.node_b] += force;  // push B away from A
            }
        }

        // Merge thread-local forces
        #pragma omp critical
        {
            for (size_t i = 0; i < num_nodes; ++i) {
                nodes[i].force += thread_forces[i];
            }
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

void compute_collision_forces(SurfaceGraph& graph, float min_distance,
                              float strength, float ramp_fraction,
                              const std::vector<std::vector<NodeId>>& skip_list) {
    auto& nodes = graph.nodes();
    const size_t num_nodes = nodes.size();

    if (num_nodes < 2) return;

    // --- Lazy local frame update ---
    // Recompute stitch_axis for nodes that have moved significantly
    const float frame_update_threshold_sq = min_distance * min_distance * 0.25f;  // (min_clearance * 0.5)^2

    // Build adjacency index if needed (for frame updates)
    if (!graph.has_adjacency_index()) {
        graph.build_adjacency_index();
    }

    #pragma omp parallel for schedule(static) if(num_nodes > 50)
    for (size_t i = 0; i < num_nodes; ++i) {
        auto& node = nodes[i];
        Vec3 displacement = node.position - node.last_frame_position;
        if (displacement.length_squared() > frame_update_threshold_sq) {
            // Recompute stitch_axis from continuity neighbors
            auto [prev_id, next_id] = graph.get_continuity_neighbors(node.id);
            Vec3 axis = Vec3::unit_x();
            if (prev_id != static_cast<NodeId>(-1) && next_id != static_cast<NodeId>(-1)) {
                Vec3 dir = nodes[next_id].position - nodes[prev_id].position;
                if (dir.length_squared() > 1e-12f) axis = dir.normalized();
            } else if (next_id != static_cast<NodeId>(-1)) {
                Vec3 dir = nodes[next_id].position - node.position;
                if (dir.length_squared() > 1e-12f) axis = dir.normalized();
            } else if (prev_id != static_cast<NodeId>(-1)) {
                Vec3 dir = node.position - nodes[prev_id].position;
                if (dir.length_squared() > 1e-12f) axis = dir.normalized();
            }
            node.stitch_axis = axis;
            node.last_frame_position = node.position;
        }
    }

    // --- Compute world-space AABBs for each node ---
    // The bounding half-extent is in local stitch frame (course, wale, normal).
    // We rotate by the local frame and take the axis-aligned envelope.
    struct NodeAABB {
        Vec3 min_pt;
        Vec3 max_pt;
    };
    std::vector<NodeAABB> aabbs(num_nodes);

    #pragma omp parallel for schedule(static) if(num_nodes > 50)
    for (size_t i = 0; i < num_nodes; ++i) {
        const auto& node = nodes[i];
        Vec3 half = node.shape.bounding_half_extent(node.shape.z_bulge != 0.0f ?
            std::abs(node.shape.z_bulge) * 0.1f : min_distance * 0.5f);

        // If bounding half-extent is zero/tiny (non-loop nodes), use min_distance as fallback
        if (half.x < min_distance * 0.5f) half.x = min_distance * 0.5f;
        if (half.y < min_distance * 0.5f) half.y = min_distance * 0.5f;
        if (half.z < min_distance * 0.5f) half.z = min_distance * 0.5f;

        // Local frame axes
        Vec3 course = node.stitch_axis;
        // Wale axis: perpendicular to course in the vertical plane
        Vec3 up = Vec3::unit_y();
        Vec3 wale = up - course * course.dot(up);
        float wale_len = wale.length();
        if (wale_len > 1e-6f) {
            wale = wale / wale_len;
        } else {
            wale = Vec3::unit_z();
        }
        Vec3 normal = course.cross(wale);

        // World-space extent: |half.x * course| + |half.y * wale| + |half.z * normal|
        // For AABB, take the absolute value of each component contribution
        Vec3 extent;
        extent.x = std::abs(half.x * course.x) + std::abs(half.y * wale.x) + std::abs(half.z * normal.x);
        extent.y = std::abs(half.x * course.y) + std::abs(half.y * wale.y) + std::abs(half.z * normal.y);
        extent.z = std::abs(half.x * course.z) + std::abs(half.y * wale.z) + std::abs(half.z * normal.z);

        aabbs[i].min_pt = node.position - extent;
        aabbs[i].max_pt = node.position + extent;
    }

    // --- Build spatial hash grid ---
    // Cell size = max AABB dimension across all nodes (ensures overlapping boxes are in adjacent cells)
    float max_extent = 0.0f;
    for (size_t i = 0; i < num_nodes; ++i) {
        Vec3 size = aabbs[i].max_pt - aabbs[i].min_pt;
        max_extent = std::max(max_extent, std::max(size.x, std::max(size.y, size.z)));
    }
    float cell_size = std::max(max_extent, min_distance);  // At least min_distance

    if (cell_size < 1e-6f) return;  // Degenerate case

    float inv_cell = 1.0f / cell_size;

    // Hash function for grid cell
    auto cell_hash = [](int cx, int cy, int cz) -> int64_t {
        // Use a large prime-based hash to avoid collisions
        return static_cast<int64_t>(cx) * 73856093LL
             ^ static_cast<int64_t>(cy) * 19349669LL
             ^ static_cast<int64_t>(cz) * 83492791LL;
    };

    // Insert nodes into grid cells (each node may span multiple cells)
    std::unordered_map<int64_t, std::vector<NodeId>> grid;

    for (size_t i = 0; i < num_nodes; ++i) {
        int min_cx = static_cast<int>(std::floor(aabbs[i].min_pt.x * inv_cell));
        int min_cy = static_cast<int>(std::floor(aabbs[i].min_pt.y * inv_cell));
        int min_cz = static_cast<int>(std::floor(aabbs[i].min_pt.z * inv_cell));
        int max_cx = static_cast<int>(std::floor(aabbs[i].max_pt.x * inv_cell));
        int max_cy = static_cast<int>(std::floor(aabbs[i].max_pt.y * inv_cell));
        int max_cz = static_cast<int>(std::floor(aabbs[i].max_pt.z * inv_cell));

        for (int cx = min_cx; cx <= max_cx; ++cx) {
            for (int cy = min_cy; cy <= max_cy; ++cy) {
                for (int cz = min_cz; cz <= max_cz; ++cz) {
                    grid[cell_hash(cx, cy, cz)].push_back(static_cast<NodeId>(i));
                }
            }
        }
    }

    // --- Check candidate pairs within each grid cell ---
    // Collect unique cell keys for parallel iteration
    std::vector<int64_t> cell_keys;
    cell_keys.reserve(grid.size());
    for (const auto& [key, _] : grid) {
        cell_keys.push_back(key);
    }

    // Thread-local force accumulation
    #pragma omp parallel if(num_nodes > 50)
    {
        std::vector<Vec3> thread_forces(num_nodes, Vec3::zero());

        #pragma omp for schedule(dynamic)
        for (size_t ci = 0; ci < cell_keys.size(); ++ci) {
            const auto& cell_nodes = grid[cell_keys[ci]];

            for (size_t a = 0; a < cell_nodes.size(); ++a) {
                NodeId id_a = cell_nodes[a];

                for (size_t b = a + 1; b < cell_nodes.size(); ++b) {
                    NodeId id_b = cell_nodes[b];

                    // Ensure consistent ordering to avoid double-counting
                    NodeId lo = std::min(id_a, id_b);
                    NodeId hi = std::max(id_a, id_b);

                    // Skip connected pairs using the cached skip list
                    if (!skip_list.empty() && lo < skip_list.size()) {
                        const auto& skips = skip_list[lo];
                        if (std::find(skips.begin(), skips.end(), hi) != skips.end()) {
                            continue;
                        }
                    }

                    // AABB overlap test (narrow-phase)
                    if (aabbs[lo].min_pt.x > aabbs[hi].max_pt.x ||
                        aabbs[hi].min_pt.x > aabbs[lo].max_pt.x ||
                        aabbs[lo].min_pt.y > aabbs[hi].max_pt.y ||
                        aabbs[hi].min_pt.y > aabbs[lo].max_pt.y ||
                        aabbs[lo].min_pt.z > aabbs[hi].max_pt.z ||
                        aabbs[hi].min_pt.z > aabbs[lo].max_pt.z) {
                        continue;  // No overlap
                    }

                    // Compute penetration depth per axis, pick minimum for separation
                    float overlap_x = std::min(aabbs[lo].max_pt.x - aabbs[hi].min_pt.x,
                                                aabbs[hi].max_pt.x - aabbs[lo].min_pt.x);
                    float overlap_y = std::min(aabbs[lo].max_pt.y - aabbs[hi].min_pt.y,
                                                aabbs[hi].max_pt.y - aabbs[lo].min_pt.y);
                    float overlap_z = std::min(aabbs[lo].max_pt.z - aabbs[hi].min_pt.z,
                                                aabbs[hi].max_pt.z - aabbs[lo].min_pt.z);

                    if (overlap_x <= 0 || overlap_y <= 0 || overlap_z <= 0) {
                        continue;  // No actual overlap
                    }

                    // Separation direction: axis with minimum penetration
                    Vec3 delta = nodes[hi].position - nodes[lo].position;
                    Vec3 sep_dir;
                    float pen_depth;

                    if (overlap_x <= overlap_y && overlap_x <= overlap_z) {
                        sep_dir = Vec3(delta.x >= 0 ? 1.0f : -1.0f, 0.0f, 0.0f);
                        pen_depth = overlap_x;
                    } else if (overlap_y <= overlap_z) {
                        sep_dir = Vec3(0.0f, delta.y >= 0 ? 1.0f : -1.0f, 0.0f);
                        pen_depth = overlap_y;
                    } else {
                        sep_dir = Vec3(0.0f, 0.0f, delta.z >= 0 ? 1.0f : -1.0f);
                        pen_depth = overlap_z;
                    }

                    // Compute the maximum possible penetration for normalization
                    Vec3 half_a = aabbs[lo].max_pt - nodes[lo].position;
                    Vec3 half_b = aabbs[hi].max_pt - nodes[hi].position;
                    float max_pen = std::abs(half_a.dot(sep_dir)) + std::abs(half_b.dot(sep_dir));
                    if (max_pen < 1e-6f) max_pen = 1e-6f;

                    // Sigmoid force based on overlap ratio (same model as barrier forces)
                    float overlap_ratio = std::min(pen_depth / max_pen, 1.0f);
                    float t = overlap_ratio * 4.0f;
                    float force_magnitude = strength * t / std::sqrt(1.0f + t * t);

                    Vec3 force = sep_dir * force_magnitude;

                    thread_forces[lo] -= force;
                    thread_forces[hi] += force;
                }
            }
        }

        // Merge thread-local forces
        #pragma omp critical
        {
            for (size_t i = 0; i < num_nodes; ++i) {
                nodes[i].force += thread_forces[i];
            }
        }
    }
}

}  // namespace yarnpath
