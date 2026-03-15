#include "surface_solver.hpp"
#include "surface_forces.hpp"
#include "logging.hpp"
#include <cmath>
#include <algorithm>
#include <chrono>
#include <queue>
#include <set>
#ifdef _OPENMP
#include <omp.h>
#endif

namespace yarnpath {

SolveResult SurfaceSolver::solve(SurfaceGraph& graph,
                                  const YarnProperties& yarn,
                                  const Gauge& gauge,
                                  const SolveConfig& config,
                                  const StepCallback& step_callback) {
    auto log = yarnpath::logging::get_logger();

    // Configure OpenMP thread count
    #ifdef _OPENMP
    int max_threads = omp_get_max_threads();
    int use_threads = (config.num_threads > 0) ? config.num_threads : max_threads;
    omp_set_num_threads(use_threads);
    log->info("Surface solver using {} OpenMP threads", use_threads);
    #else
    log->info("Surface solver running single-threaded (OpenMP not available)");
    #endif

    SolveResult result;
    result.initial_energy = graph.compute_energy();

    log->info("SurfaceSolver: starting solve with {} nodes, initial energy = {}",
              graph.node_count(), result.initial_energy);

    // Build collision skip list once (topology doesn't change during solve)
    std::vector<std::vector<NodeId>> collision_skip_list;
    if (config.force_config.enable_collision) {
        collision_skip_list.resize(graph.node_count());
        for (const auto& edge : graph.edges()) {
            NodeId a = std::min(edge.node_a, edge.node_b);
            NodeId b = std::max(edge.node_a, edge.node_b);
            collision_skip_list[a].push_back(b);
        }
        // Also skip adjacent nodes along the yarn path (i, i+1)
        for (NodeId i = 0; i + 1 < graph.node_count(); ++i) {
            collision_skip_list[i].push_back(i + 1);
        }
        // Sort and deduplicate each skip list for efficient lookup
        for (auto& skips : collision_skip_list) {
            std::sort(skips.begin(), skips.end());
            skips.erase(std::unique(skips.begin(), skips.end()), skips.end());
        }
        log->info("SurfaceSolver: built collision skip list for {} nodes", graph.node_count());
    }

    float prev_energy = result.initial_energy;

    using Clock = std::chrono::steady_clock;
    auto solve_start = Clock::now();

    for (int iter = 0; iter < config.max_iterations; ++iter) {
        auto iter_start = Clock::now();

        // Single solver step
        step(graph, yarn, gauge, config, collision_skip_list);

        auto iter_end = Clock::now();
        float iter_ms = std::chrono::duration<float, std::milli>(iter_end - iter_start).count();

        // Check convergence on every iteration
        float current_energy = graph.compute_energy();
        float energy_change = std::abs(current_energy - prev_energy);

        // Call step callback if provided
        if (step_callback) {
            bool should_continue = step_callback(graph, iter + 1, current_energy, energy_change);
            if (!should_continue) {
                result.converged = true;
                result.iterations = iter + 1;
                result.final_energy = current_energy;
                compute_fabric_normals(graph);
                log->info("SurfaceSolver: stopped by callback at iteration {}", result.iterations);
                return result;
            }
        }

        if (energy_change < config.convergence_threshold) {
            result.converged = true;
            result.iterations = iter + 1;
            result.final_energy = current_energy;

            compute_fabric_normals(graph);

            float total_s = std::chrono::duration<float>(Clock::now() - solve_start).count();
            log->info("SurfaceSolver: converged at iteration {} with energy = {} ({:.1f}s total, {:.1f}ms/iter)",
                      result.iterations, result.final_energy, total_s, iter_ms);
            return result;
        }

        prev_energy = current_energy;

        // Log progress: first 5 iterations, then every 100
        if (iter < 5 || (iter + 1) % 100 == 0) {
            float elapsed_s = std::chrono::duration<float>(Clock::now() - solve_start).count();
            log->debug("SurfaceSolver: iter {}, energy = {:.4f}, delta = {:.6f}, "
                       "{:.1f}ms/iter, {:.1f}s elapsed",
                       iter + 1, current_energy, energy_change, iter_ms, elapsed_s);
        }
    }

    // Did not converge
    result.converged = false;
    result.iterations = config.max_iterations;
    result.final_energy = graph.compute_energy();
    compute_fabric_normals(graph);

    float total_s = std::chrono::duration<float>(Clock::now() - solve_start).count();
    log->warn("SurfaceSolver: did not converge after {} iterations, final energy = {} ({:.1f}s total)",
              result.iterations, result.final_energy, total_s);

    return result;
}

void SurfaceSolver::step(SurfaceGraph& graph,
                          const YarnProperties& yarn,
                          const Gauge& gauge,
                          const SolveConfig& config,
                          const std::vector<std::vector<NodeId>>& collision_skip_list) {
    using Clock = std::chrono::steady_clock;
    static int step_count = 0;
    bool should_time = (step_count < 5 || step_count % 100 == 0);
    auto log = yarnpath::logging::get_logger();

    auto t0 = Clock::now();

    // 1. Compute all forces (including gravity and collision repulsion)
    compute_forces(graph, yarn, gauge, config.force_config, collision_skip_list);
    auto t1 = Clock::now();

    // 2. Integrate using Verlet
    integrate_verlet(graph, config.dt);
    auto t2 = Clock::now();

    // 3. Apply floor constraint if enabled
    if (config.force_config.enable_floor) {
        apply_floor_constraint(graph, config.force_config.floor_position,
                               config.force_config.gravity_direction);
    }
    auto t3 = Clock::now();

    // 4. Project constraints
    project_constraints(graph, config.constraint_iterations);
    auto t4 = Clock::now();

    if (should_time) {
        float forces_ms = std::chrono::duration<float, std::milli>(t1 - t0).count();
        float verlet_ms = std::chrono::duration<float, std::milli>(t2 - t1).count();
        float floor_ms = std::chrono::duration<float, std::milli>(t3 - t2).count();
        float constraints_ms = std::chrono::duration<float, std::milli>(t4 - t3).count();
        log->debug("  step {}: forces={:.1f}ms verlet={:.1f}ms floor={:.1f}ms constraints={:.1f}ms",
                   step_count, forces_ms, verlet_ms, floor_ms, constraints_ms);
    }
    ++step_count;
}

void SurfaceSolver::integrate_verlet(SurfaceGraph& graph, float dt) {
    // Velocity Verlet integration with proper mass handling
    // a = F / m
    // v(t + dt) = v(t) + a * dt
    // x(t + dt) = x(t) + v(t + dt) * dt

    auto& nodes = graph.nodes();

    // Parallelize over nodes - each node update is independent
    #pragma omp parallel for schedule(static) if(nodes.size() > 50)
    for (size_t i = 0; i < nodes.size(); ++i) {
        auto& node = nodes[i];
        if (node.is_pinned) {
            // Pinned nodes don't move
            node.velocity = Vec3::zero();
            continue;
        }

        // Compute acceleration: a = F / m
        float inv_mass = (node.mass > 1e-6f) ? (1.0f / node.mass) : 1.0f;
        Vec3 acceleration = node.force * inv_mass;

        // Update velocity: v = v + a * dt
        node.velocity += acceleration * dt;

        // Update position: x = x + v * dt
        node.position += node.velocity * dt;
    }
}

void SurfaceSolver::project_constraints(SurfaceGraph& graph, int iterations) {
    // Build constraint colors once for parallel projection
    // This partitions constraints into independent sets
    if (!graph.has_constraint_colors() && graph.constraint_count() > 0) {
        graph.build_constraint_colors();
    }

    const auto& colors = graph.constraint_colors();

    // If no colors (no constraints), nothing to do
    if (colors.empty()) {
        return;
    }

    for (int iter = 0; iter < iterations; ++iter) {
        // Process each color level sequentially
        // Constraints within the same color are independent and can be parallelized
        for (const auto& color_set : colors) {
            // Parallelize within each color - constraints don't conflict
            #pragma omp parallel for schedule(static) if(color_set.size() > 20)
            for (size_t i = 0; i < color_set.size(); ++i) {
                const auto& constraint = graph.constraint(color_set[i]);
                project_constraint(graph, constraint);
            }
        }
    }
}

void SurfaceSolver::project_constraint(SurfaceGraph& graph,
                                        const SurfaceConstraint& constraint) {
    auto& node_a = graph.node(constraint.node_a);
    auto& node_b = graph.node(constraint.node_b);

    // Skip if both are pinned
    if (node_a.is_pinned && node_b.is_pinned) {
        return;
    }

    Vec3 delta = node_b.position - node_a.position;
    float current_dist = delta.length();

    if (current_dist < 1e-6f) {
        return;  // Avoid division by zero
    }

    switch (constraint.type) {
        case ConstraintType::MaxStretch: {
            // If distance exceeds limit, pull nodes together
            if (current_dist > constraint.limit) {
                float correction = (current_dist - constraint.limit) / current_dist;
                Vec3 correction_vec = delta * correction;

                if (node_a.is_pinned) {
                    // Only move node_b
                    node_b.position -= correction_vec;
                } else if (node_b.is_pinned) {
                    // Only move node_a
                    node_a.position += correction_vec;
                } else {
                    // Move both equally
                    correction_vec *= 0.5f;
                    node_a.position += correction_vec;
                    node_b.position -= correction_vec;
                }
            }
            break;
        }

        case ConstraintType::MinDistance: {
            // If distance is less than limit, push nodes apart
            if (current_dist < constraint.limit) {
                float correction = (constraint.limit - current_dist) / current_dist;
                Vec3 correction_vec = delta * correction;

                if (node_a.is_pinned) {
                    // Only move node_b
                    node_b.position += correction_vec;
                } else if (node_b.is_pinned) {
                    // Only move node_a
                    node_a.position -= correction_vec;
                } else {
                    // Move both equally
                    correction_vec *= 0.5f;
                    node_a.position -= correction_vec;
                    node_b.position += correction_vec;
                }
            }
            break;
        }
    }
}

bool SurfaceSolver::constraints_satisfied(const SurfaceGraph& graph, float tolerance) {
    for (const auto& constraint : graph.constraints()) {
        const auto& node_a = graph.node(constraint.node_a);
        const auto& node_b = graph.node(constraint.node_b);

        Vec3 delta = node_b.position - node_a.position;
        // Use squared distance to avoid sqrt() - much faster
        float dist_sq = delta.length_squared();

        switch (constraint.type) {
            case ConstraintType::MaxStretch: {
                // Check if distance exceeds limit (with tolerance)
                float max_dist = constraint.limit + tolerance;
                if (dist_sq > max_dist * max_dist) {
                    return false;
                }
                break;
            }

            case ConstraintType::MinDistance: {
                // Check if distance is less than limit (with tolerance)
                float min_dist = constraint.limit - tolerance;
                if (dist_sq < min_dist * min_dist) {
                    return false;
                }
                break;
            }
        }
    }
    return true;
}

void SurfaceSolver::compute_fabric_normals(SurfaceGraph& graph) {
    // Build adjacency index if not already built (needed for neighbor lookup)
    if (!graph.has_adjacency_index()) {
        graph.build_adjacency_index();
    }

    auto& nodes = graph.nodes();
    const auto& edges = graph.edges();
    const size_t num_nodes = nodes.size();
    if (num_nodes == 0) return;

    // Pre-compute per-node passthrough topology (children and parents).
    // PassThrough edges: node_a = child, node_b = parent.
    std::vector<std::vector<NodeId>> node_children(num_nodes);
    std::vector<std::vector<NodeId>> node_parents(num_nodes);
    for (const auto& edge : edges) {
        if (edge.type != EdgeType::PassThrough) continue;
        node_children[edge.node_b].push_back(edge.node_a);
        node_parents[edge.node_a].push_back(edge.node_b);
    }

    // Build a set of PassThrough parent-child pairs for detecting row transitions.
    // If a continuity neighbor is also a parent or child (via PassThrough), the
    // continuity edge crosses a row boundary and should not be used for stitch_axis.
    std::set<std::pair<NodeId, NodeId>> passthrough_pairs;
    for (const auto& edge : edges) {
        if (edge.type != EdgeType::PassThrough) continue;
        passthrough_pairs.insert({edge.node_a, edge.node_b});
        passthrough_pairs.insert({edge.node_b, edge.node_a});
    }

    auto is_cross_row = [&](NodeId a, NodeId b) {
        return passthrough_pairs.count({a, b}) > 0;
    };

    // First pass: compute stitch_axis from continuity neighbors,
    // then correct it using passthrough topology.
    for (size_t i = 0; i < num_nodes; ++i) {
        auto& node = nodes[i];

        // Step 1: Raw stitch_axis from continuity neighbors.
        // Skip continuity neighbors that cross row boundaries (detected by
        // PassThrough edges between them) since they give a diagonal direction.
        auto [prev_id, next_id] = graph.get_continuity_neighbors(node.id);
        bool prev_valid = prev_id != static_cast<NodeId>(-1) &&
                          !is_cross_row(node.id, prev_id);
        bool next_valid = next_id != static_cast<NodeId>(-1) &&
                          !is_cross_row(node.id, next_id);

        if (prev_valid && next_valid) {
            Vec3 dir = nodes[next_id].position - nodes[prev_id].position;
            if (dir.length_squared() > 1e-12f) {
                node.stitch_axis = dir.normalized();
            }
        } else if (next_valid) {
            Vec3 dir = nodes[next_id].position - node.position;
            if (dir.length_squared() > 1e-12f) {
                node.stitch_axis = dir.normalized();
            }
        } else if (prev_valid) {
            Vec3 dir = node.position - nodes[prev_id].position;
            if (dir.length_squared() > 1e-12f) {
                node.stitch_axis = dir.normalized();
            }
        } else if (prev_id != static_cast<NodeId>(-1) && next_id != static_cast<NodeId>(-1)) {
            // Both neighbors cross rows — fall back to the old two-neighbor approach
            Vec3 dir = nodes[next_id].position - nodes[prev_id].position;
            if (dir.length_squared() > 1e-12f) {
                node.stitch_axis = dir.normalized();
            }
        } else if (next_id != static_cast<NodeId>(-1)) {
            Vec3 dir = nodes[next_id].position - node.position;
            if (dir.length_squared() > 1e-12f) {
                node.stitch_axis = dir.normalized();
            }
        } else if (prev_id != static_cast<NodeId>(-1)) {
            Vec3 dir = node.position - nodes[prev_id].position;
            if (dir.length_squared() > 1e-12f) {
                node.stitch_axis = dir.normalized();
            }
        }
        // else: keep default stitch_axis (unit_x)

        // Step 2: Derive physical wale direction from passthrough topology.
        // If this node has children or parents, the position vectors between
        // them define the true wale direction.  Project the wale component
        // out of stitch_axis so it lies purely in the course plane.
        Vec3 wale_dir = Vec3::zero();
        int wale_count = 0;
        for (NodeId child_id : node_children[i]) {
            wale_dir += nodes[child_id].position - node.position;
            wale_count++;
        }
        for (NodeId parent_id : node_parents[i]) {
            wale_dir += node.position - nodes[parent_id].position;
            wale_count++;
        }
        if (wale_count > 0) {
            wale_dir = wale_dir * (1.0f / wale_count);
            float wale_mag = wale_dir.length();
            if (wale_mag > 1e-6f) {
                wale_dir = wale_dir * (1.0f / wale_mag);

                // Project out the wale component from stitch_axis
                Vec3 course = node.stitch_axis;
                course = course - wale_dir * course.dot(wale_dir);
                float course_len = course.length();
                if (course_len > 1e-6f) {
                    node.stitch_axis = course * (1.0f / course_len);
                }
            }
        }

        // Step 3: Compute wale_axis and fabric_normal.
        // wale_axis always points parent→child (the row-stacking direction).
        // For nodes with topology, re-orthogonalize the topology-derived wale
        // against the corrected stitch_axis.  Otherwise fall back to Y-up.
        Vec3 course = node.stitch_axis;
        Vec3 wale;
        if (wale_count > 0 && wale_dir.length() > 1e-6f) {
            wale = wale_dir - course * course.dot(wale_dir);
        } else {
            Vec3 up = Vec3::unit_y();
            wale = up - course * course.dot(up);
        }
        float wale_len = wale.length();
        if (wale_len > 1e-6f) {
            wale = wale / wale_len;
        } else {
            wale = Vec3::unit_y();
        }
        node.wale_axis = wale;

        // fabric_normal = stitch_axis × wale (right-hand rule).
        // The sign depends on stitch_axis direction which alternates between
        // rows — the second pass below will fix consistency.
        Vec3 normal = course.cross(wale);
        float normal_len = normal.length();
        if (normal_len > 1e-6f) {
            normal = normal / normal_len;
        } else {
            normal = Vec3::unit_z();
        }
        node.fabric_normal = normal;
    }

    // Second pass: ensure consistent fabric_normal orientation across the
    // fabric using BFS through the edge graph.  The first pass produces
    // normals whose sign depends on stitch_axis direction, which alternates
    // between RS/WS rows.  BFS propagation flips only fabric_normal (not
    // stitch_axis or wale_axis) to make all normals point the same way.
    if (num_nodes > 1) {
        // Build adjacency list from all edges
        std::vector<std::vector<NodeId>> adj(num_nodes);
        for (const auto& edge : edges) {
            if (edge.node_a < num_nodes && edge.node_b < num_nodes) {
                adj[edge.node_a].push_back(edge.node_b);
                adj[edge.node_b].push_back(edge.node_a);
            }
        }

        // BFS from node 0
        std::vector<bool> visited(num_nodes, false);
        std::queue<NodeId> queue;
        queue.push(0);
        visited[0] = true;

        while (!queue.empty()) {
            NodeId curr = queue.front();
            queue.pop();

            for (NodeId neighbor : adj[curr]) {
                if (visited[neighbor]) continue;
                visited[neighbor] = true;

                if (nodes[neighbor].fabric_normal.dot(nodes[curr].fabric_normal) < 0.0f) {
                    nodes[neighbor].fabric_normal = nodes[neighbor].fabric_normal * -1.0f;
                }
                queue.push(neighbor);
            }
        }
    }
}

}  // namespace yarnpath
