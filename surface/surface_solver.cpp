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

    // 2. Gradient descent step with global displacement normalization
    integrate_gradient_step(graph, config.dt, config.max_displacement_per_step);
    auto t2 = Clock::now();

    // 3. Apply floor constraint if enabled
    if (config.force_config.enable_floor) {
        apply_floor_constraint(graph, config.force_config.floor_position,
                               config.force_config.gravity_direction);
    }
    auto t3 = Clock::now();

    if (should_time) {
        float forces_ms = std::chrono::duration<float, std::milli>(t1 - t0).count();
        float integrate_ms = std::chrono::duration<float, std::milli>(t2 - t1).count();
        float floor_ms = std::chrono::duration<float, std::milli>(t3 - t2).count();
        log->debug("  step {}: forces={:.1f}ms integrate={:.1f}ms floor={:.1f}ms",
                   step_count, forces_ms, integrate_ms, floor_ms);
    }
    ++step_count;
}

void SurfaceSolver::integrate_gradient_step(SurfaceGraph& graph, float dt,
                                             float max_displacement) {
    // Gradient descent with global displacement normalization:
    // 1. Compute raw displacement for each node: d = (F/m) * dt  (single dt, not dt²)
    // 2. Find max displacement across all nodes
    // 3. If max exceeds clamp, scale ALL equally (preserves force ratios)
    // 4. Apply scaled displacements

    auto& nodes = graph.nodes();
    const size_t n = nodes.size();

    // Pass 1: compute raw displacements and find global max
    std::vector<Vec3> displacements(n, Vec3::zero());
    float max_disp = 0.0f;

    #pragma omp parallel for schedule(static) reduction(max:max_disp) if(n > 50)
    for (size_t i = 0; i < n; ++i) {
        if (nodes[i].is_pinned) continue;
        float inv_mass = (nodes[i].mass > 1e-6f) ? (1.0f / nodes[i].mass) : 1.0f;
        displacements[i] = nodes[i].force * inv_mass * dt;  // single dt
        float disp_len = displacements[i].length();
        max_disp = std::max(max_disp, disp_len);
    }

    // Global normalization: if any node exceeds max, scale ALL equally
    float scale = (max_disp > max_displacement && max_disp > 1e-6f)
                  ? max_displacement / max_disp
                  : 1.0f;

    // Pass 2: apply scaled displacements
    #pragma omp parallel for schedule(static) if(n > 50)
    for (size_t i = 0; i < n; ++i) {
        auto& node = nodes[i];
        if (node.is_pinned) {
            node.velocity = Vec3::zero();
            continue;
        }
        Vec3 disp = displacements[i] * scale;
        node.velocity = disp * (1.0f / dt);  // store for floor constraint compat
        node.position += disp;
    }
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
