#include "surface_solver.hpp"
#include "surface_forces.hpp"
#include "logging.hpp"
#include <cmath>
#include <algorithm>
#include <chrono>
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
                log->info("SurfaceSolver: stopped by callback at iteration {}", result.iterations);
                return result;
            }
        }

        if (energy_change < config.convergence_threshold) {
            result.converged = true;
            result.iterations = iter + 1;
            result.final_energy = current_energy;

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

}  // namespace yarnpath
