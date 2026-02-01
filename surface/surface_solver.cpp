#include "surface_solver.hpp"
#include "surface_forces.hpp"
#include "logging.hpp"
#include <cmath>
#ifdef _OPENMP
#include <omp.h>
#endif

namespace yarnpath {

SolveResult SurfaceSolver::solve(SurfaceGraph& graph,
                                  const YarnProperties& yarn,
                                  const Gauge& gauge,
                                  const SolveConfig& config) {
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

    // Pre-solve phase: satisfy constraints before applying forces
    // This helps get the initial configuration into a valid state
    if (config.pre_solve_iterations > 0) {
        log->info("SurfaceSolver: running pre-solve constraint phase (max {} iterations)",
                  config.pre_solve_iterations);

        for (int iter = 0; iter < config.pre_solve_iterations; ++iter) {
            project_constraints(graph, config.constraint_iterations);

            // Check if all constraints are satisfied
            if (constraints_satisfied(graph)) {
                float post_presolve_energy = graph.compute_energy();
                log->info("SurfaceSolver: pre-solve converged at iteration {}, energy = {}",
                          iter + 1, post_presolve_energy);
                break;
            }

            // Log progress periodically
            if ((iter + 1) % 100 == 0) {
                float energy = graph.compute_energy();
                log->debug("SurfaceSolver: pre-solve iteration {}, energy = {}", iter + 1, energy);
            }
        }

        if (!constraints_satisfied(graph)) {
            float post_presolve_energy = graph.compute_energy();
            log->warn("SurfaceSolver: pre-solve did not satisfy all constraints, energy = {}",
                      post_presolve_energy);
        }
    }

    float prev_energy = graph.compute_energy();
    int energy_check_interval = 10;  // Adaptive: check every N iterations

    for (int iter = 0; iter < config.max_iterations; ++iter) {
        // Single solver step
        step(graph, yarn, gauge, config);

        // Check convergence periodically (not every iteration to save compute)
        bool should_check = ((iter + 1) % energy_check_interval == 0) ||
                           (iter == config.max_iterations - 1);

        if (should_check) {
            float current_energy = graph.compute_energy();
            float energy_change = std::abs(current_energy - prev_energy);

            if (energy_change < config.convergence_threshold) {
                result.converged = true;
                result.iterations = iter + 1;
                result.final_energy = current_energy;

                log->info("SurfaceSolver: converged at iteration {} with energy = {}",
                          result.iterations, result.final_energy);
                return result;
            }

            prev_energy = current_energy;

            // Log progress periodically
            if ((iter + 1) % 100 == 0) {
                log->debug("SurfaceSolver: iteration {}, energy = {}", iter + 1, current_energy);
            }

            // Adaptive interval: if energy change is large, check more frequently
            if (energy_change > config.convergence_threshold * 10.0f) {
                energy_check_interval = 5;   // Check frequently when far from convergence
            } else if (energy_change > config.convergence_threshold * 2.0f) {
                energy_check_interval = 10;  // Moderate checking
            } else {
                energy_check_interval = 20;  // Check infrequently when close to convergence
            }
        }
    }

    // Did not converge
    result.converged = false;
    result.iterations = config.max_iterations;
    result.final_energy = graph.compute_energy();

    log->warn("SurfaceSolver: did not converge after {} iterations, final energy = {}",
              result.iterations, result.final_energy);

    return result;
}

void SurfaceSolver::step(SurfaceGraph& graph,
                          const YarnProperties& yarn,
                          const Gauge& gauge,
                          const SolveConfig& config) {
    // 1. Compute all forces (including gravity and collision repulsion)
    compute_forces(graph, yarn, gauge, config.force_config);

    // 2. Integrate using Verlet
    integrate_verlet(graph, config.dt);

    // 3. Apply floor constraint if enabled
    if (config.force_config.enable_floor) {
        apply_floor_constraint(graph, config.force_config.floor_position,
                               config.force_config.gravity_direction);
    }

    // 4. Project constraints
    project_constraints(graph, config.constraint_iterations);
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
