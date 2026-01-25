#include "surface_solver.hpp"
#include "surface_forces.hpp"
#include "logging.hpp"
#include <cmath>

namespace yarnpath {

SolveResult SurfaceSolver::solve(SurfaceGraph& graph,
                                  const YarnProperties& yarn,
                                  const SolveConfig& config) {
    auto log = yarnpath::logging::get_logger();

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

    for (int iter = 0; iter < config.max_iterations; ++iter) {
        // Single solver step
        step(graph, yarn, config);

        // Check convergence
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
                          const SolveConfig& config) {
    // 1. Compute all forces (including gravity and collision repulsion)
    compute_forces(graph, yarn, config.force_config);

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

    for (auto& node : graph.nodes()) {
        if (node.is_pinned) {
            // Pinned nodes don't move
            node.velocity = vec3::zero();
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
    for (int i = 0; i < iterations; ++i) {
        for (const auto& constraint : graph.constraints()) {
            project_constraint(graph, constraint);
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
        float current_dist = delta.length();

        switch (constraint.type) {
            case ConstraintType::MaxStretch:
                // Check if distance exceeds limit (with tolerance)
                if (current_dist > constraint.limit + tolerance) {
                    return false;
                }
                break;

            case ConstraintType::MinDistance:
                // Check if distance is less than limit (with tolerance)
                if (current_dist < constraint.limit - tolerance) {
                    return false;
                }
                break;
        }
    }
    return true;
}

}  // namespace yarnpath
