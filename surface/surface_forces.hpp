#ifndef YARNPATH_SURFACE_FORCES_HPP
#define YARNPATH_SURFACE_FORCES_HPP

#include "surface_graph.hpp"
#include <yarn/yarn_properties.hpp>
#include <yarn/gauge.hpp>

namespace yarnpath {

// Configuration for force computation
struct ForceConfig {
    // Damping factor (0 = no damping, 1 = critically damped)
    float damping = 0.5f;

    // Additional passthrough tension multiplier based on yarn tension
    // REDUCED: was causing rows to pull too tight, fighting against spring rest length
    float passthrough_tension_factor = 0.1f;

    // Loop curvature force (encourages natural loop shape)
    float loop_curvature_strength = 0.1f;


    // Gravity configuration
    float gravity_strength = 9.8f;      // Gravity magnitude (m/s^2 or similar units)
    Vec3 gravity_direction{0, 1, 0};    // Default: positive Y is "down"
    bool enable_gravity = true;         // Whether to apply gravity

    // Floor configuration
    float floor_position = 0.0f;        // Floor Y position (nodes can't go below this)
    bool enable_floor = false;          // Whether to enforce floor constraint

    // Collision configuration
    bool enable_collision = false;      // Whether to apply collision repulsion (opt-in for performance)
    float collision_strength = 100.0f;  // Repulsion force strength

    // Bending resistance (prevents sharp folds along yarn path)
    bool enable_bending_resistance = true;  // Whether to apply bending forces
    float bending_stiffness = 50.0f;        // Force strength resisting sharp bends
    float min_bend_angle = 0.5f;            // Minimum allowed bend angle (radians, ~30 degrees)
};

// Compute all forces on the graph nodes
void compute_forces(SurfaceGraph& graph,
                    const YarnProperties& yarn,
                    const Gauge& gauge,
                    const ForceConfig& config = ForceConfig{});

// Individual force components (for testing and debugging)

// Spring force: F = -k * (length - rest_length) * direction
void compute_spring_forces(SurfaceGraph& graph);

// Passthrough tension: additional straightening force for through-loop edges
void compute_passthrough_tension(SurfaceGraph& graph,
                                  const YarnProperties& yarn,
                                  const Gauge& gauge,
                                  float tension_factor);

// Loop curvature: gentle force toward natural loop shape
void compute_loop_curvature_forces(SurfaceGraph& graph,
                                    const YarnProperties& yarn,
                                    const Gauge& gauge,
                                    float strength);


// Damping force: F = -damping * velocity
void apply_damping(SurfaceGraph& graph, float damping);

// Gravity force: F = mass * gravity_strength * gravity_direction
void compute_gravity_force(SurfaceGraph& graph,
                           float strength,
                           const Vec3& direction);

// Apply floor constraint (prevent nodes from going past floor along gravity direction)
void apply_floor_constraint(SurfaceGraph& graph, float floor_dist, const Vec3& direction);

// Collision repulsion force: pushes non-adjacent nodes apart when too close
void compute_collision_forces(SurfaceGraph& graph, float min_distance, float strength);

// Bending resistance: prevents sharp bends/folds along continuity edges
void compute_bending_forces(SurfaceGraph& graph, float stiffness, float min_angle);

}  // namespace yarnpath

#endif // YARNPATH_SURFACE_FORCES_HPP
