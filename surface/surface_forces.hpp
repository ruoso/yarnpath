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
    float passthrough_tension_factor = 1.0f;

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

// Determine the dominant plane from node positions (returns normal direction)
Vec3 compute_dominant_plane_normal(const SurfaceGraph& graph);

}  // namespace yarnpath

#endif // YARNPATH_SURFACE_FORCES_HPP
