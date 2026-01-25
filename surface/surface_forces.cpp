#include "surface_forces.hpp"
#include "logging.hpp"
#include <cmath>
#include <limits>

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

    // Add gravity if enabled
    if (config.enable_gravity && config.gravity_strength > 0) {
        compute_gravity_force(graph, config.gravity_strength, config.gravity_direction);
    }

    // Apply velocity damping
    apply_damping(graph, config.damping);
}

void compute_spring_forces(SurfaceGraph& graph) {
    for (const auto& edge : graph.edges()) {
        auto& node_a = graph.node(edge.node_a);
        auto& node_b = graph.node(edge.node_b);

        Vec3 delta = node_b.position - node_a.position;
        float length = delta.length();

        // Avoid division by zero
        if (length < 1e-6f) {
            continue;
        }

        // Spring force: F = -k * (length - rest_length) * direction
        float displacement = length - edge.rest_length;
        Vec3 direction = delta / length;
        Vec3 force = direction * (edge.stiffness * displacement);

        // Apply force to both nodes (Newton's third law)
        node_a.add_force(force);
        node_b.add_force(-force);
    }
}

void compute_passthrough_tension(SurfaceGraph& graph,
                                  const YarnProperties& yarn,
                                  float tension_factor) {
    // Additional tension force on passthrough edges to straighten yarn
    // Higher yarn tension = stronger straightening force

    float tension_strength = yarn.tension * tension_factor;

    for (const auto& edge : graph.edges()) {
        if (edge.type != EdgeType::PassThrough) {
            continue;
        }

        auto& node_a = graph.node(edge.node_a);
        auto& node_b = graph.node(edge.node_b);

        Vec3 delta = node_b.position - node_a.position;
        float length = delta.length();

        if (length < 1e-6f) {
            continue;
        }

        // Extra force pulling nodes together based on tension
        // This simulates the yarn being pulled taut through the loop
        Vec3 direction = delta / length;
        Vec3 force = direction * tension_strength;

        node_a.add_force(force);
        node_b.add_force(-force);
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

    // For each node that forms a loop, compute curvature-related forces
    // This is a simplified model: we look at the node's neighbors and
    // encourage a certain angle between incoming and outgoing edges

    auto& nodes = graph.nodes();
    auto& edges = graph.edges();

    for (auto& node : nodes) {
        if (!node.forms_loop) {
            continue;
        }

        // Find the previous and next nodes (if they exist via continuity edges)
        NodeId prev_id = static_cast<NodeId>(-1);
        NodeId next_id = static_cast<NodeId>(-1);

        for (const auto& edge : edges) {
            if (edge.type != EdgeType::YarnContinuity) {
                continue;
            }

            if (edge.node_b == node.id) {
                prev_id = edge.node_a;
            }
            if (edge.node_a == node.id) {
                next_id = edge.node_b;
            }
        }

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
            Vec3 up = vec3::unit_y();

            // Perpendicular to chord in the vertical plane
            Vec3 perp = up - chord_dir * chord_dir.dot(up);
            float perp_len = perp.length();

            if (perp_len < 1e-6f) {
                // Chord is vertical, use X instead
                up = vec3::unit_x();
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
    for (auto& node : graph.nodes()) {
        node.force -= node.velocity * damping;
    }
}

void compute_gravity_force(SurfaceGraph& graph,
                           float strength,
                           const Vec3& direction) {
    auto log = yarnpath::logging::get_logger();

    // F = m * g * direction
    Vec3 gravity_dir = direction.normalized();

    static int log_counter = 0;
    bool should_log = (log_counter++ % 1000 == 0);  // Log every 1000 calls

    float total_force = 0.0f;
    for (auto& node : graph.nodes()) {
        if (node.is_pinned) {
            continue;
        }
        Vec3 gravity_force = gravity_dir * (node.mass * strength);
        node.add_force(gravity_force);
        total_force += gravity_force.length();
    }

    if (should_log) {
        log->debug("Gravity: strength={}, dir=({},{},{}), total_force={}",
                   strength, gravity_dir.x, gravity_dir.y, gravity_dir.z, total_force);
    }
}

void apply_floor_constraint(SurfaceGraph& graph, float floor_z) {
    auto log = yarnpath::logging::get_logger();

    static int log_counter = 0;
    bool should_log = (log_counter++ % 1000 == 0);

    int nodes_on_floor = 0;
    float min_z = std::numeric_limits<float>::max();
    float max_z = std::numeric_limits<float>::lowest();

    // Prevent nodes from going below the floor (in Z direction)
    for (auto& node : graph.nodes()) {
        min_z = std::min(min_z, node.position.z);
        max_z = std::max(max_z, node.position.z);

        if (node.position.z > floor_z) {
            node.position.z = floor_z;
            // Also zero out downward velocity
            if (node.velocity.z > 0) {
                node.velocity.z = 0;
            }
            nodes_on_floor++;
        }
    }

    if (should_log) {
        log->debug("Floor: z={}, nodes_on_floor={}/{}, z_range=[{}, {}]",
                   floor_z, nodes_on_floor, graph.node_count(), min_z, max_z);
    }
}

Vec3 compute_dominant_plane_normal(const SurfaceGraph& graph) {
    // Compute the dominant plane from node positions using PCA-like approach
    // For knitting, the fabric typically lies in a plane, so we find that plane's normal

    if (graph.node_count() < 3) {
        return vec3::unit_z();  // Default to Z-up
    }

    // Compute centroid
    Vec3 centroid = vec3::zero();
    for (const auto& node : graph.nodes()) {
        centroid += node.position;
    }
    centroid = centroid / static_cast<float>(graph.node_count());

    // Compute covariance matrix (simplified: just find the direction with least variance)
    // For a flat fabric, the normal is perpendicular to the plane of highest variance

    // Sample approach: compute average normal from triangles formed by consecutive nodes
    Vec3 avg_normal = vec3::zero();
    int normal_count = 0;

    const auto& nodes = graph.nodes();
    for (size_t i = 2; i < nodes.size(); ++i) {
        Vec3 v1 = nodes[i-1].position - nodes[i-2].position;
        Vec3 v2 = nodes[i].position - nodes[i-2].position;
        Vec3 normal = v1.cross(v2);
        float len = normal.length();
        if (len > 1e-6f) {
            avg_normal += normal / len;
            normal_count++;
        }
    }

    if (normal_count > 0 && avg_normal.length() > 1e-6f) {
        return avg_normal.normalized();
    }

    // Fallback: assume fabric is in XY plane, normal is Z
    return vec3::unit_z();
}

}  // namespace yarnpath
