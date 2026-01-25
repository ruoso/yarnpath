#include "physical_loop.hpp"
#include "logging.hpp"
#include <cmath>

namespace yarnpath {

LoopDimensions LoopDimensions::calculate(const YarnProperties& yarn, const Gauge& gauge) {
    LoopDimensions dim;

    // Loop opening is needle diameter minus yarn wrapping around both sides
    dim.opening_diameter = gauge.needle_diameter - 2.0f * yarn.radius;
    if (dim.opening_diameter < yarn.radius) {
        dim.opening_diameter = yarn.radius;  // Minimum opening
    }

    // Apply tension: tighter knitting = smaller loops
    dim.opening_diameter *= yarn.loop_size_factor();

    // Loop height: yarn wraps around needle, so height is related to
    // half the circumference of (needle + yarn thickness)
    float wrap_circumference = 3.14159f * (gauge.needle_diameter + 2.0f * yarn.radius);
    dim.loop_height = wrap_circumference * 0.5f * yarn.loop_aspect_ratio;

    // Loop width is similar to opening diameter plus yarn on both sides
    dim.loop_width = dim.opening_diameter + 2.0f * yarn.radius;

    // Total yarn length in one loop (approximate)
    dim.yarn_length = wrap_circumference * (1.0f + yarn.loop_slack);

    return dim;
}

PhysicalLoop PhysicalLoop::from_properties(
    const YarnProperties& yarn,
    const Gauge& gauge,
    const Vec3& position,
    const Vec3& up_direction) {

    PhysicalLoop loop;

    // Calculate dimensions
    auto dim = LoopDimensions::calculate(yarn, gauge);
    loop.opening_diameter = dim.opening_diameter;
    loop.loop_height = dim.loop_height;
    loop.loop_width = dim.loop_width;
    loop.yarn_radius = yarn.radius;

    // Position - this is where the needle axis passes through
    loop.center = position;
    loop.normal = up_direction.normalized();

    // === CYLINDER-BASED LOOP GEOMETRY ===
    // The needle is a cylinder with:
    // - Axis along X (horizontal)
    // - Center at 'position' (in the YZ plane)
    // - Radius = needle_diameter / 2
    //
    // The yarn wraps FULLY around the needle at a fixed distance from the axis.
    // In the YZ plane, the yarn centerline follows a circular arc.
    //
    // Angle convention (θ measured from +Y axis, positive toward +Z):
    //   θ = 0: top of needle (Y = +wrap_radius, Z = 0)
    //   θ = π/2: back of needle (Y = 0, Z = +wrap_radius)
    //   θ = π: bottom (Y = -wrap_radius, Z = 0)
    //   θ = -π/2: front (Y = 0, Z = -wrap_radius)

    float needle_radius = gauge.needle_diameter * 0.5f;
    float wrap_radius = needle_radius + yarn.radius;  // Distance from needle axis to yarn centerline

    constexpr float PI = 3.14159265f;

    // Entry is at the FRONT of the needle (θ = -π/2), exit is at the BACK (θ = π/2)
    // This way the yarn passes through from front to back naturally
    float theta_front = -PI * 0.5f;
    float theta_back = PI * 0.5f;

    // Entry point: front of needle, at loop center X
    // (X transitions happen in connectors, not in the loop shape)
    loop.entry_point = Vec3(
        position.x,                                      // Center X (constant in loop shape)
        position.y,                                      // At needle center height
        wrap_radius * std::sin(theta_front)              // = -wrap_radius (front)
    );

    // Exit point: back of needle, at loop center X
    loop.exit_point = Vec3(
        position.x,                                      // Center X (constant in loop shape)
        position.y,                                      // At needle center height
        wrap_radius * std::sin(theta_back)               // = +wrap_radius (back)
    );

    // Entry tangent: at front (θ = -π/2), the circular arc tangent points up (+Y)
    // No X component - let the connector curves handle horizontal movement
    loop.entry_tangent = Vec3(0.0f, 1.0f, 0.0f);

    // Exit tangent: at back (θ = π/2), the yarn continues toward front (-Z)
    // with slight downward curve. This allows smooth transition to the next loop
    // which is above and toward the front.
    loop.exit_tangent = Vec3(0.0f, -0.3f, -1.0f).normalized();

    // Apex: at the top of the needle (θ = 0), this is where child loops pass through
    // X is at center because the loop reaches center.x by the time it gets to the top
    // This means the X transition is 50% complete at the pass-through point
    loop.apex_point = Vec3(
        position.x,                             // Center X - halfway through the X transition
        position.y + wrap_radius,               // Top of the wrap
        0.0f                                    // Z = 0 at top
    );

    // Base point: at the bottom of the needle (θ = π)
    loop.base_point = Vec3(
        position.x,                             // Center X (constant in loop shape)
        position.y - wrap_radius,               // Bottom of the wrap
        0.0f                                    // Z = 0 at bottom
    );

    // Opening center is at the needle axis
    loop.opening_center = position;

    // Generate the shape from the cylinder wrap
    loop.generate_shape(gauge);

    return loop;
}

void PhysicalLoop::generate_shape(const Gauge& gauge) {
    shape = BezierSpline();

    // === CYLINDER-BASED LOOP SHAPE ===
    //
    // The needle is a cylinder:
    //   - Axis along X
    //   - Center at (center.x, center.y, 0) in any YZ cross-section
    //   - Radius = needle_diameter / 2
    //
    // The yarn wraps FULLY around the needle, staying at distance wrap_radius
    // from the needle axis. The yarn path is parameterized by:
    //   - θ (theta): angle around the cylinder in the YZ plane
    //   - X: position along the needle axis
    //
    // Convention for θ (measured from +Y axis, positive toward +Z):
    //   - θ = 0: top of needle (Y = +wrap_radius, Z = 0)
    //   - θ = π/2: back of needle (Y = 0, Z = +wrap_radius)
    //   - θ = π: bottom of needle (Y = -wrap_radius, Z = 0)
    //   - θ = -π/2: front of needle (Y = 0, Z = -wrap_radius)
    //
    // For a cast-on loop wrapping FULLY around the needle:
    //   - Entry: θ = -π/2 (front), X = left edge
    //   - Goes UP to θ = 0 (top)
    //   - Continues to θ = π/2 (back)
    //   - Down to θ = π (bottom)
    //   - Back around to θ = 3π/2 = -π/2 (front), X = right edge
    //   - Exit
    //
    // This creates a full 360° wrap around the needle.

    float needle_radius = gauge.needle_diameter * 0.5f;
    float wrap_radius = needle_radius + yarn_radius;

    // Point on cylinder surface given angle θ and X offset
    auto cylinder_point = [&](float theta, float x) -> Vec3 {
        return Vec3(
            x,
            center.y + wrap_radius * std::cos(theta),
            wrap_radius * std::sin(theta)
        );
    };

    // Tangent direction for circular arc (perpendicular to radius)
    // Points in direction of increasing θ
    auto arc_tangent = [&](float theta) -> Vec3 {
        return Vec3(
            0.0f,
            -wrap_radius * std::sin(theta),
            wrap_radius * std::cos(theta)
        );
    };

    // Bezier approximation factor for circular arcs
    // For arc angle α: k = (4/3) * tan(α/4)
    auto bezier_k = [](float arc_angle) -> float {
        return (4.0f / 3.0f) * std::tan(std::abs(arc_angle) / 4.0f);
    };

    constexpr float PI = 3.14159265f;

    // The loop wraps from FRONT to BACK over the top:
    // Entry at front (θ = -90°) → Top (θ = 0°) → Exit at back (θ = 90°)
    // This is a 180° wrap forming a U-shape open at the bottom
    float theta_front = -PI * 0.5f;      // Front of needle (-90°) - entry
    float theta_top = 0.0f;              // Top of needle (0°)
    float theta_back = PI * 0.5f;        // Back of needle (90°) - exit

    // Each 90° arc uses the same Bezier factor
    float arc_90 = PI * 0.5f;
    float k = bezier_k(arc_90);

    // Loop shape has CONSTANT X = center.x (touching the needle)
    // X transitions happen in the connector curves, not in the loop shape
    float loop_x = center.x;

    Vec3 p0, p3, t0, t3;

    // === Segment 1: Front to Top (constant X) ===
    p0 = cylinder_point(theta_front, loop_x);
    p3 = cylinder_point(theta_top, loop_x);
    t0 = arc_tangent(theta_front) * k;
    t3 = arc_tangent(theta_top) * (-k);
    CubicBezier seg1(p0, p0 + t0, p3 + t3, p3);
    yarnpath::logging::get_logger()->debug("loop_shape seg1 (front→top): p0=({:.3f},{:.3f},{:.3f}) p3=({:.3f},{:.3f},{:.3f}) max_curvature={:.3f}",
                 p0.x, p0.y, p0.z, p3.x, p3.y, p3.z, seg1.max_curvature());
    shape.add_segment(seg1);

    // === Segment 2: Top to Back (constant X) ===
    p0 = p3;
    p3 = cylinder_point(theta_back, loop_x);
    t0 = arc_tangent(theta_top) * k;
    t3 = arc_tangent(theta_back) * (-k);
    CubicBezier seg2(p0, p0 + t0, p3 + t3, p3);
    yarnpath::logging::get_logger()->debug("loop_shape seg2 (top→back): p0=({:.3f},{:.3f},{:.3f}) p3=({:.3f},{:.3f},{:.3f}) max_curvature={:.3f}",
                 p0.x, p0.y, p0.z, p3.x, p3.y, p3.z, seg2.max_curvature());
    shape.add_segment(seg2);
}

bool PhysicalLoop::segment_passes_through(const Vec3& p1, const Vec3& p2) const {
    // The opening is modeled as a disk in the XZ plane at opening_center
    // with radius = opening_radius()
    // The normal is in the Y direction (loop opens up/down)

    Vec3 d = p2 - p1;
    float denom = d.dot(normal);

    if (std::abs(denom) < 1e-8f) {
        // Line is parallel to the disk plane
        return false;
    }

    float t = (opening_center - p1).dot(normal) / denom;

    // Check if intersection is within the line segment
    if (t < 0.0f || t > 1.0f) {
        return false;
    }

    // Find intersection point
    Vec3 intersection = p1 + d * t;

    // Check if intersection is within the disk radius
    Vec3 offset = intersection - opening_center;
    // Project offset onto the disk plane (remove normal component)
    Vec3 in_plane = offset - normal * offset.dot(normal);
    float dist = in_plane.length();

    return dist <= opening_radius();
}

}  // namespace yarnpath
