#include "physical_loop.hpp"
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

    // Entry and exit are at the front of the needle (θ = -π/2)
    float theta_front = -PI * 0.5f;

    // Entry point: front of needle, left side of loop
    loop.entry_point = Vec3(
        position.x - loop.loop_width * 0.5f,
        position.y + wrap_radius * std::cos(theta_front),  // = position.y (cos(-90°) = 0)
        wrap_radius * std::sin(theta_front)                 // = -wrap_radius (front)
    );

    // Exit point: front of needle, right side of loop
    loop.exit_point = Vec3(
        position.x + loop.loop_width * 0.5f,
        position.y + wrap_radius * std::cos(theta_front),  // = position.y
        wrap_radius * std::sin(theta_front)                 // = -wrap_radius (front)
    );

    // Apex: at the top of the needle (θ = 0), used for visual reference
    // For full wrap, the "apex" is really the topmost point
    loop.apex_point = Vec3(
        position.x - loop.loop_width * 0.25f,  // Slightly left of center (where top is reached)
        position.y + wrap_radius,               // Top of the wrap
        0.0f                                    // Z = 0 at top
    );

    // Base point: at the bottom of the needle (θ = π)
    loop.base_point = Vec3(
        position.x + loop.loop_width * 0.25f,  // Slightly right of center (where bottom is reached)
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
    float half_width = loop_width * 0.5f;

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

    // Key angles for the wrap (going clockwise when viewed from +X)
    float theta_front = -PI * 0.5f;   // Front of needle (-90°)
    float theta_top = 0.0f;           // Top of needle (0°)
    float theta_back = PI * 0.5f;     // Back of needle (90°)
    float theta_bottom = PI;          // Bottom of needle (180°)
    float theta_exit = PI * 1.5f;     // Front again (270° = -90° + 360°)

    // Each 90° arc uses the same Bezier factor
    float arc_90 = PI * 0.5f;
    float k = bezier_k(arc_90);

    // X positions progress from left to right as we go around
    float x_entry = center.x - half_width;
    float x_top = center.x - half_width * 0.5f;
    float x_back = center.x;
    float x_bottom = center.x + half_width * 0.5f;
    float x_exit = center.x + half_width;

    Vec3 p0, p3, t0, t3;
    float dx;

    // === Segment 1: Front to Top (entry, going up) ===
    p0 = cylinder_point(theta_front, x_entry);
    p3 = cylinder_point(theta_top, x_top);
    t0 = arc_tangent(theta_front) * k;
    t3 = arc_tangent(theta_top) * (-k);
    dx = (p3.x - p0.x) * 0.4f;
    shape.add_segment(CubicBezier(p0, p0 + t0 + Vec3(dx, 0, 0), p3 + t3 + Vec3(-dx, 0, 0), p3));

    // === Segment 2: Top to Back ===
    p0 = p3;
    p3 = cylinder_point(theta_back, x_back);
    t0 = arc_tangent(theta_top) * k;
    t3 = arc_tangent(theta_back) * (-k);
    dx = (p3.x - p0.x) * 0.4f;
    shape.add_segment(CubicBezier(p0, p0 + t0 + Vec3(dx, 0, 0), p3 + t3 + Vec3(-dx, 0, 0), p3));

    // === Segment 3: Back to Bottom ===
    p0 = p3;
    p3 = cylinder_point(theta_bottom, x_bottom);
    t0 = arc_tangent(theta_back) * k;
    t3 = arc_tangent(theta_bottom) * (-k);
    dx = (p3.x - p0.x) * 0.4f;
    shape.add_segment(CubicBezier(p0, p0 + t0 + Vec3(dx, 0, 0), p3 + t3 + Vec3(-dx, 0, 0), p3));

    // === Segment 4: Bottom to Front (exit) ===
    // theta_exit = 3π/2 = 270°, but for cos/sin we use -π/2 (same position)
    p0 = p3;
    p3 = cylinder_point(theta_front, x_exit);  // Same angle as entry, different X
    t0 = arc_tangent(theta_bottom) * k;
    // At the exit (front), the tangent should point in direction of increasing theta
    // which is the same as at entry
    t3 = arc_tangent(theta_front) * (-k);
    dx = (p3.x - p0.x) * 0.4f;
    shape.add_segment(CubicBezier(p0, p0 + t0 + Vec3(dx, 0, 0), p3 + t3 + Vec3(-dx, 0, 0), p3));
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
