#include "physical_loop.hpp"
#include "logging.hpp"
#include <cmath>

namespace yarnpath {

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
    loop.yarn_compressed_radius = yarn.compressed_radius;

    // Position - this is where the needle axis passes through
    loop.center = position;
    loop.normal = up_direction.normalized();

    float needle_radius = gauge.needle_diameter * 0.5f;
    float wrap_radius = needle_radius + yarn.compressed_radius;

    constexpr float PI = 3.14159265f;

    float theta_front = -PI * 0.5f;
    float theta_back = PI * 0.5f;

    loop.entry_point = Vec3(
        position.x,
        position.y,
        wrap_radius * std::sin(theta_front)
    );

    loop.exit_point = Vec3(
        position.x,
        position.y,
        wrap_radius * std::sin(theta_back)
    );

    loop.entry_tangent = Vec3(0.0f, 1.0f, 0.0f);
    loop.exit_tangent = Vec3(0.0f, -0.3f, -1.0f).normalized();

    loop.apex_point = Vec3(
        position.x,
        position.y + wrap_radius,
        0.0f
    );

    loop.base_point = Vec3(
        position.x,
        position.y - wrap_radius,
        0.0f
    );

    loop.opening_center = position;

    loop.generate_shape(gauge);

    return loop;
}

void PhysicalLoop::generate_shape(const Gauge& gauge) {
    shape = CatmullRomSpline();

    float needle_radius = gauge.needle_diameter * 0.5f;
    float wrap_radius = needle_radius + yarn_compressed_radius;

    auto cylinder_point = [&](float theta, float x) -> Vec3 {
        return Vec3(
            x,
            center.y + wrap_radius * std::cos(theta),
            wrap_radius * std::sin(theta)
        );
    };

    constexpr float PI = 3.14159265f;

    // Generate waypoints around the needle wrap arc (front to back, 180 degrees)
    // Use enough waypoints for a smooth approximation
    float theta_front = -PI * 0.5f;
    float theta_back = PI * 0.5f;
    float loop_x = center.x;

    constexpr int NUM_WAYPOINTS = 9;  // 9 points for 180-degree arc
    for (int i = 0; i < NUM_WAYPOINTS; ++i) {
        float t = static_cast<float>(i) / static_cast<float>(NUM_WAYPOINTS - 1);
        float theta = theta_front + t * (theta_back - theta_front);
        shape.add_waypoint(cylinder_point(theta, loop_x));
    }
}

bool PhysicalLoop::segment_passes_through(const Vec3& p1, const Vec3& p2) const {
    Vec3 d = p2 - p1;
    float denom = d.dot(normal);

    if (std::abs(denom) < 1e-8f) {
        return false;
    }

    float t = (opening_center - p1).dot(normal) / denom;

    if (t < 0.0f || t > 1.0f) {
        return false;
    }

    Vec3 intersection = p1 + d * t;

    Vec3 offset = intersection - opening_center;
    Vec3 in_plane = offset - normal * offset.dot(normal);
    float dist = in_plane.length();

    return dist <= opening_radius();
}

}  // namespace yarnpath
