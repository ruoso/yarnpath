#ifndef YARNPATH_MATH_CATMULL_ROM_SPLINE_HPP
#define YARNPATH_MATH_CATMULL_ROM_SPLINE_HPP

#include <math/vec3.hpp>
#include <array>
#include <vector>

namespace yarnpath {

// A centripetal Catmull-Rom spline through a sequence of waypoints.
//
// Waypoints are interpolated exactly.  Tangents at each waypoint are
// determined automatically from the surrounding points using centripetal
// parameterization (alpha = 0.5), which avoids cusps and self-intersections
// even with non-uniform spacing.
//
// Endpoints use reflected phantom points:
//   P_{-1}  = 2*P_0   - P_1
//   P_{N}   = 2*P_{N-1} - P_{N-2}
class CatmullRomSpline {
public:
    CatmullRomSpline() = default;

    // Add a single waypoint
    void add_waypoint(const Vec3& point);

    // Add multiple waypoints
    void add_waypoints(const std::vector<Vec3>& points);

    // Access waypoints
    const std::vector<Vec3>& waypoints() const { return waypoints_; }
    std::vector<Vec3>& waypoints() { return waypoints_; }
    size_t waypoint_count() const { return waypoints_.size(); }
    size_t segment_count() const;  // max(0, waypoint_count - 1)
    bool empty() const { return waypoints_.size() < 2; }

    // First / last waypoint
    Vec3 start() const;
    Vec3 end() const;

    // Evaluate position at global parameter t in [0, segment_count()]
    Vec3 evaluate(float t) const;

    // Tangent (first derivative) at global parameter t
    Vec3 tangent(float t) const;

    // Total arc length (numerical, via sampling)
    float total_arc_length(int samples_per_segment = 20) const;

    // Convert to polyline with approximate segment length
    std::vector<Vec3> to_polyline(float segment_length) const;

    // Convert to polyline with fixed number of samples per spline segment
    std::vector<Vec3> to_polyline_fixed(int samples_per_segment) const;

private:
    std::vector<Vec3> waypoints_;

    // Get the four surrounding points (P0, P1, P2, P3) for spline segment i,
    // where the segment interpolates between P1 and P2.
    // Uses reflected phantom points at boundaries.
    std::array<Vec3, 4> get_segment_points(size_t segment_index) const;

    // Centripetal Catmull-Rom evaluation between P1 and P2 given four
    // surrounding points P0..P3.  Parameter t in [0, 1].
    // Uses the Barry-Goldman recursive algorithm with alpha = 0.5.
    static Vec3 evaluate_segment(const Vec3& p0, const Vec3& p1,
                                  const Vec3& p2, const Vec3& p3, float t);

    // First derivative of the centripetal segment at parameter t in [0, 1].
    static Vec3 derivative_segment(const Vec3& p0, const Vec3& p1,
                                    const Vec3& p2, const Vec3& p3, float t);
};

}  // namespace yarnpath

#endif // YARNPATH_MATH_CATMULL_ROM_SPLINE_HPP
