#ifndef YARNPATH_GEOMETRY_CUBIC_BEZIER_HPP
#define YARNPATH_GEOMETRY_CUBIC_BEZIER_HPP

#include <math/vec3.hpp>
#include <array>
#include <vector>

namespace yarnpath {

// A single cubic Bezier curve segment
struct CubicBezier {
    std::array<Vec3, 4> control_points;

    // Constructors
    CubicBezier() = default;
    CubicBezier(const Vec3& p0, const Vec3& p1, const Vec3& p2, const Vec3& p3);

    // Evaluate position at parameter t in [0, 1]
    Vec3 evaluate(float t) const;

    // First derivative at parameter t
    Vec3 derivative(float t) const;

    // Second derivative at parameter t
    Vec3 second_derivative(float t) const;

    // Curvature at parameter t
    float curvature(float t) const;

    // Maximum curvature across the curve (sampled)
    float max_curvature(int samples = 10) const;

    // Approximate arc length (numerical integration)
    float arc_length(int samples = 20) const;

    // Get tangent vector (normalized derivative)
    Vec3 tangent(float t) const;

    // Get normal vector (perpendicular to tangent in the curve plane)
    Vec3 normal(float t) const;

    // Split curve at parameter t into two curves
    std::pair<CubicBezier, CubicBezier> split(float t) const;

    // Subdivide curve to ensure max curvature constraint
    std::vector<CubicBezier> subdivide_for_curvature(float max_curvature, int max_depth = 5) const;

    // Create from Hermite interpolation (positions and tangents at endpoints)
    static CubicBezier from_hermite(const Vec3& p0, const Vec3& tangent0,
                                     const Vec3& p1, const Vec3& tangent1);

    // Return a reversed copy of this curve (same shape, opposite direction)
    CubicBezier reversed() const {
        return CubicBezier(control_points[3], control_points[2],
                          control_points[1], control_points[0]);
    }

    // Access control points by name
    const Vec3& start() const { return control_points[0]; }
    const Vec3& control1() const { return control_points[1]; }
    const Vec3& control2() const { return control_points[2]; }
    const Vec3& end() const { return control_points[3]; }

    Vec3& start() { return control_points[0]; }
    Vec3& control1() { return control_points[1]; }
    Vec3& control2() { return control_points[2]; }
    Vec3& end() { return control_points[3]; }
};

// A point on a yarn path with associated curvature hint
struct YarnPathPoint {
    Vec3 position;
    float tension;  // 0.0 = smooth transition, 1.0 = tight loop

    YarnPathPoint() : position(), tension(0.5f) {}
    YarnPathPoint(const Vec3& pos, float t = 0.5f) : position(pos), tension(t) {}
};

// A spline composed of multiple Bezier segments
class BezierSpline {
public:
    BezierSpline() = default;
    explicit BezierSpline(std::vector<CubicBezier> segments);

    // Add a segment to the spline
    void add_segment(const CubicBezier& segment);

    // Access segments
    const std::vector<CubicBezier>& segments() const { return segments_; }
    std::vector<CubicBezier>& segments() { return segments_; }
    size_t segment_count() const { return segments_.size(); }
    bool empty() const { return segments_.empty(); }

    // Evaluate position at global parameter t in [0, segment_count]
    Vec3 evaluate(float t) const;

    // Get tangent at global parameter t
    Vec3 tangent(float t) const;

    // Total arc length
    float total_arc_length(int samples_per_segment = 20) const;

    // Maximum curvature across all segments
    float max_curvature(int samples_per_segment = 10) const;

    // Enforce C1 continuity (position and tangent) between segments
    void enforce_c1_continuity();

    // Clamp curvature by adjusting control points
    void clamp_curvature(float max_curvature);

    // Convert to polyline with approximate segment length
    std::vector<Vec3> to_polyline(float segment_length) const;

    // Convert to polyline with fixed number of samples per Bezier segment
    std::vector<Vec3> to_polyline_fixed(int samples_per_segment) const;

    // Merge segments shorter than min_length into adjacent segments
    void merge_short_segments(float min_length);

    // Combined cleanup: merge short segments, enforce continuity
    void cleanup(float min_segment_length);

private:
    std::vector<CubicBezier> segments_;
};

// Create a smooth continuation segment from a spline to a target point.
// Analyzes the last segment of the spline to determine incoming direction,
// then creates a curve that respects the max_curvature constraint.
//
// Parameters:
//   spline: The existing spline to continue from (uses last segment's exit tangent)
//   target_point: The destination point for the new segment
//   target_direction: The desired tangent direction at the target point (will be normalized)
//   max_curvature: Maximum allowed curvature (1/min_bend_radius)
//
// Returns: A CubicBezier segment from the spline's end to target_point
CubicBezier create_continuation_segment(
    const BezierSpline& spline,
    const Vec3& target_point,
    const Vec3& target_direction,
    float max_curvature);

// Create a smooth continuation segment that maintains clearance around a reference point.
// The curve will bulge away from the clearance point to ensure the minimum distance
// is maintained throughout the trajectory.
//
// Parameters:
//   spline: The existing spline to continue from (uses last segment's exit tangent)
//   target_point: The destination point for the new segment
//   target_direction: The desired tangent direction at the target point (will be normalized)
//   clearance_point: The center point that must be avoided
//   clearance_radius: The minimum distance to maintain from clearance_point
//   max_curvature: Maximum allowed curvature (1/min_bend_radius)
//
// Returns: A CubicBezier segment that curves around the clearance zone
CubicBezier create_continuation_segment_with_clearance(
    const BezierSpline& spline,
    const Vec3& target_point,
    const Vec3& target_direction,
    const Vec3& clearance_point,
    float clearance_radius,
    float max_curvature);

}  // namespace yarnpath

#endif // YARNPATH_GEOMETRY_CUBIC_BEZIER_HPP
