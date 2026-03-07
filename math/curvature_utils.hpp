#ifndef YARNPATH_MATH_CURVATURE_UTILS_HPP
#define YARNPATH_MATH_CURVATURE_UTILS_HPP

#include <math/vec3.hpp>
#include <math/cubic_bezier.hpp>
#include <vector>
#include <utility>

namespace yarnpath {

// Number of samples to use for curvature checking
static constexpr int CURVATURE_SAMPLES = 32;

// Safe normalization that returns fallback instead of zero vector
Vec3 safe_normalized(const Vec3& v, const Vec3& fallback = Vec3(1,0,0));

// Check if a curve has valid (finite) control points
bool has_valid_control_points(const CubicBezier& curve);

// Compute tangent magnitudes for a Hermite curve that keep endpoint curvature ≤ max_k.
//
// For Hermite cubic Bezier with T0 = m0*d0, T1 = m1*d1, chord D = P1-P0:
//   κ(0) = 6 * |d0 × (D - m1*d1/3)| / m0²
//   κ(1) = 6 * |d1 × (-D + m0*d0/3)| / m1²
//
// Iteratively solves for the minimum m0, m1 satisfying both constraints,
// then applies a safety factor for interior curvature peaks.
std::pair<float, float> compute_curvature_safe_magnitudes(
    const Vec3& d0_unit, const Vec3& d1_unit,
    const Vec3& chord, float max_k);

// Build a single curvature-safe Hermite curve between two points.
// Computes tangent magnitudes analytically from the curvature constraint
// so that endpoint curvature stays within max_k.
CubicBezier build_curvature_safe_hermite(
    const Vec3& start, const Vec3& start_dir,
    const Vec3& end, const Vec3& end_dir,
    float max_k);

// Build a chain of smooth, C2-continuous curves through waypoints.
//
// Uses a clamped natural cubic spline to compute optimal tangent vectors
// (direction + magnitude) at each interior waypoint.  The entry tangent is
// fixed for C1 continuity with the preceding spline; interior tangent
// directions are solved via the tridiagonal natural-spline system that
// minimises integrated squared second-derivative (curvature energy).
// The exit tangent uses the natural (zero second-derivative) end condition.
//
// waypoints.size() >= 2.  entry_tangent is the full tangent vector
// (direction × magnitude) at waypoints[0].
// Returns waypoints.size()-1 CubicBezier curves.
std::vector<CubicBezier> build_curvature_safe_hermite_chain(
    const std::vector<Vec3>& waypoints,
    const Vec3& entry_tangent,
    float max_k);

}  // namespace yarnpath

#endif // YARNPATH_MATH_CURVATURE_UTILS_HPP
