#include "curvature_utils.hpp"
#include <cmath>
#include <algorithm>

namespace yarnpath {

Vec3 safe_normalized(const Vec3& v, const Vec3& fallback) {
    float len = v.length();
    return (len > 1e-8f) ? v * (1.0f / len) : fallback;
}

bool has_valid_control_points(const CubicBezier& curve) {
    for (const auto& cp : curve.control_points) {
        if (std::isnan(cp.x) || std::isnan(cp.y) || std::isnan(cp.z) ||
            std::isinf(cp.x) || std::isinf(cp.y) || std::isinf(cp.z)) {
            return false;
        }
    }
    return true;
}

std::vector<CubicBezier> subdivide_for_curvature_dense(
    const CubicBezier& curve, float max_k, int max_depth) {
    if (max_depth <= 0 || curve.max_curvature(CURVATURE_SAMPLES) <= max_k) {
        return {curve};
    }
    auto [left, right] = curve.split(0.5f);
    auto left_result = subdivide_for_curvature_dense(left, max_k, max_depth - 1);
    auto right_result = subdivide_for_curvature_dense(right, max_k, max_depth - 1);
    left_result.insert(left_result.end(), right_result.begin(), right_result.end());
    return left_result;
}

std::pair<float, float> compute_curvature_safe_magnitudes(
    const Vec3& d0_unit, const Vec3& d1_unit,
    const Vec3& chord, float max_k) {

    float dist = chord.length();
    if (dist < 1e-6f || max_k < 1e-8f) {
        return {0.0f, 0.0f};
    }

    // Precompute cross products
    Vec3 d0xD = d0_unit.cross(chord);
    Vec3 d0xd1 = d0_unit.cross(d1_unit);
    Vec3 d1xD = d1_unit.cross(chord);

    // Start with rule-of-thirds as baseline
    float m0 = dist / 3.0f;
    float m1 = dist / 3.0f;

    // Iteratively solve the coupled endpoint curvature constraints
    for (int iter = 0; iter < 3; ++iter) {
        // κ(0) = 6 * |d0×D - (m1/3)(d0×d1)| / m0²  ≤ max_k
        Vec3 num0_vec = d0xD - d0xd1 * (m1 / 3.0f);
        float num0 = num0_vec.length();
        if (num0 > 1e-8f) {
            float needed_m0 = std::sqrt(6.0f * num0 / max_k);
            m0 = std::max(m0, needed_m0);
        }

        // κ(1) = 6 * |d1×D + (m0/3)(d0×d1)| / m1²  ≤ max_k
        // (sign: d1×d0 = -(d0×d1), so d1×(-D+m0*d0/3) = -d1×D-(m0/3)(d0×d1)
        //  magnitude = |d1×D + (m0/3)(d0×d1)| )
        Vec3 num1_vec = d1xD + d0xd1 * (m0 / 3.0f);
        float num1 = num1_vec.length();
        if (num1 > 1e-8f) {
            float needed_m1 = std::sqrt(6.0f * num1 / max_k);
            m1 = std::max(m1, needed_m1);
        }
    }

    // Safety factor for interior curvature peaks (endpoints don't bound the whole curve).
    // The interior peak can be up to ~2x the endpoint curvature for S-curves,
    // so we use a generous factor to ensure compliance.
    m0 *= 2.0f;
    m1 *= 2.0f;

    // Cap at 2*dist to prevent self-intersecting control polygons
    m0 = std::min(m0, dist * 2.0f);
    m1 = std::min(m1, dist * 2.0f);

    return {m0, m1};
}

std::vector<CubicBezier> build_curvature_safe_hermite_segments(
    const Vec3& start, const Vec3& start_dir,
    const Vec3& end, const Vec3& end_dir,
    float max_k, int max_splits) {

    Vec3 chord = end - start;
    float dist = chord.length();
    if (dist < 1e-6f) return {CubicBezier(start, start, end, end)};

    Vec3 d0 = safe_normalized(start_dir);
    Vec3 d1 = safe_normalized(end_dir);

    auto [m0, m1] = compute_curvature_safe_magnitudes(d0, d1, chord, max_k);

    auto curve = CubicBezier::from_hermite(start, d0 * m0, end, d1 * m1);

    float actual_k = curve.max_curvature(CURVATURE_SAMPLES);
    if (actual_k <= max_k) {
        return {curve};
    }

    // Interior curvature exceeds limit. This typically happens with S-curves
    // where start_dir and end_dir create an inflection. Split into two curves
    // through the midpoint with a blended intermediate direction.
    if (max_splits <= 0) {
        // Can't split further; return best-effort curve
        return {curve};
    }

    Vec3 mid = (start + end) * 0.5f;
    // Intermediate direction: blend of chord direction and average of endpoint dirs
    Vec3 chord_dir = safe_normalized(chord);
    Vec3 mid_dir = safe_normalized(chord_dir * 0.6f + (d0 + d1) * 0.2f, chord_dir);

    auto left = build_curvature_safe_hermite_segments(
        start, start_dir, mid, mid_dir, max_k, max_splits - 1);
    auto right = build_curvature_safe_hermite_segments(
        mid, mid_dir, end, end_dir, max_k, max_splits - 1);

    left.insert(left.end(), right.begin(), right.end());
    return left;
}

CubicBezier build_curvature_safe_hermite(
    const Vec3& start, const Vec3& start_dir,
    const Vec3& end, const Vec3& end_dir,
    float max_k) {

    Vec3 chord = end - start;
    float dist = chord.length();
    if (dist < 1e-6f) return CubicBezier(start, start, end, end);

    Vec3 d0 = safe_normalized(start_dir);
    Vec3 d1 = safe_normalized(end_dir);

    auto [m0, m1] = compute_curvature_safe_magnitudes(d0, d1, chord, max_k);
    return CubicBezier::from_hermite(start, d0 * m0, end, d1 * m1);
}

}  // namespace yarnpath
