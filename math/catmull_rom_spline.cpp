#include <math/catmull_rom_spline.hpp>
#include <algorithm>
#include <cmath>
#include <cassert>

namespace yarnpath {

void CatmullRomSpline::add_waypoint(const Vec3& point) {
    waypoints_.push_back(point);
}

void CatmullRomSpline::add_waypoints(const std::vector<Vec3>& points) {
    waypoints_.insert(waypoints_.end(), points.begin(), points.end());
}

size_t CatmullRomSpline::segment_count() const {
    return waypoints_.size() < 2 ? 0 : waypoints_.size() - 1;
}

Vec3 CatmullRomSpline::start() const {
    assert(!waypoints_.empty());
    return waypoints_.front();
}

Vec3 CatmullRomSpline::end() const {
    assert(!waypoints_.empty());
    return waypoints_.back();
}

std::array<Vec3, 4> CatmullRomSpline::get_segment_points(size_t seg) const {
    assert(seg < segment_count());
    const size_t n = waypoints_.size();

    // P1 and P2 are the segment endpoints
    const Vec3& p1 = waypoints_[seg];
    const Vec3& p2 = waypoints_[seg + 1];

    // P0: previous point, or reflected phantom
    Vec3 p0 = (seg > 0)
        ? waypoints_[seg - 1]
        : p1 * 2.0f - p2;  // 2*P1 - P2

    // P3: next-next point, or reflected phantom
    Vec3 p3 = (seg + 2 < n)
        ? waypoints_[seg + 2]
        : p2 * 2.0f - p1;  // 2*P2 - P1

    return {p0, p1, p2, p3};
}

// Barry-Goldman algorithm for centripetal Catmull-Rom (alpha = 0.5).
//
// Given knot times t0 < t1 < t2 < t3 computed from centripetal
// parameterization, and a query time t in [t1, t2], this performs the
// recursive interpolation that produces a C1-continuous curve passing
// exactly through P1 (at t1) and P2 (at t2).
Vec3 CatmullRomSpline::evaluate_segment(const Vec3& p0, const Vec3& p1,
                                          const Vec3& p2, const Vec3& p3,
                                          float t) {
    // Centripetal knot intervals: dt = |P_{i+1} - P_i|^0.5
    const float dt01 = std::pow(std::max((p1 - p0).length_squared(), 1e-12f), 0.25f);
    const float dt12 = std::pow(std::max((p2 - p1).length_squared(), 1e-12f), 0.25f);
    const float dt23 = std::pow(std::max((p3 - p2).length_squared(), 1e-12f), 0.25f);

    // Knot values
    const float t0 = 0.0f;
    const float t1 = t0 + dt01;
    const float t2 = t1 + dt12;
    const float t3 = t2 + dt23;

    // Map input t from [0,1] to [t1, t2]
    const float u = t1 + t * (t2 - t1);

    // First level
    const Vec3 a1 = p0 * ((t1 - u) / (t1 - t0)) + p1 * ((u - t0) / (t1 - t0));
    const Vec3 a2 = p1 * ((t2 - u) / (t2 - t1)) + p2 * ((u - t1) / (t2 - t1));
    const Vec3 a3 = p2 * ((t3 - u) / (t3 - t2)) + p3 * ((u - t2) / (t3 - t2));

    // Second level
    const Vec3 b1 = a1 * ((t2 - u) / (t2 - t0)) + a2 * ((u - t0) / (t2 - t0));
    const Vec3 b2 = a2 * ((t3 - u) / (t3 - t1)) + a3 * ((u - t1) / (t3 - t1));

    // Third level
    const Vec3 c  = b1 * ((t2 - u) / (t2 - t1)) + b2 * ((u - t1) / (t2 - t1));

    return c;
}

// Numerical derivative via central difference.
Vec3 CatmullRomSpline::derivative_segment(const Vec3& p0, const Vec3& p1,
                                            const Vec3& p2, const Vec3& p3,
                                            float t) {
    const float h = 1e-4f;
    const float t_lo = std::max(0.0f, t - h);
    const float t_hi = std::min(1.0f, t + h);
    const float dt = t_hi - t_lo;
    if (dt < 1e-8f) {
        return Vec3::zero();
    }
    return (evaluate_segment(p0, p1, p2, p3, t_hi)
          - evaluate_segment(p0, p1, p2, p3, t_lo)) / dt;
}

Vec3 CatmullRomSpline::evaluate(float t) const {
    if (waypoints_.size() < 2) {
        return waypoints_.empty() ? Vec3::zero() : waypoints_[0];
    }

    const float n = static_cast<float>(segment_count());

    // Clamp t to valid range
    t = std::max(0.0f, std::min(t, n));

    // Determine which segment and local parameter
    size_t seg = static_cast<size_t>(t);
    if (seg >= segment_count()) {
        seg = segment_count() - 1;
    }
    const float local_t = t - static_cast<float>(seg);

    auto pts = get_segment_points(seg);
    return evaluate_segment(pts[0], pts[1], pts[2], pts[3], local_t);
}

Vec3 CatmullRomSpline::tangent(float t) const {
    if (waypoints_.size() < 2) {
        return Vec3::zero();
    }

    const float n = static_cast<float>(segment_count());
    t = std::max(0.0f, std::min(t, n));

    size_t seg = static_cast<size_t>(t);
    if (seg >= segment_count()) {
        seg = segment_count() - 1;
    }
    const float local_t = t - static_cast<float>(seg);

    auto pts = get_segment_points(seg);
    Vec3 d = derivative_segment(pts[0], pts[1], pts[2], pts[3], local_t);
    float len = d.length();
    if (len > 1e-8f) {
        return d / len;
    }
    // Fallback: chord direction
    return (pts[2] - pts[1]).normalized();
}

float CatmullRomSpline::total_arc_length(int samples_per_segment) const {
    if (waypoints_.size() < 2) return 0.0f;

    float total = 0.0f;
    for (size_t seg = 0; seg < segment_count(); ++seg) {
        auto pts = get_segment_points(seg);
        Vec3 prev = evaluate_segment(pts[0], pts[1], pts[2], pts[3], 0.0f);
        for (int i = 1; i <= samples_per_segment; ++i) {
            const float t = static_cast<float>(i) / static_cast<float>(samples_per_segment);
            Vec3 curr = evaluate_segment(pts[0], pts[1], pts[2], pts[3], t);
            total += (curr - prev).length();
            prev = curr;
        }
    }
    return total;
}

std::vector<Vec3> CatmullRomSpline::to_polyline(float segment_length) const {
    if (waypoints_.size() < 2) {
        if (waypoints_.size() == 1) return {waypoints_[0]};
        return {};
    }

    const float total = total_arc_length(20);
    if (total < 1e-8f) {
        return {waypoints_.front(), waypoints_.back()};
    }

    const int total_samples = std::max(2, static_cast<int>(std::ceil(total / segment_length)));
    const int samples_per_seg = std::max(1,
        static_cast<int>(std::ceil(static_cast<float>(total_samples) / static_cast<float>(segment_count()))));

    return to_polyline_fixed(samples_per_seg);
}

std::vector<Vec3> CatmullRomSpline::to_polyline_fixed(int samples_per_segment) const {
    if (waypoints_.size() < 2) {
        if (waypoints_.size() == 1) return {waypoints_[0]};
        return {};
    }

    std::vector<Vec3> result;
    result.reserve(segment_count() * samples_per_segment + 1);

    for (size_t seg = 0; seg < segment_count(); ++seg) {
        auto pts = get_segment_points(seg);
        const int start_i = (seg == 0) ? 0 : 1;  // avoid duplicating junction points
        for (int i = start_i; i <= samples_per_segment; ++i) {
            const float t = static_cast<float>(i) / static_cast<float>(samples_per_segment);
            result.push_back(evaluate_segment(pts[0], pts[1], pts[2], pts[3], t));
        }
    }

    return result;
}

}  // namespace yarnpath
