#include "cubic_bezier.hpp"
#include <cmath>
#include <algorithm>
#include <stdexcept>

namespace yarnpath {

CubicBezier::CubicBezier(const Vec3& p0, const Vec3& p1, const Vec3& p2, const Vec3& p3)
    : control_points{p0, p1, p2, p3} {}

Vec3 CubicBezier::evaluate(float t) const {
    // De Casteljau's algorithm
    float u = 1.0f - t;
    float tt = t * t;
    float uu = u * u;
    float ttt = tt * t;
    float uuu = uu * u;

    return control_points[0] * uuu +
           control_points[1] * (3.0f * uu * t) +
           control_points[2] * (3.0f * u * tt) +
           control_points[3] * ttt;
}

Vec3 CubicBezier::derivative(float t) const {
    // First derivative of cubic Bezier
    float u = 1.0f - t;
    float uu = u * u;
    float tt = t * t;

    Vec3 d0 = control_points[1] - control_points[0];
    Vec3 d1 = control_points[2] - control_points[1];
    Vec3 d2 = control_points[3] - control_points[2];

    return d0 * (3.0f * uu) + d1 * (6.0f * u * t) + d2 * (3.0f * tt);
}

Vec3 CubicBezier::second_derivative(float t) const {
    // Second derivative of cubic Bezier
    float u = 1.0f - t;

    Vec3 d0 = control_points[1] - control_points[0];
    Vec3 d1 = control_points[2] - control_points[1];
    Vec3 d2 = control_points[3] - control_points[2];

    Vec3 dd0 = d1 - d0;
    Vec3 dd1 = d2 - d1;

    return dd0 * (6.0f * u) + dd1 * (6.0f * t);
}

float CubicBezier::curvature(float t) const {
    Vec3 d1 = derivative(t);
    Vec3 d2 = second_derivative(t);

    Vec3 cross = d1.cross(d2);
    float d1_len = d1.length();

    if (d1_len < 1e-8f) {
        return 0.0f;
    }

    return cross.length() / (d1_len * d1_len * d1_len);
}

float CubicBezier::max_curvature(int samples) const {
    float max_k = 0.0f;
    for (int i = 0; i <= samples; ++i) {
        float t = static_cast<float>(i) / static_cast<float>(samples);
        max_k = std::max(max_k, curvature(t));
    }
    return max_k;
}

float CubicBezier::arc_length(int samples) const {
    float length = 0.0f;
    Vec3 prev = control_points[0];

    for (int i = 1; i <= samples; ++i) {
        float t = static_cast<float>(i) / static_cast<float>(samples);
        Vec3 curr = evaluate(t);
        length += (curr - prev).length();
        prev = curr;
    }
    return length;
}

Vec3 CubicBezier::tangent(float t) const {
    return derivative(t).normalized();
}

Vec3 CubicBezier::normal(float t) const {
    Vec3 d1 = derivative(t);
    Vec3 d2 = second_derivative(t);

    // Normal is in the plane of the curve, perpendicular to tangent
    // Using the formula: N = (d1 x d2) x d1, normalized
    Vec3 cross = d1.cross(d2);
    Vec3 n = cross.cross(d1);

    float len = n.length();
    if (len < 1e-8f) {
        // Degenerate case: use arbitrary perpendicular
        if (std::abs(d1.x) < 0.9f) {
            n = d1.cross(vec3::unit_x());
        } else {
            n = d1.cross(vec3::unit_y());
        }
    }
    return n.normalized();
}

std::pair<CubicBezier, CubicBezier> CubicBezier::split(float t) const {
    // De Casteljau subdivision
    Vec3 p01 = lerp(control_points[0], control_points[1], t);
    Vec3 p12 = lerp(control_points[1], control_points[2], t);
    Vec3 p23 = lerp(control_points[2], control_points[3], t);

    Vec3 p012 = lerp(p01, p12, t);
    Vec3 p123 = lerp(p12, p23, t);

    Vec3 p0123 = lerp(p012, p123, t);

    CubicBezier left(control_points[0], p01, p012, p0123);
    CubicBezier right(p0123, p123, p23, control_points[3]);

    return {left, right};
}

std::vector<CubicBezier> CubicBezier::subdivide_for_curvature(float max_k, int max_depth) const {
    if (max_depth <= 0 || max_curvature() <= max_k) {
        return {*this};
    }

    auto [left, right] = split(0.5f);
    auto left_result = left.subdivide_for_curvature(max_k, max_depth - 1);
    auto right_result = right.subdivide_for_curvature(max_k, max_depth - 1);

    left_result.insert(left_result.end(), right_result.begin(), right_result.end());
    return left_result;
}

CubicBezier CubicBezier::from_hermite(const Vec3& p0, const Vec3& tangent0,
                                       const Vec3& p1, const Vec3& tangent1) {
    // Convert Hermite to Bezier
    // P1 = P0 + T0/3
    // P2 = P3 - T1/3
    Vec3 c1 = p0 + tangent0 / 3.0f;
    Vec3 c2 = p1 - tangent1 / 3.0f;
    return CubicBezier(p0, c1, c2, p1);
}

// BezierSpline implementation

BezierSpline::BezierSpline(std::vector<CubicBezier> segments)
    : segments_(std::move(segments)) {}

void BezierSpline::add_segment(const CubicBezier& segment) {
    segments_.push_back(segment);
}

Vec3 BezierSpline::evaluate(float t) const {
    if (segments_.empty()) {
        return vec3::zero();
    }

    // Clamp t to valid range
    t = std::clamp(t, 0.0f, static_cast<float>(segments_.size()));

    size_t segment_index = static_cast<size_t>(t);
    if (segment_index >= segments_.size()) {
        segment_index = segments_.size() - 1;
    }

    float local_t = t - static_cast<float>(segment_index);
    return segments_[segment_index].evaluate(local_t);
}

Vec3 BezierSpline::tangent(float t) const {
    if (segments_.empty()) {
        return vec3::unit_x();
    }

    t = std::clamp(t, 0.0f, static_cast<float>(segments_.size()));

    size_t segment_index = static_cast<size_t>(t);
    if (segment_index >= segments_.size()) {
        segment_index = segments_.size() - 1;
    }

    float local_t = t - static_cast<float>(segment_index);
    return segments_[segment_index].tangent(local_t);
}

float BezierSpline::total_arc_length(int samples_per_segment) const {
    float total = 0.0f;
    for (const auto& seg : segments_) {
        total += seg.arc_length(samples_per_segment);
    }
    return total;
}

float BezierSpline::max_curvature(int samples_per_segment) const {
    float max_k = 0.0f;
    for (const auto& seg : segments_) {
        max_k = std::max(max_k, seg.max_curvature(samples_per_segment));
    }
    return max_k;
}

void BezierSpline::enforce_c1_continuity() {
    if (segments_.size() < 2) {
        return;
    }

    for (size_t i = 1; i < segments_.size(); ++i) {
        // Ensure position continuity
        segments_[i].start() = segments_[i - 1].end();

        // Ensure tangent continuity by averaging
        Vec3 tangent_prev = segments_[i - 1].end() - segments_[i - 1].control2();
        Vec3 tangent_next = segments_[i].control1() - segments_[i].start();

        Vec3 avg_tangent = (tangent_prev + tangent_next) / 2.0f;

        segments_[i - 1].control2() = segments_[i - 1].end() - avg_tangent;
        segments_[i].control1() = segments_[i].start() + avg_tangent;
    }
}

void BezierSpline::clamp_curvature(float max_k) {
    std::vector<CubicBezier> new_segments;
    for (const auto& seg : segments_) {
        auto subdivided = seg.subdivide_for_curvature(max_k);
        new_segments.insert(new_segments.end(), subdivided.begin(), subdivided.end());
    }
    segments_ = std::move(new_segments);
}

std::vector<Vec3> BezierSpline::to_polyline(float segment_length) const {
    if (segments_.empty()) {
        return {};
    }

    std::vector<Vec3> points;
    points.push_back(segments_[0].start());

    for (const auto& seg : segments_) {
        float arc_len = seg.arc_length();
        int num_samples = std::max(1, static_cast<int>(std::ceil(arc_len / segment_length)));

        for (int i = 1; i <= num_samples; ++i) {
            float t = static_cast<float>(i) / static_cast<float>(num_samples);
            points.push_back(seg.evaluate(t));
        }
    }

    return points;
}

std::vector<Vec3> BezierSpline::to_polyline_fixed(int samples_per_segment) const {
    if (segments_.empty()) {
        return {};
    }

    std::vector<Vec3> points;
    points.push_back(segments_[0].start());

    for (const auto& seg : segments_) {
        for (int i = 1; i <= samples_per_segment; ++i) {
            float t = static_cast<float>(i) / static_cast<float>(samples_per_segment);
            points.push_back(seg.evaluate(t));
        }
    }

    return points;
}

}  // namespace yarnpath
