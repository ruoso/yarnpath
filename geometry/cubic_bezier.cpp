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

BezierSpline BezierSpline::from_yarn_points(const std::vector<YarnPathPoint>& points) {
    BezierSpline spline;

    if (points.size() < 2) {
        return spline;
    }

    // For each pair of consecutive points, create a Bezier segment
    // The tangent at each point is computed from neighboring points
    // Tension controls how much the tangent affects the curve shape

    for (size_t i = 0; i + 1 < points.size(); ++i) {
        const auto& p0 = points[i];
        const auto& p1 = points[i + 1];

        // Compute tangent at p0 (looking at neighbors)
        Vec3 tangent0;
        if (i == 0) {
            // First point: tangent points toward next point
            tangent0 = p1.position - p0.position;
        } else {
            // Interior point: tangent from previous to next (Catmull-Rom style)
            tangent0 = points[i + 1].position - points[i - 1].position;
        }

        // Compute tangent at p1 (looking at neighbors)
        Vec3 tangent1;
        if (i + 2 >= points.size()) {
            // Last point: tangent points from previous point
            tangent1 = p1.position - p0.position;
        } else {
            // Interior point: tangent from previous to next
            tangent1 = points[i + 2].position - points[i].position;
        }

        // Scale tangents based on tension and distance
        // High tension = shorter tangent = tighter curve
        // Low tension = longer tangent = smoother curve
        float dist = p0.position.distance_to(p1.position);

        // Tension affects tangent length: high tension = short tangent
        // Standard Catmull-Rom uses 0.5 factor, we modify based on tension
        // tension 0 -> factor ~0.5 (smooth)
        // tension 1 -> factor ~0.1 (tight)
        float factor0 = 0.5f * (1.0f - p0.tension * 0.8f);
        float factor1 = 0.5f * (1.0f - p1.tension * 0.8f);

        // Clamp tangent magnitude to be reasonable relative to segment length
        float mag0 = tangent0.length();
        float mag1 = tangent1.length();

        Vec3 scaled_tangent0 = (mag0 > 0.0001f)
            ? tangent0 * (factor0 * dist / mag0)
            : Vec3(dist * factor0, 0.0f, 0.0f);

        Vec3 scaled_tangent1 = (mag1 > 0.0001f)
            ? tangent1 * (factor1 * dist / mag1)
            : Vec3(dist * factor1, 0.0f, 0.0f);

        // Create Bezier from Hermite data
        CubicBezier segment = CubicBezier::from_hermite(
            p0.position, scaled_tangent0,
            p1.position, scaled_tangent1
        );

        spline.add_segment(segment);
    }

    return spline;
}

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

void BezierSpline::merge_short_segments(float min_length) {
    if (segments_.size() < 2) {
        return;
    }

    std::vector<CubicBezier> new_segments;

    size_t i = 0;
    while (i < segments_.size()) {
        float arc_len = segments_[i].arc_length();

        if (arc_len >= min_length || i + 1 >= segments_.size()) {
            // Keep this segment as-is
            new_segments.push_back(segments_[i]);
            i++;
        } else {
            // Merge with next segment
            // Create a new Bezier curve from start of current to end of next
            // preserving the tangent directions at the outer endpoints
            const auto& curr = segments_[i];
            const auto& next = segments_[i + 1];

            Vec3 start_pos = curr.start();
            Vec3 end_pos = next.end();

            // Get tangent directions at endpoints
            Vec3 start_tangent = curr.derivative(0.0f);
            Vec3 end_tangent = next.derivative(1.0f);

            // Scale tangents based on the combined segment length
            float combined_arc = curr.arc_length() + next.arc_length();
            float start_tangent_len = start_tangent.length();
            float end_tangent_len = end_tangent.length();

            if (start_tangent_len > 1e-6f) {
                start_tangent = start_tangent * (combined_arc * 0.33f / start_tangent_len);
            }
            if (end_tangent_len > 1e-6f) {
                end_tangent = end_tangent * (combined_arc * 0.33f / end_tangent_len);
            }

            CubicBezier merged = CubicBezier::from_hermite(start_pos, start_tangent,
                                                           end_pos, end_tangent);
            new_segments.push_back(merged);
            i += 2;
        }
    }

    segments_ = std::move(new_segments);
}

BezierSpline BezierSpline::to_adaptive_spline(float max_k, int min_samples, int max_samples) const {
    if (segments_.empty()) {
        return BezierSpline();
    }

    // Sample the entire spline at fine intervals
    std::vector<Vec3> positions;
    std::vector<Vec3> tangents;
    std::vector<float> curvatures;

    int total_samples = static_cast<int>(segments_.size()) * max_samples;
    float dt = static_cast<float>(segments_.size()) / static_cast<float>(total_samples);

    for (int i = 0; i <= total_samples; ++i) {
        float t = static_cast<float>(i) * dt;
        if (t > static_cast<float>(segments_.size())) {
            t = static_cast<float>(segments_.size());
        }

        positions.push_back(evaluate(t));
        tangents.push_back(tangent(t));

        // Compute curvature at this point
        size_t seg_idx = static_cast<size_t>(t);
        if (seg_idx >= segments_.size()) {
            seg_idx = segments_.size() - 1;
        }
        float local_t = t - static_cast<float>(seg_idx);
        curvatures.push_back(segments_[seg_idx].curvature(local_t));
    }

    // Select points based on curvature
    // High curvature regions: keep more points
    // Low curvature regions: keep fewer points
    std::vector<size_t> selected_indices;
    selected_indices.push_back(0);

    float accumulated_score = 0.0f;
    float threshold = 1.0f / static_cast<float>(min_samples);

    for (size_t i = 1; i < positions.size() - 1; ++i) {
        // Score based on curvature relative to max_k
        float curvature_ratio = std::min(1.0f, curvatures[i] / max_k);
        accumulated_score += (1.0f + curvature_ratio * 2.0f) * dt;

        if (accumulated_score >= threshold) {
            selected_indices.push_back(i);
            accumulated_score = 0.0f;
        }
    }

    selected_indices.push_back(positions.size() - 1);

    // Ensure minimum number of points
    if (selected_indices.size() < static_cast<size_t>(min_samples + 1)) {
        // Uniformly add more points
        size_t target_count = static_cast<size_t>(min_samples + 1);
        std::vector<size_t> uniform_indices;
        for (size_t i = 0; i < target_count; ++i) {
            size_t idx = (i * (positions.size() - 1)) / (target_count - 1);
            uniform_indices.push_back(idx);
        }
        selected_indices = std::move(uniform_indices);
    }

    // Build new spline from selected points using Hermite interpolation
    std::vector<YarnPathPoint> selected_points;
    for (size_t idx : selected_indices) {
        YarnPathPoint pt;
        pt.position = positions[idx];
        // Use inverse curvature as tension (high curvature = high tension)
        float curvature_ratio = std::min(1.0f, curvatures[idx] / max_k);
        pt.tension = curvature_ratio;
        selected_points.push_back(pt);
    }

    return BezierSpline::from_yarn_points(selected_points);
}

void BezierSpline::cleanup(float min_segment_length) {
    merge_short_segments(min_segment_length);
    enforce_c1_continuity();
}

}  // namespace yarnpath
