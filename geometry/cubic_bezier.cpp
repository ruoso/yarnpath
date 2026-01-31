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


void BezierSpline::cleanup(float min_segment_length) {
    merge_short_segments(min_segment_length);
    enforce_c1_continuity();
}

CubicBezier create_continuation_segment(
    const BezierSpline& spline,
    const Vec3& target_point,
    const Vec3& target_direction,
    float max_curvature) {
    
    // Get the starting point and incoming direction from the spline
    Vec3 start_point;
    Vec3 incoming_dir;
    
    if (spline.empty()) {
        // No spline segments - use target direction reversed as incoming
        start_point = target_point;  // Degenerate case
        incoming_dir = target_direction.normalized() * -1.0f;
    } else {
        // Get the last segment's end point and exit tangent
        const auto& last_segment = spline.segments().back();
        start_point = last_segment.end();
        incoming_dir = last_segment.tangent(1.0f);  // Tangent at t=1 (end)
    }
    
    // Normalize target direction
    Vec3 target_dir = target_direction.normalized();
    
    // Calculate the distance between points
    Vec3 to_target = target_point - start_point;
    float distance = to_target.length();
    
    if (distance < 1e-6f) {
        // Points are coincident - return a degenerate segment
        return CubicBezier(start_point, start_point, target_point, target_point);
    }
    
    // Calculate tangent magnitudes based on curvature constraint.
    // For a Hermite curve, the curvature at endpoints depends on the tangent magnitude.
    // A larger tangent magnitude creates a gentler curve (lower curvature).
    // 
    // The minimum bend radius is 1/max_curvature. To ensure we don't exceed
    // max_curvature, we need tangent magnitudes that are proportional to
    // the distance and inversely related to the allowed curvature.
    //
    // For a smooth curve: tangent_magnitude ~= distance * factor
    // where factor depends on how much the direction changes.
    
    // Calculate how much the direction changes (dot product of normalized directions)
    float direction_alignment = incoming_dir.dot(target_dir);
    // direction_alignment: 1.0 = same direction, -1.0 = opposite, 0 = perpendicular
    
    // Minimum bend radius
    float min_bend_radius = (max_curvature > 1e-6f) ? (1.0f / max_curvature) : 1000.0f;
    
    // Base tangent magnitude - scales with distance
    // When directions are aligned, we need less tangent to make a smooth curve
    // When directions differ significantly, we need more tangent to avoid sharp bends
    float alignment_factor = 0.5f - 0.3f * direction_alignment;  // Range: [0.2, 0.8]
    
    // Ensure tangent is long enough to respect minimum bend radius
    // The tangent magnitude should be at least proportional to min_bend_radius
    // for curves that need to turn significantly
    float curvature_factor = std::max(0.3f, min_bend_radius / distance);
    curvature_factor = std::min(curvature_factor, 1.0f);  // Cap at 1.0
    
    float tangent_magnitude = distance * std::max(alignment_factor, curvature_factor * 0.5f);
    
    // Create the Hermite curve
    Vec3 start_tangent = incoming_dir * tangent_magnitude;
    Vec3 end_tangent = target_dir * tangent_magnitude;
    
    return CubicBezier::from_hermite(start_point, start_tangent, target_point, end_tangent);
}

CubicBezier create_continuation_segment_with_clearance(
    const BezierSpline& spline,
    const Vec3& target_point,
    const Vec3& target_direction,
    const Vec3& clearance_point,
    float clearance_radius,
    float max_curvature) {
    
    // Get the starting point and incoming direction from the spline
    Vec3 start_point;
    Vec3 incoming_dir;
    
    if (spline.empty()) {
        // No spline segments - use target direction reversed as incoming
        start_point = target_point;  // Degenerate case
        incoming_dir = target_direction.normalized() * -1.0f;
    } else {
        // Get the last segment's end point and exit tangent
        const auto& last_segment = spline.segments().back();
        start_point = last_segment.end();
        incoming_dir = last_segment.tangent(1.0f);  // Tangent at t=1 (end)
    }
    
    // Normalize target direction
    Vec3 target_dir = target_direction.normalized();
    
    // Calculate the distance between start and target
    Vec3 to_target = target_point - start_point;
    float distance = to_target.length();
    
    if (distance < 1e-6f) {
        // Points are coincident - return a degenerate segment
        return CubicBezier(start_point, start_point, target_point, target_point);
    }
    
    // Calculate the midpoint of the straight line path
    Vec3 midpoint = (start_point + target_point) * 0.5f;
    
    // Vector from clearance point to midpoint
    Vec3 clearance_to_mid = midpoint - clearance_point;
    float clearance_to_mid_dist = clearance_to_mid.length();
    
    // Determine which side of the clearance point we should curve around
    // Use the cross product of incoming and target directions to find the "outside"
    // If that's ambiguous, use the direction away from the clearance point
    Vec3 curve_normal;
    
    // Calculate the plane normal from the path (incoming x outgoing gives perpendicular)
    Vec3 path_cross = incoming_dir.cross(target_dir);
    float path_cross_len = path_cross.length();
    
    if (path_cross_len > 1e-6f) {
        // Incoming and target directions define a plane
        // The curve will bulge perpendicular to this, away from clearance point
        Vec3 plane_normal = path_cross.normalized();
        
        // Project clearance_to_mid onto the plane perpendicular to the path
        Vec3 avg_dir = (incoming_dir + target_dir).normalized();
        Vec3 in_plane = clearance_to_mid - avg_dir * clearance_to_mid.dot(avg_dir);
        float in_plane_len = in_plane.length();
        
        if (in_plane_len > 1e-6f) {
            curve_normal = in_plane.normalized();
        } else {
            // Clearance point is on the path line, use plane normal
            curve_normal = plane_normal;
        }
    } else {
        // Incoming and target are parallel (or anti-parallel)
        // Need to find a perpendicular direction away from clearance
        if (clearance_to_mid_dist > 1e-6f) {
            // Project out the component along the path
            Vec3 path_dir = to_target.normalized();
            Vec3 perp = clearance_to_mid - path_dir * clearance_to_mid.dot(path_dir);
            float perp_len = perp.length();
            if (perp_len > 1e-6f) {
                curve_normal = perp.normalized();
            } else {
                // Clearance is exactly on the line - pick arbitrary perpendicular
                if (std::abs(path_dir.x) < 0.9f) {
                    curve_normal = path_dir.cross(vec3::unit_x()).normalized();
                } else {
                    curve_normal = path_dir.cross(vec3::unit_y()).normalized();
                }
            }
        } else {
            // Midpoint is at clearance point - pick arbitrary perpendicular
            Vec3 path_dir = to_target.normalized();
            if (std::abs(path_dir.x) < 0.9f) {
                curve_normal = path_dir.cross(vec3::unit_x()).normalized();
            } else {
                curve_normal = path_dir.cross(vec3::unit_y()).normalized();
            }
        }
    }
    
    // Calculate how much we need to bulge outward to clear the obstacle
    // Find the closest approach of a straight line to the clearance point
    Vec3 path_dir = to_target.normalized();
    Vec3 start_to_clearance = clearance_point - start_point;
    float projection = start_to_clearance.dot(path_dir);
    projection = std::clamp(projection, 0.0f, distance);  // Clamp to segment
    Vec3 closest_on_line = start_point + path_dir * projection;
    float straight_line_clearance = (clearance_point - closest_on_line).length();
    
    // How much additional bulge do we need?
    float needed_bulge = 0.0f;
    if (straight_line_clearance < clearance_radius) {
        needed_bulge = clearance_radius - straight_line_clearance;
        // Add some extra margin for the curve (Bezier doesn't go through control points)
        needed_bulge *= 1.5f;
    }
    
    // Minimum bend radius
    float min_bend_radius = (max_curvature > 1e-6f) ? (1.0f / max_curvature) : 1000.0f;
    
    // Calculate base tangent magnitude
    float direction_alignment = incoming_dir.dot(target_dir);
    float alignment_factor = 0.5f - 0.3f * direction_alignment;
    float curvature_factor = std::max(0.3f, min_bend_radius / distance);
    curvature_factor = std::min(curvature_factor, 1.0f);
    float tangent_magnitude = distance * std::max(alignment_factor, curvature_factor * 0.5f);
    
    // If we need to bulge, increase tangent magnitude and add perpendicular component
    Vec3 start_tangent = incoming_dir * tangent_magnitude;
    Vec3 end_tangent = target_dir * tangent_magnitude;
    
    if (needed_bulge > 1e-6f) {
        // Add perpendicular component to control points to create bulge
        // The control points of a Bezier are offset from the straight line
        // We modify the tangents to push the curve away from the clearance zone
        
        // Scale bulge influence based on how much tangent needs to turn
        float bulge_factor = needed_bulge * 2.0f;  // Control point offset needs to be larger
        
        // Add perpendicular component to tangents
        // For start tangent: add component that pushes the curve outward at the start
        // For end tangent: add component that brings the curve back inward at the end
        start_tangent = start_tangent + curve_normal * bulge_factor;
        end_tangent = end_tangent + curve_normal * bulge_factor;
        
        // Also increase magnitude to maintain smooth curvature with the added turn
        float extra_magnitude = needed_bulge * 0.5f;
        start_tangent = start_tangent.normalized() * (tangent_magnitude + extra_magnitude);
        end_tangent = end_tangent.normalized() * (tangent_magnitude + extra_magnitude);
    }
    
    return CubicBezier::from_hermite(start_point, start_tangent, target_point, end_tangent);
}

}  // namespace yarnpath
