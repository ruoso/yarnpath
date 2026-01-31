#include "cubic_bezier.hpp"
#include <cmath>
#include <algorithm>
#include <stdexcept>

#include <math/vec3.hpp>

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
            n = d1.cross(Vec3::unit_x());
        } else {
            n = d1.cross(Vec3::unit_y());
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
        return Vec3::zero();
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
        return Vec3::unit_x();
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

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <vector>

// --- helpers ---------------------------------------------------------------

static bool solve_3x3_columns(
    const Vec3& A0, const Vec3& A1, const Vec3& A2, // columns
    const Vec3& R,
    float& x0, float& x1, float& x2)
{
    // Solve [A0 A1 A2] * [x0 x1 x2]^T = R
    const float det = A0.dot(A1.cross(A2));
    if (std::abs(det) < 1e-10f) return false;

    x0 = R.dot(A1.cross(A2)) / det;
    x1 = A0.dot(R.cross(A2)) / det;
    x2 = A0.dot(A1.cross(R)) / det;
    return true;
}

// Robust-ish root finding for f(t) = |B(t)-C|^2 - r^2 over [0,1].
static std::vector<float> intersect_sphere_ts(
    const CubicBezier& curve,
    const Vec3& center,
    float radius,
    int samples = 128,
    float eps_f = 1e-6f)
{
    auto f = [&](float t) -> float {
        Vec3 p = curve.evaluate(t);
        Vec3 d = p - center;
        return d.dot(d) - radius * radius;
    };

    std::vector<float> roots;
    roots.reserve(4);

    float t0 = 0.0f;
    float f0 = f(t0);

    for (int i = 1; i <= samples; ++i) {
        float t1 = float(i) / float(samples);
        float f1 = f(t1);

        bool sign_change = (f0 <= 0.0f && f1 >= 0.0f) || (f0 >= 0.0f && f1 <= 0.0f);
        bool near_zero   = (std::abs(f1) < eps_f);

        if (sign_change || near_zero) {
            float a = t0, b = t1;
            float fa = f0, fb = f1;

            // If we're "near zero" but didn't bracket, widen slightly.
            if (!sign_change) {
                float dt = 1.0f / float(samples);
                a = std::max(0.0f, t1 - dt);
                b = std::min(1.0f, t1 + dt);
                fa = f(a);
                fb = f(b);
            }

            // Bisection refine
            for (int it = 0; it < 50; ++it) {
                float m  = 0.5f * (a + b);
                float fm = f(m);

                if (std::abs(fm) < eps_f) { a = b = m; break; }

                bool left = (fa <= 0.0f && fm >= 0.0f) || (fa >= 0.0f && fm <= 0.0f);
                if (left) { b = m; fb = fm; }
                else      { a = m; fa = fm; }
            }

            float root = 0.5f * (a + b);

            // dedupe
            bool dup = false;
            for (float r : roots) {
                if (std::abs(r - root) < 1e-4f) { dup = true; break; }
            }
            if (!dup) roots.push_back(root);
        }

        t0 = t1;
        f0 = f1;
    }

    std::sort(roots.begin(), roots.end());
    return roots;
}

// Pick a stable perpendicular unit vector to v (used in antipodal/degenerate cases).
static Vec3 any_perp_unit(const Vec3& v)
{
    Vec3 ref = (std::abs(v.x) < 0.9f) ? Vec3::unit_x() : Vec3::unit_y();
    Vec3 p = v.cross(ref);
    float len = p.length();
    if (len < 1e-9f) {
        ref = Vec3::unit_z();
        p = v.cross(ref);
        len = p.length();
        if (len < 1e-9f) return Vec3(1,0,0);
    }
    return p / len;
}

// Spherical linear interpolation between unit vectors u and v.
// Returns a unit vector on the shortest arc.
static Vec3 slerp_unit(const Vec3& u, const Vec3& v, float t)
{
    float dot = std::clamp(u.dot(v), -1.0f, 1.0f);
    float theta = std::acos(dot);
    if (theta < 1e-6f) return u;

    float s = std::sin(theta);
    float w0 = std::sin((1.0f - t) * theta) / s;
    float w1 = std::sin(t * theta) / s;

    Vec3 r = u * w0 + v * w1;
    float rl = r.length();
    return (rl > 1e-9f) ? (r / rl) : u;
}

// Returns true and sets M if we find a useful arc-midpoint on the clearance sphere.
static bool find_clearance_point_M(
    const CubicBezier& base,
    const Vec3& sphere_center,
    float sphere_radius,
    Vec3& out_M)
{
    // Intersections (parameter values)
    std::vector<float> ts = intersect_sphere_ts(base, sphere_center, sphere_radius);

    if (ts.size() < 2) {
        return false;
    }

    // Choose the best pair if we have more than 2 intersections.
    // We pick the pair whose midpoint t is closest to where the sphere center projects
    // onto the chord (base.start -> base.end). This tends to choose the “relevant” crossing.
    Vec3 P0 = base.start();
    Vec3 P3 = base.end();
    Vec3 chord = P3 - P0;
    float dist = chord.length();
    Vec3 path_dir = (dist > 1e-9f) ? (chord / dist) : Vec3(0,1,0);

    float proj = (sphere_center - P0).dot(path_dir);
    proj = std::clamp(proj, 0.0f, dist);
    float t_proj = (dist > 1e-9f) ? (proj / dist) : 0.5f;

    float bestScore = std::numeric_limits<float>::infinity();
    float t1 = ts[0], t2 = ts[1];

    for (size_t i = 0; i + 1 < ts.size(); ++i) {
        float a = ts[i];
        float b = ts[i + 1];
        float mid = 0.5f * (a + b);

        float score = std::abs(mid - t_proj);
        if (score < bestScore) {
            bestScore = score;
            t1 = a; t2 = b;
        }
    }

    // Evaluate intersection points
    Vec3 Pa = base.evaluate(t1);
    Vec3 Pb = base.evaluate(t2);

    // Convert to unit radial directions
    Vec3 u = Pa - sphere_center;
    Vec3 v = Pb - sphere_center;
    float ul = u.length();
    float vl = v.length();

    if (ul < 1e-6f || vl < 1e-6f) {
        return false;
    }

    u /= ul;
    v /= vl;

    // Midpoint along the shortest great-circle arc:
    // slerp(u,v,0.5) is stable except near antipodal; handle that case.
    float dot = std::clamp(u.dot(v), -1.0f, 1.0f);

    Vec3 w;
    if (dot < -0.9999f) {
        // Nearly antipodal: infinitely many shortest arcs.
        // Choose a midpoint direction using a stable plane based on path_dir.
        // Make path_dir tangent at u by projecting out radial component.
        Vec3 tangent = path_dir - u * path_dir.dot(u);
        float tl = tangent.length();
        if (tl < 1e-6f) {
            tangent = any_perp_unit(u);
        } else {
            tangent /= tl;
        }
        // Rotate u by 90 degrees in the (u,tangent) plane: midpoint direction.
        // (This is one of many valid choices; it’s stable and “forwardish”.)
        w = (u + tangent).normalized();
        if (w.length() < 1e-6f) w = tangent;
    } else {
        w = slerp_unit(u, v, 0.5f);
    }

    // Output point on the sphere surface
    out_M = sphere_center + w * sphere_radius;
    return true;
}

// Construct a cubic that hits a point M at parameter t_hit, while matching endpoint directions.
// We use 3 scalar DOFs (a,b,c):
//   P1 = P0 + a*d0 + c*n
//   P2 = P3 - b*d1 + c*n
static bool build_curve_through_point_at_t(
    const Vec3& P0, const Vec3& d0_unit,
    const Vec3& P3, const Vec3& d1_unit,
    const Vec3& n_unit,
    const Vec3& M,
    float t_hit,
    CubicBezier& out_curve,
    float clamp_handle = -1.0f) // if >0, clamps a,b,c to this max
{
    float t = std::clamp(t_hit, 1e-4f, 1.0f - 1e-4f);
    float u = 1.0f - t;

    float alpha = 3.0f * u*u * t;
    float beta  = 3.0f * u * t*t;
    float gamma = alpha + beta;

    // Baseline point C(t) for collapsed handles (P1=P0, P2=P3)
    Vec3 C = (u*u*u + alpha) * P0 + (beta + t*t*t) * P3;
    Vec3 R = M - C;

    // R = (alpha*a) d0 + (-beta*b) d1 + (gamma*c) n
    Vec3 A0 = d0_unit * alpha;
    Vec3 A1 = d1_unit * (-beta);
    Vec3 A2 = n_unit  * gamma;

    float a, b, c;
    if (!solve_3x3_columns(A0, A1, A2, R, a, b, c)) return false;

    // Basic sanity
    a = std::max(0.0f, a);
    b = std::max(0.0f, b);

    // We generally want the bulge to go in the chosen +n direction.
    // If c comes out negative, flip n and c.
    if (c < 0.0f) c = -c; // caller should pass an n that makes sense; keep it non-negative.

    if (clamp_handle > 0.0f) {
        a = std::min(a, clamp_handle);
        b = std::min(b, clamp_handle);
        c = std::min(c, clamp_handle);
    }

    Vec3 P1 = P0 + d0_unit * a + n_unit * c;
    Vec3 P2 = P3 - d1_unit * b + n_unit * c;

    out_curve = CubicBezier(P0, P1, P2, P3);
    return true;
}

static std::pair<Vec3, Vec3>
fair_hermite_tangents(
    const Vec3& P0,
    const Vec3& P1,
    const Vec3& d0_unit,
    const Vec3& d1_unit)
{
    Vec3 D = P1 - P0;
    float dist = D.length();
    if (dist < 1e-6f) {
        return { Vec3(0,0,0), Vec3(0,0,0) };
    }

    // Dot products describing geometric alignment
    float c  = std::clamp(d0_unit.dot(d1_unit), -1.0f, 1.0f);
    float p0 = D.dot(d0_unit);
    float p1 = D.dot(d1_unit);

    // Closed-form solution minimizing ∫|B''(t)|² dt
    float denom = 4.0f - c * c; // ∈ [3,4]
    float m0 = (6.0f * p0 - 3.0f * c * p1) / denom;
    float m1 = (6.0f * p1 - 3.0f * c * p0) / denom;

    return {
        d0_unit * m0,
        d1_unit * m1
    };
}

static Vec3 sphere_tangent_toward_target(
    const Vec3& sphere_center,
    const Vec3& M_on_or_near_sphere,
    const Vec3& target_point,
    const Vec3& fallback_dir_unit)
{
    Vec3 u = (M_on_or_near_sphere - sphere_center);
    float ul = u.length();
    if (ul < 1e-6f) return fallback_dir_unit;
    u /= ul;

    Vec3 toward = target_point - M_on_or_near_sphere;

    // Project onto tangent plane at M: remove radial component
    Vec3 t = toward - u * toward.dot(u);
    float tl = t.length();
    if (tl < 1e-6f) {
        // If target is almost radial from M, choose any tangent direction
        // consistent with fallback.
        Vec3 f = fallback_dir_unit - u * fallback_dir_unit.dot(u);
        float fl = f.length();
        if (fl < 1e-6f) {
            // truly degenerate: pick any perpendicular to u
            Vec3 ref = (std::abs(u.x) < 0.9f) ? Vec3::unit_x() : Vec3::unit_y();
            f = u.cross(ref);
            fl = f.length();
            if (fl < 1e-6f) return Vec3(1,0,0);
        }
        return f / fl;
    }

    return t / tl;
}

// --- main function ----------------------------------------------------------

std::vector<CubicBezier> create_continuation_segment_with_clearance(
    const BezierSpline& spline,
    const Vec3& target_point,
    const Vec3& target_direction,
    const Vec3& clearance_point,
    float clearance_radius,
    float max_curvature)
{
    std::vector<CubicBezier> out;

    // 1) Start point + incoming direction
    Vec3 P0;
    Vec3 d0;
    if (spline.empty()) {
        // Degenerate: no previous segment; choose something consistent
        P0 = target_point;
        d0 = (-target_direction).normalized();
    } else {
        const auto& last = spline.segments().back();
        P0 = last.end();
        d0 = last.tangent(1.0f).normalized();
    }

    Vec3 P3 = target_point;
    Vec3 d1 = target_direction.normalized();

    Vec3 chord = P3 - P0;
    float dist = chord.length();
    if (dist < 1e-6f) {
        return { CubicBezier(P0, P0, P3, P3) };
    }
    Vec3 path_dir = chord / dist;

    // 2) Baseline curve (no clearance bulge): Hermite with a reasonable magnitude
    // You can keep your existing heuristic for magnitude; this is a simple one:
    auto [T0, T1] = fair_hermite_tangents(P0, P3, d0, d1);
    CubicBezier base = CubicBezier::from_hermite(P0, T0, P3, T1);

    // 3) Intersect baseline with clearance sphere
    Vec3 M;
    bool have_M = find_clearance_point_M(base, clearance_point, clearance_radius, M);

    if (!have_M) {
        // No clearance issue: just return the single baseline segment
        out.push_back(base);
        return out;
    }

    Vec3 tM = sphere_tangent_toward_target(clearance_point, M, P3, d1);

    // ---- 5) Build segment A: (P0,d0) -> (M,tM) ----
    auto [T0A, TMA] = fair_hermite_tangents(P0, M, d0, tM);
    CubicBezier segA = CubicBezier::from_hermite(P0, T0A, M, TMA);

    // ---- 6) Build segment B: (M,tM) -> (P3,d1) ----
    auto [TMB, T1B] = fair_hermite_tangents(M, P3, tM, d1);
    CubicBezier segB = CubicBezier::from_hermite(M, TMB, P3, T1B);

    out.push_back(segA);
    out.push_back(segB);
 
    return out;
}

}  // namespace yarnpath
