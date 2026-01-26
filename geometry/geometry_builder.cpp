#include "geometry_builder.hpp"
#include "logging.hpp"
#include <cmath>
#include <algorithm>

namespace yarnpath {

// Helper: create smooth bezier curve between two points with tangent constraints
BezierSpline make_connector(const YarnProperties& yarn,
                            const Vec3& from, const Vec3& from_dir,
                            const Vec3& to, const Vec3& to_dir) {
    (void)yarn;  // Not currently used but kept for API consistency
    BezierSpline spline;

    Vec3 chord = to - from;
    float dist = chord.length();

    if (dist < 0.0001f) {
        // Degenerate case: start and end are the same
        spline.add_segment(CubicBezier(from, from, to, to));
        return spline;
    }

    // Scale tangents based on distance - use ~1/3 of distance for smooth curves
    float tangent_scale = dist * 0.4f;

    Vec3 t0 = from_dir.normalized() * tangent_scale;
    Vec3 t1 = to_dir.normalized() * tangent_scale;

    auto bezier = CubicBezier::from_hermite(from, t0, to, -t1);
    spline.add_segment(bezier);

    return spline;
}

// Build the curve for a single segment
//
// For loop segments: creates a loop shape from base (curr_pos) up to apex and back
// For non-loop segments: simple connector curve from prev to curr
//
// The physics simulation positions nodes at:
// - Loop segments: the interlocking point (base of the loop)
// - The apex is determined by where child segments pass through
BezierSpline build_segment_curve(
    const Vec3& prev_pos,
    const Vec3& curr_pos,
    const Vec3& next_pos,
    bool forms_loop,
    const std::vector<Vec3>& child_positions,
    const YarnProperties& yarn) {

    BezierSpline spline;

    // Calculate incoming direction
    Vec3 in_vec = curr_pos - prev_pos;
    float in_dist = in_vec.length();
    Vec3 in_dir = (in_dist > 0.0001f) ? in_vec.normalized() : Vec3(1.0f, 0.0f, 0.0f);

    // Calculate outgoing direction
    Vec3 out_vec = next_pos - curr_pos;
    float out_dist = out_vec.length();
    Vec3 out_dir = (out_dist > 0.0001f) ? out_vec.normalized() : in_dir;

    if (forms_loop) {
        // Loop segment: create a loop shape
        // Base = curr_pos (interlocking point)
        // Apex = average of child positions, or default height if no children

        Vec3 apex;
        float loop_height;

        if (!child_positions.empty()) {
            // Apex is where children pass through
            apex = vec3::zero();
            for (const auto& cp : child_positions) {
                apex += cp;
            }
            apex = apex * (1.0f / static_cast<float>(child_positions.size()));
            loop_height = (apex - curr_pos).length();
        } else {
            // No children yet (live loop) - use default height
            loop_height = yarn.radius * 6.0f;
            // Default apex direction is "up" (positive Y)
            apex = curr_pos + Vec3(0.0f, loop_height, 0.0f);
        }

        // Ensure minimum loop height
        if (loop_height < yarn.radius * 2.0f) {
            loop_height = yarn.radius * 2.0f;
            Vec3 up_dir = (apex - curr_pos);
            if (up_dir.length() > 0.0001f) {
                up_dir = up_dir.normalized();
            } else {
                up_dir = Vec3(0.0f, 1.0f, 0.0f);
            }
            apex = curr_pos + up_dir * loop_height;
        }

        // Direction from base to apex
        Vec3 up_dir = (apex - curr_pos).normalized();

        // Tangent scale for smooth curves
        float tangent_scale = loop_height * 0.5f;

        // Curve 1: from previous position to base, curving up toward the loop
        if (in_dist > 0.0001f) {
            // Blend incoming direction with upward direction at the base
            Vec3 base_tangent = (in_dir + up_dir * 0.5f).normalized() * tangent_scale;
            auto to_base = CubicBezier::from_hermite(
                prev_pos, in_dir * (in_dist * 0.4f),
                curr_pos, base_tangent);
            spline.add_segment(to_base);
        }

        // Curve 2: from base up to apex
        auto up_curve = CubicBezier::from_hermite(
            curr_pos, up_dir * tangent_scale,
            apex, up_dir * tangent_scale);
        spline.add_segment(up_curve);

        // Curve 3: from apex back down toward exit
        // Exit point is partway toward next position
        Vec3 exit_pt = curr_pos + out_dir * std::min(out_dist * 0.3f, loop_height * 0.5f);
        Vec3 down_tangent = (-up_dir + out_dir * 0.5f).normalized() * tangent_scale;
        auto down_curve = CubicBezier::from_hermite(
            apex, -up_dir * tangent_scale,
            exit_pt, down_tangent);
        spline.add_segment(down_curve);

    } else {
        // Non-loop segment: simple connector from prev to curr
        if (in_dist < 0.0001f) {
            return spline;  // Degenerate case
        }

        float tangent_scale = in_dist * 0.4f;
        tangent_scale = std::max(tangent_scale, yarn.min_bend_radius * 0.3f);

        // Average tangent for smoothness
        Vec3 avg_dir = (in_dir + out_dir).normalized();
        if (avg_dir.length() < 0.0001f) avg_dir = in_dir;

        auto bezier = CubicBezier::from_hermite(
            prev_pos, in_dir * tangent_scale,
            curr_pos, avg_dir * tangent_scale);
        spline.add_segment(bezier);
    }

    return spline;
}

// Main entry point: build geometry using surface-relaxed positions
GeometryPath build_geometry(const YarnPath& yarn_path,
                            const SurfaceGraph& surface,
                            const YarnProperties& yarn,
                            const Gauge& gauge) {
    (void)gauge;  // Not currently used - positions come from surface

    auto log = yarnpath::logging::get_logger();
    log->info("build_geometry: building geometry for {} segments using surface positions",
              yarn_path.segment_count());

    GeometryPath result;
    const auto& segments = yarn_path.segments();

    if (segments.empty()) {
        return result;
    }

    // Collect all positions from surface in yarn order
    std::vector<Vec3> positions;
    positions.reserve(segments.size());

    for (size_t i = 0; i < segments.size(); ++i) {
        SegmentId seg_id = static_cast<SegmentId>(i);
        if (surface.has_segment(seg_id)) {
            NodeId node_id = surface.node_for_segment(seg_id);
            positions.push_back(surface.node(node_id).position);
        } else {
            // Fallback: shouldn't happen if surface was built correctly
            log->warn("build_geometry: segment {} not found in surface, using origin", seg_id);
            positions.push_back(vec3::zero());
        }
    }

    // Build reverse lookup: loop_segment_id -> positions of segments that pass through it
    // When segment A has segment B in its "through" list, segment A passes through loop B.
    // So B's apex is informed by A's position.
    std::map<SegmentId, std::vector<Vec3>> loop_apex_positions;
    for (size_t i = 0; i < segments.size(); ++i) {
        const auto& seg = segments[i];
        for (SegmentId parent_id : seg.through) {
            loop_apex_positions[parent_id].push_back(positions[i]);
        }
    }

    log->debug("build_geometry: built reverse lookup for {} loops",
               loop_apex_positions.size());

    // Build geometry segment by segment
    for (size_t i = 0; i < segments.size(); ++i) {
        SegmentId seg_id = static_cast<SegmentId>(i);
        const auto& seg = segments[i];

        SegmentGeometry geom;
        geom.segment_id = seg_id;

        // Get positions for this segment and neighbors
        Vec3 prev_pos = (i > 0) ? positions[i - 1] : positions[i];
        Vec3 curr_pos = positions[i];
        Vec3 next_pos = (i < positions.size() - 1) ? positions[i + 1] : positions[i];

        // Get child positions if this is a loop (segments that pass through this one)
        std::vector<Vec3> child_positions;
        auto it = loop_apex_positions.find(seg_id);
        if (it != loop_apex_positions.end()) {
            child_positions = it->second;
        }

        // Build the curve for this segment
        geom.curve = build_segment_curve(
            prev_pos, curr_pos, next_pos,
            seg.forms_loop,
            child_positions,
            yarn);

        // Calculate arc length and max curvature
        geom.arc_length = geom.curve.total_arc_length();
        geom.max_curvature = geom.curve.max_curvature();

        log->debug("  segment {}: forms_loop={}, children={}, arc_length={:.3f}, max_curvature={:.3f}",
                   seg_id, seg.forms_loop, child_positions.size(),
                   geom.arc_length, geom.max_curvature);

        result.segments_.push_back(std::move(geom));
    }

    log->info("build_geometry: built {} segment geometries", result.segments_.size());

    return result;
}

}  // namespace yarnpath
