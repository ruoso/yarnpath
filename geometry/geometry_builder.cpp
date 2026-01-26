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
// For loop segments: creates a cylinder-wrap loop shape (front-to-back in Z)
// For non-loop segments: simple connector curve from prev to curr
//
// The physics simulation positions nodes at:
// - Loop segments: the interlocking point (base of the loop)
// - The apex is determined by where child segments pass through
//
// is_significant: if true, allow direction changes (at passthrough/loop points)
BezierSpline build_segment_curve(
    const Vec3& prev_pos,
    const Vec3& curr_pos,
    const Vec3& next_pos,
    bool forms_loop,
    const std::vector<Vec3>& child_positions,
    const YarnProperties& yarn,
    bool is_significant) {

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
        // Loop segment: yarn curves from current base up to apex, then down to next base
        // Break into segments to control curvature at each critical point:
        // 1. Base turn: curve at curr_pos turning from -Z toward up
        // 2. Straight up: from base exit to apex entry
        // 3. Apex wrap: curve crossing in +Z direction
        // 4. Straight down: from apex exit to next base entry
        // 5. Next base turn: curve turning from down toward -Z

        // Wrap/bend radius based on yarn's ability to bend tightly
        float bend_radius = std::max(yarn.min_bend_radius, yarn.radius * 2.0f);

        // Determine apex position - this is where children pass through
        Vec3 apex;
        float loop_height;

        if (!child_positions.empty()) {
            // Apex is the average position of children passing through
            apex = vec3::zero();
            for (const auto& cp : child_positions) {
                apex += cp;
            }
            apex = apex * (1.0f / static_cast<float>(child_positions.size()));
            loop_height = (apex - curr_pos).length();
        } else {
            // No children yet (live loop) - use default height
            loop_height = yarn.radius * 6.0f;
            apex = curr_pos + Vec3(0.0f, loop_height, 0.0f);
        }

        // Ensure minimum loop height
        if (loop_height < bend_radius * 2.0f) {
            loop_height = bend_radius * 2.0f;
            Vec3 up_dir = (apex - curr_pos);
            if (up_dir.length() > 0.0001f) {
                up_dir = up_dir.normalized();
            } else {
                up_dir = Vec3(0.0f, 1.0f, 0.0f);
            }
            apex = curr_pos + up_dir * loop_height;
        }

        // Direction from current base to apex
        Vec3 base_to_apex = apex - curr_pos;
        Vec3 up_dir = base_to_apex.normalized();

        // Direction from apex to next base
        Vec3 apex_to_next = next_pos - apex;
        float apex_to_next_dist = apex_to_next.length();
        Vec3 down_dir = (apex_to_next_dist > 0.0001f) ? apex_to_next.normalized() : Vec3(0.0f, -1.0f, 0.0f);

        // Key positions for the loop segments
        // Base exit: after turning at base, heading up (offset by bend_radius in up direction)
        Vec3 base_exit = curr_pos + up_dir * bend_radius;

        // Apex entry: before the Z-wrap at apex (offset by bend_radius below apex)
        Vec3 apex_entry = apex - up_dir * bend_radius;

        // Apex exit: after the Z-wrap at apex (offset by bend_radius toward next)
        Vec3 apex_exit = apex + down_dir * bend_radius;

        // Next base entry: before turning at next base (offset by bend_radius above next_pos)
        Vec3 next_entry = next_pos - down_dir * bend_radius;

        // Tangent magnitude for turns (quarter circle at bend_radius)
        float turn_tangent = bend_radius * 0.55f;  // ~0.55 for smooth quarter circle

        // Curve 1: Base turn - from curr_pos (arriving in -Z) curving up
        Vec3 base_in_tangent = Vec3(0.0f, 0.0f, -1.0f) * turn_tangent;
        Vec3 base_out_tangent = up_dir * turn_tangent;
        auto base_turn = CubicBezier::from_hermite(
            curr_pos, base_in_tangent,
            base_exit, base_out_tangent);
        spline.add_segment(base_turn);

        // Curve 2: Straight up from base_exit to apex_entry
        float up_dist = (apex_entry - base_exit).length();
        if (up_dist > 0.01f) {
            Vec3 up_tangent = up_dir * (up_dist * 0.4f);
            auto straight_up = CubicBezier::from_hermite(
                base_exit, up_tangent,
                apex_entry, up_tangent);
            spline.add_segment(straight_up);
        }

        // Curve 3: Apex wrap - curving from up direction to +Z to down direction
        Vec3 apex_in_tangent = up_dir * turn_tangent;
        Vec3 apex_cross_tangent = Vec3(0.0f, 0.0f, 1.0f) * turn_tangent;
        Vec3 apex_out_tangent = down_dir * turn_tangent;

        // First half: entry to apex center (up to +Z)
        auto apex_wrap1 = CubicBezier::from_hermite(
            apex_entry, apex_in_tangent,
            apex, apex_cross_tangent);
        spline.add_segment(apex_wrap1);

        // Second half: apex center to exit (+Z to down)
        auto apex_wrap2 = CubicBezier::from_hermite(
            apex, apex_cross_tangent,
            apex_exit, apex_out_tangent);
        spline.add_segment(apex_wrap2);

        // Curve 4: Straight down from apex_exit to next_entry
        float down_dist = (next_entry - apex_exit).length();
        if (down_dist > 0.01f) {
            Vec3 down_tangent = down_dir * (down_dist * 0.4f);
            auto straight_down = CubicBezier::from_hermite(
                apex_exit, down_tangent,
                next_entry, down_tangent);
            spline.add_segment(straight_down);
        }

        // Curve 5: Next base turn - from down direction curving to -Z
        Vec3 next_in_tangent = down_dir * turn_tangent;
        Vec3 next_out_tangent = Vec3(0.0f, 0.0f, -1.0f) * turn_tangent;
        auto next_turn = CubicBezier::from_hermite(
            next_entry, next_in_tangent,
            next_pos, next_out_tangent);
        spline.add_segment(next_turn);

    } else {
        // Non-loop segment: simple connector from prev to curr
        if (in_dist < 0.0001f) {
            return spline;  // Degenerate case
        }

        float tangent_scale = in_dist * 0.4f;
        tangent_scale = std::max(tangent_scale, yarn.min_bend_radius * 0.3f);

        Vec3 end_tangent;
        if (!is_significant) {
            // Maintain flow direction - use incoming direction as tangent
            // The curve should follow the yarn's natural path without artificial bends
            end_tangent = in_dir;
        } else {
            // Allow direction change at topologically meaningful points
            Vec3 avg_dir = (in_dir + out_dir).normalized();
            if (avg_dir.length() < 0.0001f) avg_dir = in_dir;
            end_tangent = avg_dir;
        }

        auto bezier = CubicBezier::from_hermite(
            prev_pos, in_dir * tangent_scale,
            curr_pos, end_tangent * tangent_scale);
        spline.add_segment(bezier);
    }

    return spline;
}

// Build geometry with callback for visualization/debugging
GeometryPath build_geometry_with_callback(
    const YarnPath& yarn_path,
    const SurfaceGraph& surface,
    const YarnProperties& yarn,
    const Gauge& gauge,
    const GeometryBuildCallback& callback) {

    (void)gauge;  // Not currently used - positions come from surface
    const GeometryBuildCallback* callback_ptr = callback ? &callback : nullptr;

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

    // Track alternation state for passthrough Z-offset
    bool next_passthrough_positive_z = true;  // Start with +Z (behind)

    // Accumulated spline for callback visualization
    BezierSpline accumulated_spline;

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

        // Phase 3: Apply alternating Z-offset at passthrough points
        if (!seg.through.empty()) {
            float z_offset = next_passthrough_positive_z
                ? +yarn.radius * 1.5f   // Behind (+Z)
                : -yarn.radius * 1.5f;  // In front (-Z)

            curr_pos.z += z_offset;

            // Alternate for next passthrough
            next_passthrough_positive_z = !next_passthrough_positive_z;
        }

        // Get child positions if this is a loop (segments that pass through this one)
        std::vector<Vec3> child_positions;
        auto it = loop_apex_positions.find(seg_id);
        if (it != loop_apex_positions.end()) {
            child_positions = it->second;
        }

        // Phase 2: Determine if this is a topologically significant point
        // Only two cases require direction changes:
        // 1. This segment passes through a parent loop (passthrough)
        // 2. This or previous segment forms a loop (entry/exit)
        bool is_significant =
            !seg.through.empty() ||         // Passes through parent loop
            seg.forms_loop ||               // This segment forms a loop
            (i > 0 && segments[i-1].forms_loop);  // Previous was a loop

        // Build the curve for this segment
        geom.curve = build_segment_curve(
            prev_pos, curr_pos, next_pos,
            seg.forms_loop,
            child_positions,
            yarn,
            is_significant);

        // Report each curve segment via callback if provided
        if (callback_ptr) {
            // Log key positions for debugging
            log->info("  seg {} positions: curr=({:.1f},{:.1f},{:.1f}) next=({:.1f},{:.1f},{:.1f})",
                     seg_id, curr_pos.x, curr_pos.y, curr_pos.z,
                     next_pos.x, next_pos.y, next_pos.z);

            const auto& curve_segments = geom.curve.segments();
            for (size_t c = 0; c < curve_segments.size(); ++c) {
                accumulated_spline.add_segment(curve_segments[c]);
                std::string desc;
                if (seg.forms_loop) {
                    // Up to 6 curves, but straight sections may be skipped
                    // Actual order depends on which conditionals pass
                    char buf[64];
                    snprintf(buf, sizeof(buf), "loop: curve %zu of %zu", c + 1, curve_segments.size());
                    desc = buf;
                } else {
                    desc = is_significant ? "connector (significant)" : "connector (flow)";
                }
                (*callback_ptr)(seg_id, desc, accumulated_spline);
            }
        }

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

// Main entry point: build geometry using surface-relaxed positions
GeometryPath build_geometry(const YarnPath& yarn_path,
                            const SurfaceGraph& surface,
                            const YarnProperties& yarn,
                            const Gauge& gauge) {
    // Call the callback version with an empty callback
    return build_geometry_with_callback(yarn_path, surface, yarn, gauge, nullptr);
}

}  // namespace yarnpath
