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
// For loop segments: creates a cylinder-wrap loop shape with Z-crossing
// For non-loop segments: simple connector curve from prev to curr
//
// The physics simulation positions nodes at:
// - Loop segments: the interlocking point (base of the loop)
// - The apex is determined by where child segments pass through
//
// is_significant: if true, allow direction changes (at passthrough/loop points)
// current_z_positive: which side of Z the yarn is currently on (true = +Z/back)
BezierSpline build_segment_curve(
    const Vec3& prev_pos,
    const Vec3& curr_pos,
    const Vec3& next_pos,
    bool forms_loop,
    const std::vector<Vec3>& child_positions,
    const YarnProperties& yarn,
    bool current_z_positive) {

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
        // 1. Base turn: curve at curr_pos turning from current Z-side toward up
        // 2. Straight up: from base exit to apex entry
        // 3. Apex wrap: curve crossing to opposite Z-side
        // 4. Straight down: from apex exit to next base entry
        // 5. Next base turn: curve turning from down toward original Z-side

        // Z-direction based on current state (alternates along yarn path)
        float z_sign = current_z_positive ? 1.0f : -1.0f;

        // Wrap offset - small offset for entry/exit points to create curvature
        float wrap_offset = yarn.radius * 0.5f;

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

            // Offset apex higher in Y so the loop goes over the child segment
            apex.y += yarn.radius;

            loop_height = (apex - curr_pos).length();
        } else {
            // No children, this is the last row, should just form a small loop
            loop_height = yarn.radius * 2.0f;
            apex = curr_pos + Vec3(0.0f, loop_height, 0.0f);
        }

        // Adjust wrap offset if loop is too short
        if (loop_height < wrap_offset * 2.0f) {
            wrap_offset = loop_height * 0.4f;
        }

        // Direction from current base to apex
        Vec3 base_to_apex = apex - curr_pos;
        Vec3 up_dir = base_to_apex.normalized();

        // Direction from apex to next base
        Vec3 apex_to_next = next_pos - apex;
        float apex_to_next_dist = apex_to_next.length();
        Vec3 down_dir = (apex_to_next_dist > 0.0001f) ? apex_to_next.normalized() : Vec3(0.0f, -1.0f, 0.0f);

        // Key positions for the loop segments
        // Base center is offset down by yarn.radius to keep aligned with surface
        Vec3 base_center = curr_pos - Vec3(0.0f, yarn.radius, 0.0f);

        // Determine X direction based on yarn flow (toward next segment)
        float x_dir = (next_pos.x >= curr_pos.x) ? 1.0f : -1.0f;

        // Base entry/exit: offset in X following yarn direction, at surface level
        Vec3 base_entry = curr_pos + Vec3(-x_dir * wrap_offset, 0.0f, 0.0f);
        Vec3 base_exit = curr_pos + Vec3(x_dir * wrap_offset, 0.0f, 0.0f);

        // Apex entry/exit: offset in Z, slightly lower in Y so curve goes UP at apex
        Vec3 apex_entry = apex + Vec3(0.0f, -wrap_offset, z_sign * wrap_offset);
        Vec3 apex_exit = apex + Vec3(0.0f, -wrap_offset, -z_sign * wrap_offset);

        // Next base entry: where the down_leg ends (at surface level, on arriving side)
        Vec3 next_base_entry = next_pos + Vec3(-x_dir * wrap_offset, 0.0f, 0.0f);

        // Tangent magnitude for turns (quarter circle at wrap_offset)
        float turn_tangent = wrap_offset * 1.5f;  // Larger tangent for smoother curvature

        // Curve 1: Base wrap - curve dips down at base_center
        // First half: base_entry to base_center (arriving from Z-side, curving down and in X direction)
        auto base_wrap1 = CubicBezier::from_hermite(
            base_entry, Vec3(0.0f, 0.0f, z_sign) * turn_tangent,   // Arriving from Z-side
            base_center, Vec3(x_dir, -1.0f, 0.0f).normalized() * turn_tangent);  // Curving down in X direction
        spline.add_segment(base_wrap1);

        // Second half: base_center to base_exit (curving up and in X direction)
        auto base_wrap2 = CubicBezier::from_hermite(
            base_center, Vec3(x_dir, 1.0f, 0.0f).normalized() * turn_tangent,   // Curving up in X direction
            base_exit, Vec3(0.0f, 1.0f, 0.0f) * turn_tangent);                  // Departing upward
        spline.add_segment(base_wrap2);

        // Curve 2: From base_exit up to apex_entry
        Vec3 to_apex_entry = apex_entry - base_exit;
        float to_apex_dist = to_apex_entry.length();
        if (to_apex_dist > 0.01f) {
            auto up_leg = CubicBezier::from_hermite(
                base_exit, Vec3(0.0f, 1.0f, 0.0f) * (to_apex_dist * 0.4f),  // Departing upward
                apex_entry, Vec3(0.0f, 1.0f, 0.0f) * (to_apex_dist * 0.4f)); // Arriving from below
            spline.add_segment(up_leg);
        }

        // Curve 3: Apex wrap - curve goes up at apex
        // apex_entry is at +z_sign side and below, apex is center and highest, apex_exit is at -z_sign side and below

        // First half: entry to apex center (arriving from below, curving up and across Z)
        auto apex_wrap1 = CubicBezier::from_hermite(
            apex_entry, Vec3(0.0f, 1.0f, 0.0f) * turn_tangent,  // Arriving from below
            apex, Vec3(0.0f, 1.0f, -z_sign).normalized() * turn_tangent);  // Curving up and across Z
        spline.add_segment(apex_wrap1);

        // Second half: apex center to exit (curving down and across Z)
        auto apex_wrap2 = CubicBezier::from_hermite(
            apex, Vec3(0.0f, -1.0f, -z_sign).normalized() * turn_tangent,  // Curving down and across Z
            apex_exit, Vec3(0.0f, -1.0f, 0.0f) * turn_tangent);            // Departing downward
        spline.add_segment(apex_wrap2);

        // Curve 4: From apex_exit down to next_base_entry
        // This ends at the base_entry of the next segment's loop
        Vec3 to_next_base = next_base_entry - apex_exit;
        float to_next_dist = to_next_base.length();
        if (to_next_dist > 0.01f) {
            auto down_leg = CubicBezier::from_hermite(
                apex_exit, Vec3(0.0f, -1.0f, 0.0f) * (to_next_dist * 0.4f),      // Departing downward
                next_base_entry, Vec3(0.0f, -1.0f, 0.0f) * (to_next_dist * 0.4f)); // Arriving from above
            spline.add_segment(down_leg);
        }

    } else {
        // Non-loop segment: simple connector from prev to curr
        if (in_dist < 0.0001f) {
            return spline;  // Degenerate case
        }

        float tangent_scale = in_dist * 0.4f;
        tangent_scale = std::max(tangent_scale, yarn.min_bend_radius * 0.3f);

        auto bezier = CubicBezier::from_hermite(
            prev_pos, in_dir * tangent_scale,
            curr_pos, in_dir * tangent_scale);
        spline.add_segment(bezier);
    }

    return spline;
}

static std::optional<Vec3> get_segment_base_position(
    const YarnPath& yarn_path,
    const SurfaceGraph& surface,
    const YarnProperties& yarn,
    const std::map<SegmentId, std::vector<SegmentId>>& parent_map,
    const std::map<SegmentId, std::vector<SegmentId>>& children_map,
    SegmentId segment_id) {
    if (surface.has_segment(segment_id)) {
        NodeId node_id = surface.node_for_segment(segment_id);
        Vec3 base_pos = surface.node(node_id).position;
        auto parents_it = parent_map.find(segment_id);
        auto children_it = children_map.find(segment_id);
        if (parents_it == parent_map.end() ||
            (parents_it != parent_map.end() && parents_it->second.empty())) {
            // no parents, go through children, find the lowest Y,
            // make the base that - yarn diameter
            float min_y = std::numeric_limits<float>::max();
            if (children_it != children_map.end()) {
                for (SegmentId child_id : children_it->second) {
                    auto child_pos_opt = get_segment_base_position(
                        yarn_path, surface, yarn,
                        parent_map, children_map, child_id);
                    if (child_pos_opt.has_value()) {
                        Vec3 child_pos = child_pos_opt.value();
                        min_y = std::min(min_y, child_pos.y);
                    }
                }
            }
            if (min_y > base_pos.y) {
                yarnpath::logging::get_logger()->debug(
                    "get_segment_base_position: segment {} has no parents, adjusting base position from {} to {}",
                    segment_id, base_pos.y, min_y - yarn.radius);
                base_pos.y = min_y - yarn.radius;
            }
        }
        return base_pos;
    }
    return std::nullopt;
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

    // Build reverse lookup of parents for a given segment, allowing to identify which
    // segments have this as a child. If it doesn't have any, the base can move close 
    // to the apex.
    std::map<SegmentId, std::vector<SegmentId>> segment_parents;
    for (size_t i = 0; i < segments.size(); ++i) {
        const auto& seg = segments[i];
        for (SegmentId parent_id : seg.through) {
            segment_parents[i].push_back(parent_id);
        }
    }

    // also a simple lookup of children for a given segment
    std::map<SegmentId, std::vector<SegmentId>> segment_children;
    for (size_t i = 0; i < segments.size(); ++i) {
        const auto& seg = segments[i];
        for (SegmentId parent_id : seg.through) {
            segment_children[parent_id].push_back(i);
        }
    }

    // Collect all positions from surface in yarn order
    std::vector<Vec3> positions;
    positions.reserve(segments.size());

    for (size_t i = 0; i < segments.size(); ++i) {
        SegmentId seg_id = static_cast<SegmentId>(i);
        auto pos_opt = get_segment_base_position(
            yarn_path, surface, yarn,
            segment_parents, segment_children, seg_id);
        if (!pos_opt.has_value()) {
            log->warn("build_geometry: segment {} position could not be determined, using origin", seg_id);
            pos_opt = vec3::zero();
        }
        positions.push_back(pos_opt.value());
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

    // Track Z-state along yarn path - which side of Z the yarn is currently on
    // This alternates each time yarn crosses Z (at passthrough points)
    bool current_z_positive = true;  // Start on +Z (back) side

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

        // Get child positions if this is a loop (segments that pass through this one)
        std::vector<Vec3> child_positions;
        auto it = loop_apex_positions.find(seg_id);
        if (it != loop_apex_positions.end()) {
            child_positions = it->second;
        }

        // if the next segment is a child of this segment, we need to flip
        // current_z_positive for the next segment, as that is the end of the row
        if (i < segments.size() - 1) {
            const auto& next_seg = segments[i + 1];
            if (std::find(next_seg.through.begin(), next_seg.through.end(), seg_id) != next_seg.through.end()) {
                current_z_positive = !current_z_positive;
            }
        }

        // Build the curve for this segment
        geom.curve = build_segment_curve(
            prev_pos, curr_pos, next_pos,
            seg.forms_loop,
            child_positions,
            yarn,
            current_z_positive);

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
                    desc = "connector";
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
