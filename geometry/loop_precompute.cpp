#include "loop_precompute.hpp"
#include <common/logging.hpp>
#include <cmath>
#include <algorithm>

namespace yarnpath {

LoopShapeParams get_loop_shape_params(
    const YarnSegment& segment,
    const YarnProperties& yarn,
    const Gauge& gauge) {

    return compute_stitch_shape(
        yarn,
        gauge,
        segment.orientation,
        segment.wrap_direction,
        segment.work_type);
}

Vec3 calculate_apex_position(
    const Vec3& curr_pos,
    const std::vector<Vec3>& child_positions,
    float effective_loop_height,
    float yarn_compressed_diameter,
    const Vec3& wale_axis) {

    float apex_height = effective_loop_height * 0.5f;
    // Ensure minimum clearance of one yarn diameter above children
    apex_height = std::max(apex_height, yarn_compressed_diameter);

    if (!child_positions.empty()) {
        Vec3 avg_child = Vec3::zero();
        for (const auto& cp : child_positions) {
            avg_child += cp;
        }
        avg_child = avg_child * (1.0f / static_cast<float>(child_positions.size()));
        return avg_child;
    } else {
        return curr_pos + wale_axis * apex_height;
    }
}

std::map<SegmentId, PrecomputedLoopGeometry> precompute_loop_geometry(
    const std::vector<YarnSegment>& segments,
    const std::vector<SegmentFrame>& frames,
    const std::map<SegmentId, std::vector<Vec3>>& loop_child_positions,
    const std::map<SegmentId, std::vector<SegmentId>>& loop_child_ids,
    const YarnProperties& yarn,
    const Gauge& gauge,
    float effective_loop_height,
    float yarn_compressed_diameter,
    float yarn_compressed_radius) {

    std::map<SegmentId, PrecomputedLoopGeometry> result;

    // --- Pass 1: Basic geometry for all loops ---
    for (size_t i = 0; i < segments.size(); ++i) {
        const auto& seg = segments[i];
        if (!seg.forms_loop) continue;

        SegmentId seg_id = static_cast<SegmentId>(i);
        PrecomputedLoopGeometry geom;

        // 1. Get topology-based shape
        geom.shape = get_loop_shape_params(seg, yarn, gauge);

        // 2. Get child positions for this loop
        std::vector<Vec3> child_positions;
        auto child_pos_it = loop_child_positions.find(seg_id);
        if (child_pos_it != loop_child_positions.end()) {
            child_positions = child_pos_it->second;
        }

        // 3. Calculate apex position using gauge-derived height along wale axis
        Vec3 curr_pos = frames[i].position;
        Vec3 next_pos = (i < frames.size() - 1) ? frames[i + 1].position : frames[i].position;
        Vec3 wale = frames[i].wale_axis;

        geom.oriented_wale = wale;
        bool is_parentless = seg.through.empty();
        geom.stitch_axis = is_parentless
            ? frames[i].stitch_axis * -1.0f
            : frames[i].stitch_axis;

        if (is_parentless) {
            Vec3 stitch = frames[i].stitch_axis;
            float cross_offset = yarn_compressed_radius * 3.0f;
            float dip_depth = yarn_compressed_diameter * 3.0f;

            if (!child_positions.empty()) {
                Vec3 child_center = calculate_apex_position(
                    curr_pos, child_positions,
                    effective_loop_height, yarn_compressed_diameter, wale);

                geom.apex = child_center;

                Vec3 dip_dir = wale * -1.0f;
                Vec3 fnormal = frames[i].fabric_normal;
                float back_offset = yarn_compressed_diameter * 1.5f;
                float bulge_sign = (geom.shape.z_bulge >= 0.0f) ? -1.0f : 1.0f;

                Vec3 shallow_dip = child_center + dip_dir * dip_depth;
                Vec3 deep_dip = child_center + dip_dir * (dip_depth + yarn_compressed_diameter);

                geom.apex_entry = deep_dip + fnormal * (back_offset * bulge_sign);
                geom.apex_exit = shallow_dip + fnormal * (yarn_compressed_diameter * bulge_sign);

                geom.entry_through = deep_dip + stitch * cross_offset;
                geom.exit_through = deep_dip - stitch * cross_offset;
            } else {
                float flat_height = yarn_compressed_radius;
                geom.apex = curr_pos + wale * flat_height;
                geom.apex_entry = curr_pos;
                geom.apex_exit = next_pos;

                Vec3 dip_dir = wale * -1.0f;
                geom.entry_through = curr_pos + stitch * cross_offset + dip_dir * dip_depth;
                geom.exit_through = next_pos - stitch * cross_offset + dip_dir * dip_depth;
            }
        } else {
            geom.apex = calculate_apex_position(curr_pos, child_positions,
                                                effective_loop_height, yarn_compressed_diameter,
                                                wale);

            if (child_positions.empty()) {
                Vec3 apex_offset = geom.apex - curr_pos;
                float wale_height = apex_offset.dot(wale);
                Vec3 lateral = apex_offset - wale * wale_height;
                float scaled_height = wale_height * geom.shape.apex_height_factor * geom.shape.height_multiplier;
                geom.apex = curr_pos + wale * scaled_height + lateral;
            }

            geom.apex = geom.apex + frames[i].stitch_axis * geom.shape.apex_lean_x;

            geom.apex_entry = curr_pos;
            geom.apex_exit = next_pos;

            float dip_depth = yarn_compressed_diameter;
            float apex_wale_sign = (geom.apex - curr_pos).dot(wale);
            Vec3 dip_dir = (apex_wale_sign >= 0.0f) ? wale * -1.0f : wale;

            geom.entry_through = curr_pos + dip_dir * dip_depth;
            geom.exit_through = next_pos + dip_dir * dip_depth;
        }

        Vec3 fnormal = frames[i].fabric_normal;
        geom.fabric_normal = fnormal;

        geom.wrap_radius = gauge.needle_diameter * 0.5f + yarn_compressed_radius;

        geom.clearance_radius = yarn_compressed_diameter * 1.5f;
        if (child_positions.size() > 1) {
            geom.clearance_radius *= 1.0f + 0.2f * (child_positions.size() - 1);
        }

        result[seg_id] = geom;
    }

    // --- Pass 2: Distribute crossover slots along parent loop paths ---
    for (size_t i = 0; i < segments.size(); ++i) {
        const auto& seg = segments[i];
        if (!seg.forms_loop) continue;

        SegmentId seg_id = static_cast<SegmentId>(i);
        auto& geom = result[seg_id];

        std::vector<SegmentId> child_ids;
        auto child_id_it = loop_child_ids.find(seg_id);
        if (child_id_it != loop_child_ids.end()) {
            child_ids = child_id_it->second;
        }
        if (child_ids.empty()) continue;

        Vec3 parent_travel = frames[i].stitch_axis;

        int total_slots = 0;
        for (SegmentId child_id : child_ids) {
            total_slots += segments[child_id].forms_loop ? 2 : 1;
        }

        Vec3 slot_center = geom.apex;
        if (seg.through.empty()) {
            auto child_pos_it = loop_child_positions.find(seg_id);
            if (child_pos_it != loop_child_positions.end() && !child_pos_it->second.empty()) {
                slot_center = Vec3(0, 0, 0);
                for (const auto& cp : child_pos_it->second) {
                    slot_center = slot_center + cp;
                }
                slot_center = slot_center * (1.0f / static_cast<float>(child_pos_it->second.size()));
            }
        }

        Vec3 fabric_normal = frames[i].fabric_normal;

        for (int s = 0; s < total_slots; ++s) {
            float offset = (static_cast<float>(s) - (total_slots - 1) / 2.0f)
                         * yarn_compressed_diameter;

            CrossoverSlot slot;
            slot.position = slot_center + parent_travel * offset;
            slot.tangent = parent_travel;
            slot.crossing_normal = fabric_normal;

            geom.crossover_slots.push_back(slot);
        }
    }

    return result;
}

bool add_waypoint_to_splines(
    GeometryBuildState& state,
    CatmullRomSpline& segment_spline,
    const Vec3& point,
    const std::string& description,
    const CurveAddedCallback& on_curve_added) {

    // Skip degenerate waypoints (too close to previous)
    if (state.running_spline.waypoint_count() > 0) {
        Vec3 prev = state.running_spline.end();
        if ((point - prev).length() < 1e-5f) {
            return false;
        }
    }

    // Check for NaN/Inf
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
        return false;
    }

    state.running_spline.add_waypoint(point);
    segment_spline.add_waypoint(point);
    if (on_curve_added) on_curve_added(description);
    return true;
}

void build_parent_passthrough(
    GeometryBuildState& state,
    CatmullRomSpline& segment_spline,
    const CrossoverData& crossover,
    const CurveAddedCallback& on_curve_added) {

    if (state.running_spline.waypoint_count() == 0) return;

    add_waypoint_to_splines(state, segment_spline, crossover.exit,
                            "passthrough", on_curve_added);
}

void build_full_loop_chain(
    GeometryBuildState& state,
    CatmullRomSpline& segment_spline,
    const PrecomputedLoopGeometry& loop_geom,
    const std::vector<CrossoverData>& entry_crossovers,
    const std::vector<CrossoverData>& exit_crossovers,
    const CurveAddedCallback& on_curve_added,
    const WaypointAddedCallback& on_waypoint_added) {

    if (state.running_spline.waypoint_count() == 0) return;

    std::vector<Vec3> waypoints;
    std::vector<std::string> names;

    // Helper: add a waypoint only if it differs from the previous one
    auto push_waypoint = [&](const Vec3& pt, const std::string& name) {
        if (!waypoints.empty() && (pt - waypoints.back()).length() < 1e-4f) return;
        if (!waypoints.empty()) names.push_back(name);
        waypoints.push_back(pt);
        if (on_waypoint_added) on_waypoint_added(name, pt);
    };

    // Start: current running spline position
    waypoints.push_back(state.running_spline.end());

    // Entry: approach, then cross through parent opening (or just dip)
    if (!entry_crossovers.empty()) {
        Vec3 entry_in = entry_crossovers.front().entry;
        Vec3 entry_out = entry_crossovers.front().exit;
        Vec3 crossing_dir = safe_normalized(entry_out - entry_in, loop_geom.oriented_wale);

        Vec3 start = waypoints[0];
        Vec3 to_entry = entry_in - start;
        float entry_dist = to_entry.length();

        if (entry_dist >= 2.0f * state.yarn_compressed_radius) {
            Vec3 approach = entry_in
                - loop_geom.stitch_axis * state.yarn_compressed_radius
                - crossing_dir * state.yarn_compressed_radius;

            Vec3 travel_dir = safe_normalized(to_entry);
            float approach_proj = (approach - start).dot(travel_dir);
            if (approach_proj < state.yarn_compressed_radius * 0.5f) {
                approach = start + travel_dir * (entry_dist * 0.33f)
                         - crossing_dir * state.yarn_compressed_radius;
            }
            push_waypoint(approach, "loop_approach");
            for (const auto& xover : entry_crossovers) {
                Vec3 mid = (xover.entry + xover.exit) * 0.5f;
                push_waypoint(mid, "crossover_entry");
            }
        } else {
            for (const auto& xover : entry_crossovers) {
                push_waypoint(xover.exit, "crossover_entry_out");
            }
        }
    } else {
        push_waypoint(loop_geom.apex_entry, "loop_approach");
        push_waypoint(loop_geom.entry_through, "loop_entry_dip");
    }

    // Wrap geometry
    Vec3 travel = loop_geom.stitch_axis;
    bool has_children = !loop_geom.crossover_slots.empty();
    int num_slots = static_cast<int>(loop_geom.crossover_slots.size());
    float bundle_radius = state.yarn_compressed_radius * std::sqrt(static_cast<float>(std::max(num_slots, 1)));

    float min_wrap = loop_geom.wrap_radius;
    float wrap_clearance;
    if (has_children) {
        wrap_clearance = std::max({bundle_radius + state.yarn_compressed_radius,
                                   state.yarn_compressed_diameter * 1.5f,
                                   min_wrap});
    } else {
        wrap_clearance = min_wrap;
    }
    float bulge_sign = (loop_geom.shape.z_bulge >= 0.0f) ? -1.0f : 1.0f;
    Vec3 wrap_offset = loop_geom.fabric_normal * (wrap_clearance * bulge_sign);

    Vec3 entry_leg_base;
    Vec3 exit_leg_base;
    if (!entry_crossovers.empty() || !exit_crossovers.empty()) {
        entry_leg_base = entry_crossovers.back().exit;
        exit_leg_base = exit_crossovers.front().entry;
    } else {
        entry_leg_base = loop_geom.entry_through;
        exit_leg_base = loop_geom.exit_through;
    }

    Vec3 bundle_entry = loop_geom.apex - travel * wrap_clearance;
    Vec3 bundle_exit = loop_geom.apex + travel * wrap_clearance;

    // z_bulge offset for leg waypoints
    float z_abs = std::abs(loop_geom.shape.z_bulge);
    float bulge_sign_leg = (loop_geom.shape.z_bulge >= 0.0f) ? 1.0f : -1.0f;

    auto compute_leg_bulge = [&](const Vec3& leg_base, const Vec3& wrap_pt) -> Vec3 {
        float span = (wrap_pt - leg_base).length();
        float half_span = span * 0.5f;
        // Simple scaling: limit bulge to half the span to avoid extreme curvature
        float max_h = half_span * 0.5f;
        float h = std::min(z_abs, max_h);
        return loop_geom.fabric_normal * (h * bulge_sign_leg);
    };

    // RISING LEG
    {
        Vec3 entry_leg_bulge = compute_leg_bulge(entry_leg_base, bundle_entry);
        Vec3 leg_mid = (entry_leg_base + bundle_entry) * 0.5f + entry_leg_bulge;
        push_waypoint(leg_mid, "loop_entry_leg");
    }

    // Wrap: smooth 3-point arc
    push_waypoint(bundle_entry, "loop_wrap_approach");
    push_waypoint(loop_geom.apex + wrap_offset, "loop_apex");
    push_waypoint(bundle_exit, "loop_wrap_depart");

    // FALLING LEG
    {
        Vec3 exit_leg_bulge = compute_leg_bulge(exit_leg_base, bundle_exit);
        Vec3 leg_mid = (exit_leg_base + bundle_exit) * 0.5f + exit_leg_bulge;
        push_waypoint(leg_mid, "loop_exit_leg");
    }

    // Exit crossover
    if (!exit_crossovers.empty()) {
        for (const auto& xover : exit_crossovers) {
            Vec3 mid = (xover.entry + xover.exit) * 0.5f;
            push_waypoint(mid, "crossover_exit");
        }
        Vec3 exit_in = exit_crossovers.back().entry;
        Vec3 exit_out = exit_crossovers.back().exit;
        Vec3 crossing_dir = safe_normalized(exit_in - exit_out, loop_geom.oriented_wale);
        Vec3 depart = exit_out
            + loop_geom.stitch_axis * state.yarn_compressed_diameter
            - crossing_dir * state.yarn_compressed_diameter;

        Vec3 depart_offset = depart - exit_out;
        if (depart_offset.dot(loop_geom.stitch_axis) < 0.0f) {
            depart = exit_out + loop_geom.stitch_axis * state.yarn_compressed_diameter;
        }
        push_waypoint(depart, "loop_depart");
    } else {
        push_waypoint(loop_geom.apex_exit, "loop_exit");
    }

    // Need at least 2 waypoints for a spline
    if (waypoints.size() < 2) return;

    // Debug: dump waypoints and wale projections
    {
        auto log = logging::get_logger();
        Vec3 wale = loop_geom.oriented_wale;
        log->debug("build_full_loop_chain: {} waypoints, wale=({:.3f},{:.3f},{:.3f})",
                   waypoints.size(), wale.x, wale.y, wale.z);
        for (size_t i = 0; i < waypoints.size(); ++i) {
            Vec3 p = waypoints[i];
            float w_proj = (p - waypoints[0]).dot(wale);
            std::string label = (i == 0) ? "start" : names[i-1];
            log->debug("  [{}] {}: ({:.3f},{:.3f},{:.3f}) wale={:.3f}",
                       i, label, p.x, p.y, p.z, w_proj);
        }
    }

    // Add all waypoints to the splines directly — Catmull-Rom handles smoothing
    for (size_t i = 0; i < waypoints.size(); ++i) {
        std::string name = (i < names.size()) ? names[i] : "chain_waypoint";
        add_waypoint_to_splines(state, segment_spline, waypoints[i], name, on_curve_added);
    }
}

}  // namespace yarnpath
