#include "loop_precompute.hpp"
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
    float yarn_compressed_diameter) {

    float apex_height = effective_loop_height * 0.5f;
    // Ensure minimum clearance of one yarn diameter above children
    apex_height = std::max(apex_height, yarn_compressed_diameter);

    if (!child_positions.empty()) {
        Vec3 apex = Vec3::zero();
        for (const auto& cp : child_positions) {
            apex += cp;
        }
        apex = apex * (1.0f / static_cast<float>(child_positions.size()));
        apex.y += apex_height;
        return apex;
    } else {
        // No children - loop above current position
        return curr_pos + Vec3(0.0f, apex_height, 0.0f);
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

        // 3. Calculate apex position using gauge-derived height
        Vec3 curr_pos = frames[i].position;
        Vec3 next_pos = (i < frames.size() - 1) ? frames[i + 1].position : frames[i].position;
        geom.apex = calculate_apex_position(curr_pos, child_positions,
                                            effective_loop_height, yarn_compressed_diameter);

        // Apply shape modifiers to apex
        geom.apex.y = curr_pos.y + (geom.apex.y - curr_pos.y) * geom.shape.apex_height_factor * geom.shape.height_multiplier;

        // Apply lean along stitch_axis (not world X) for decreases
        geom.apex = geom.apex + frames[i].stitch_axis * geom.shape.apex_lean_x;

        // Apex wraps the needle and returns to the real Z position.
        // z_bulge is applied to the legs (entry_through/exit_through), not here.

        // 4. Entry/exit at base positions
        geom.apex_entry = curr_pos;
        geom.apex_exit = next_pos;

        // 5. U-shape through-opening waypoints: yarn dips below base.
        // No z_bulge here — the dip is at base Z.  The z_bulge appears
        // on the loop legs (between dip and apex), added at build time.
        float dip_depth = yarn_compressed_diameter;
        Vec3 wale = frames[i].wale_axis;
        Vec3 fnormal = frames[i].fabric_normal;

        geom.entry_through = curr_pos - wale * dip_depth;
        geom.exit_through = next_pos - wale * dip_depth;
        geom.fabric_normal = fnormal;

        // Wrap radius: curvature at apex bounded by needle wrap
        geom.wrap_radius = gauge.needle_diameter * 0.5f + yarn_compressed_radius;

        // 6. Calculate clearance radius
        geom.clearance_radius = yarn_compressed_diameter * 1.5f;
        if (child_positions.size() > 1) {
            geom.clearance_radius *= 1.0f + 0.2f * (child_positions.size() - 1);
        }

        result[seg_id] = geom;
    }

    // --- Pass 2: Distribute crossover slots along parent loop paths ---
    // The parent decides where crossings happen by distributing unbiased slots
    // evenly along its loop path. Children claim slots at build time.
    for (size_t i = 0; i < segments.size(); ++i) {
        const auto& seg = segments[i];
        if (!seg.forms_loop) continue;

        SegmentId seg_id = static_cast<SegmentId>(i);
        auto& geom = result[seg_id];

        // Get child IDs for this loop
        std::vector<SegmentId> child_ids;
        auto child_id_it = loop_child_ids.find(seg_id);
        if (child_id_it != loop_child_ids.end()) {
            child_ids = child_id_it->second;
        }
        if (child_ids.empty()) continue;

        // Determine the z_sign from the first child's orientation
        LoopShapeParams first_child_shape = get_loop_shape_params(segments[child_ids[0]], yarn, gauge);
        float z_sign = (first_child_shape.z_bulge >= 0) ? 1.0f : -1.0f;

        // Compute parent travel direction for blending offsets
        Vec3 parent_travel = safe_normalized(
            geom.apex_exit - geom.apex_entry, Vec3(1.0f, 0.0f, 0.0f));

        // Count total crossover slots needed:
        // loop-forming children need 2 (entry + exit), others need 1
        int total_slots = 0;
        for (SegmentId child_id : child_ids) {
            total_slots += segments[child_id].forms_loop ? 2 : 1;
        }

        // Distribute slots at the parent loop's APEX, where the loop is
        // most flexible and has the widest opening.  The parent loop path
        // passes through these positions, and children cross here on their
        // rising / falling legs.
        Vec3 fabric_normal = frames[i].fabric_normal;

        for (int s = 0; s < total_slots; ++s) {
            float offset = (static_cast<float>(s) - (total_slots - 1) / 2.0f)
                         * yarn_compressed_diameter;

            CrossoverSlot slot;
            slot.position = geom.apex + parent_travel * offset;
            slot.tangent = parent_travel;
            slot.crossing_normal = fabric_normal;

            geom.crossover_slots.push_back(slot);
        }
    }

    return result;
}

bool add_curve_if_valid(
    GeometryBuildState& state,
    BezierSpline& segment_spline,
    const CubicBezier& curve,
    const std::string& description,
    const CurveAddedCallback& on_curve_added) {
    float chord = (curve.end() - curve.start()).length();
    if (chord < 1e-5f) {
        return false;  // Skip degenerate curve
    }
    if (!has_valid_control_points(curve)) {
        return false;  // Skip curves with NaN/Inf
    }

    state.running_spline.add_segment(curve);
    segment_spline.add_segment(curve);
    if (on_curve_added) on_curve_added(description);
    return true;
}

void add_connector_with_curvature_check(
    GeometryBuildState& state,
    BezierSpline& segment_spline,
    const Vec3& start, const Vec3& start_dir,
    const Vec3& end, const Vec3& end_dir,
    const std::string& description,
    const CurveAddedCallback& on_curve_added) {

    auto curve = build_curvature_safe_hermite(
        start, start_dir, end, end_dir, state.max_curvature);
    add_curve_if_valid(state, segment_spline, curve, description, on_curve_added);
}

void build_surface_guided_loop(
    GeometryBuildState& state,
    BezierSpline& segment_spline,
    const PrecomputedLoopGeometry& loop_geom,
    const CurveAddedCallback& on_curve_added) {

    // 5-waypoint U-shaped loop:
    //   entry → entry_through (dip into parent opening)
    //   entry_through → apex   (rise to top of loop)
    //   apex → exit_through    (descend back into parent opening)
    //   exit_through → exit    (rise to base level)
    std::vector<Vec3> waypoints = {
        loop_geom.apex_entry,
        loop_geom.entry_through,
        loop_geom.apex,
        loop_geom.exit_through,
        loop_geom.apex_exit
    };
    std::vector<std::string> names = {
        "loop_entry_dip", "loop_rise", "loop_fall", "loop_exit_rise"
    };

    // Entry tangent direction from the running spline, scaled to the first
    // chord length.  The natural spline expects M_0 with magnitude ~chord.
    // Using the raw connector derivative (whose magnitude reflects the
    // connector's own chord length) would cause overshooting.
    Vec3 entry_tangent;
    if (!state.running_spline.segments().empty()) {
        Vec3 dir = safe_normalized(
            state.running_spline.segments().back().derivative(1.0f));
        float first_chord = (waypoints[1] - waypoints[0]).length();
        entry_tangent = dir * first_chord;
    } else {
        entry_tangent = waypoints[1] - waypoints[0];
    }

    auto curves = build_curvature_safe_hermite_chain(
        waypoints, entry_tangent, state.max_curvature);
    for (int i = 0; i < static_cast<int>(curves.size()); ++i) {
        add_curve_if_valid(state, segment_spline, curves[i], names[i], on_curve_added);
    }
}

void build_surface_guided_loop_with_crossings(
    GeometryBuildState& state,
    BezierSpline& segment_spline,
    const PrecomputedLoopGeometry& loop_geom,
    const CurveAddedCallback& on_curve_added) {

    const Vec3& entry = loop_geom.apex_entry;
    const Vec3& entry_through = loop_geom.entry_through;
    const Vec3& exit_through = loop_geom.exit_through;
    const Vec3& exit_pt = loop_geom.apex_exit;
    const auto& crossover_slots = loop_geom.crossover_slots;

    // Travel direction
    Vec3 travel_dir = safe_normalized(exit_pt - entry, Vec3(1.0f, 0.0f, 0.0f));

    // Sort crossover slots by projection onto travel direction
    int total_slots = static_cast<int>(crossover_slots.size());
    struct SlotWithProj {
        int index;
        float proj;
    };
    std::vector<SlotWithProj> sorted;
    for (int s = 0; s < total_slots; ++s) {
        float proj = (crossover_slots[s].position - entry).dot(travel_dir);
        sorted.push_back({s, proj});
    }
    std::sort(sorted.begin(), sorted.end(),
        [](const SlotWithProj& a, const SlotWithProj& b) { return a.proj < b.proj; });

    // Build waypoint sequence:
    // entry → entry_through → [crossover slots near apex] → exit_through → exit
    std::vector<Vec3> waypoints;
    std::vector<std::string> names;

    waypoints.push_back(entry);
    names.push_back("loop_entry_dip");

    waypoints.push_back(entry_through);
    names.push_back("loop_rise");

    for (const auto& sp : sorted) {
        waypoints.push_back(crossover_slots[sp.index].position);
        names.push_back("loop_crossing");
    }

    waypoints.push_back(exit_through);
    names.push_back("loop_fall");

    waypoints.push_back(exit_pt);

    // Entry tangent: direction from running spline, magnitude = first chord.
    Vec3 entry_tangent;
    if (!state.running_spline.segments().empty()) {
        Vec3 dir = safe_normalized(
            state.running_spline.segments().back().derivative(1.0f));
        float first_chord = (waypoints[1] - waypoints[0]).length();
        entry_tangent = dir * first_chord;
    } else {
        entry_tangent = waypoints[1] - waypoints[0];
    }

    auto curves = build_curvature_safe_hermite_chain(
        waypoints, entry_tangent, state.max_curvature);
    for (int i = 0; i < static_cast<int>(curves.size()); ++i) {
        add_curve_if_valid(state, segment_spline, curves[i], names[i], on_curve_added);
    }
}

void build_parent_passthrough(
    GeometryBuildState& state,
    BezierSpline& segment_spline,
    const CrossoverData& crossover,
    const CurveAddedCallback& on_curve_added) {

    if (state.running_spline.empty()) return;

    Vec3 current_end = state.running_spline.segments().back().end();
    Vec3 current_dir = state.running_spline.segments().back().tangent(1.0f);
    Vec3 to_exit = crossover.exit - current_end;
    if (to_exit.length() < 1e-5f) return;

    add_connector_with_curvature_check(
        state, segment_spline,
        current_end, current_dir,
        crossover.exit, crossover.exit_direction,
        "passthrough", on_curve_added);
}

void build_full_loop_chain(
    GeometryBuildState& state,
    BezierSpline& segment_spline,
    const PrecomputedLoopGeometry& loop_geom,
    const std::vector<CrossoverData>& entry_crossovers,
    const std::vector<CrossoverData>& exit_crossovers,
    const CurveAddedCallback& on_curve_added) {

    if (state.running_spline.empty()) return;

    std::vector<Vec3> waypoints;
    std::vector<std::string> names;

    // Helper: add a waypoint only if it differs from the previous one
    auto push_waypoint = [&](const Vec3& pt, const std::string& name) {
        if (!waypoints.empty() && (pt - waypoints.back()).length() < 1e-4f) return;
        if (!waypoints.empty()) names.push_back(name);  // name labels the *curve* from prev to this
        waypoints.push_back(pt);
    };

    // Start: current running spline position
    waypoints.push_back(state.running_spline.segments().back().end());

    // Approach: guide toward loop entry (base level)
    push_waypoint(loop_geom.apex_entry, "loop_approach");

    // Entry dip: yarn dips below parent base through the opening (no z_bulge)
    push_waypoint(loop_geom.entry_through, "loop_entry_dip");

    // z_bulge offset for leg waypoints
    Vec3 leg_bulge = loop_geom.fabric_normal * loop_geom.shape.z_bulge;

    // Compute the convex wrapping clearance for child yarns.
    // Children passing through form a bundle of cylinders.  Rather than
    // lining them up and computing min/max extents, we model the bundle
    // as a single equivalent circle whose area equals N child circles:
    //   bundle_radius = yarn_compressed_radius * sqrt(N)
    // The parent loop wraps around this bundle.
    Vec3 travel = safe_normalized(
        loop_geom.apex_exit - loop_geom.apex_entry, Vec3(1, 0, 0));
    bool has_children = !loop_geom.crossover_slots.empty();
    int num_slots = static_cast<int>(loop_geom.crossover_slots.size());
    float bundle_radius = state.yarn_compressed_radius * std::sqrt(static_cast<float>(std::max(num_slots, 1)));

    // The parent must clear the bundle plus its own radius in the
    // fabric-normal direction.  The wrap must follow the same side as
    // z_bulge: knit loops bulge toward +fabric_normal, purl toward -.
    float wrap_clearance = bundle_radius + state.yarn_compressed_radius;
    float bulge_sign = (loop_geom.shape.z_bulge >= 0.0f) ? -1.0f : 1.0f;
    Vec3 wrap_offset = loop_geom.fabric_normal * (wrap_clearance * bulge_sign);

    // RISING LEG: midpoint between dip and apex, with z_bulge
    {
        Vec3 leg_mid = (loop_geom.entry_through + loop_geom.apex) * 0.5f + leg_bulge;
        push_waypoint(leg_mid, "loop_entry_leg");
    }

    if (has_children) {
        // Neutralize the leg bulge before the wrap begins: a waypoint
        // at the apex level (no bulge) just outside the wrap region.
        Vec3 pre_wrap = loop_geom.apex - travel * wrap_clearance;
        push_waypoint(pre_wrap, "loop_pre_wrap");

        // Wrap waypoints sit one yarn radius from the bundle circle edge,
        // i.e. wrap_clearance = bundle_radius + yarn_compressed_radius
        // from the bundle center, in both travel and normal directions.
        Vec3 bundle_entry = loop_geom.apex - travel * wrap_clearance;
        push_waypoint(bundle_entry + wrap_offset, "loop_wrap_entry");

        // Apex: over the center of the bundle, offset outward
        push_waypoint(loop_geom.apex + wrap_offset, "loop_apex");

        Vec3 bundle_exit = loop_geom.apex + travel * wrap_clearance;
        push_waypoint(bundle_exit + wrap_offset, "loop_wrap_exit");

        // Neutralize the wrap offset after exiting: back to apex level.
        Vec3 post_wrap = loop_geom.apex + travel * wrap_clearance;
        push_waypoint(post_wrap, "loop_post_wrap");
    } else {
        push_waypoint(loop_geom.apex, "loop_apex");
    }

    // FALLING LEG: midpoint between apex and dip, with z_bulge
    {
        Vec3 leg_mid = (loop_geom.apex + loop_geom.exit_through) * 0.5f + leg_bulge;
        push_waypoint(leg_mid, "loop_exit_leg");
    }

    // Exit dip: yarn dips back below parent base (no z_bulge)
    push_waypoint(loop_geom.exit_through, "loop_exit_dip");

    // Exit: back to base level
    push_waypoint(loop_geom.apex_exit, "loop_exit");

    // Need at least 2 waypoints for a chain
    if (waypoints.size() < 2) return;

    // Entry tangent: direction from running spline, magnitude = first chord
    Vec3 dir = safe_normalized(
        state.running_spline.segments().back().derivative(1.0f));
    float first_chord = (waypoints[1] - waypoints[0]).length();
    Vec3 entry_tangent = dir * first_chord;

    auto curves = build_curvature_safe_hermite_chain(
        waypoints, entry_tangent, state.max_curvature);
    for (int i = 0; i < static_cast<int>(curves.size()); ++i) {
        std::string name = (i < static_cast<int>(names.size())) ? names[i] : "chain_curve";
        add_curve_if_valid(state, segment_spline, curves[i], name, on_curve_added);
    }
}

void initialize_running_spline(
    GeometryBuildState& state,
    const Vec3& start_pos,
    const Vec3& first_target,
    BezierSpline& segment_spline,
    const CurveAddedCallback& on_curve_added) {

    Vec3 initial_dir = safe_normalized(first_target - start_pos);

    // Tail length: enough runway for smooth transitions
    float min_bend_radius = 1.0f / state.max_curvature;
    float init_length = std::max(state.effective_stitch_width, min_bend_radius * 3.0f);
    auto init_seg = build_curvature_safe_hermite(
        start_pos - initial_dir * init_length, initial_dir,
        start_pos, initial_dir,
        state.max_curvature);
    add_curve_if_valid(state, segment_spline, init_seg, "init_tail", on_curve_added);
}

}  // namespace yarnpath
