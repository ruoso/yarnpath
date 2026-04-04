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
        // Apex is at the average child position. The solver already places
        // children at the correct physical distance. 3D wrapping around
        // children is handled by build_full_loop_chain().
        return avg_child;
    } else {
        // No children - loop above current position along wale axis.
        // Use the frame's wale direction directly; the caller's test
        // projects onto the same axis so the sign is consistent.
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
        // Parentless loops wrap against the stitch axis (crossed loop)
        geom.stitch_axis = is_parentless
            ? frames[i].stitch_axis * -1.0f
            : frames[i].stitch_axis;

        if (is_parentless) {
            // Parentless segments (cast-on foundation, YO, M1L/M1R):
            // crossed loop centered at the child base when children exist.
            // The entire loop (dip, entry/exit, wrap) lives at the child
            // level. Without children, use a flat loop at the segment base.
            //
            // Offsets are generous (3× radius, 2× diameter) to prevent
            // tube self-overlap: the loop doubles back on itself, so
            // entry and exit sides need ample separation in stitch,
            // wale, and fabric-normal directions.
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

                // The entry dips deep in the wale direction before the
                // leg rise, while the exit departs at a shallower level.
                // This vertical separation ensures the transit between
                // adjacent foundation loops (at the shallow level) passes
                // above the loop body (at the deep level).
                Vec3 shallow_dip = child_center + dip_dir * dip_depth;
                Vec3 deep_dip = child_center + dip_dir * (dip_depth + yarn_compressed_diameter);

                geom.apex_entry = deep_dip + fnormal * (back_offset * bulge_sign);
                geom.apex_exit = shallow_dip + fnormal * (yarn_compressed_diameter * bulge_sign);

                // Crossed dip points at the deep level
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

            // Apply shape modifiers to apex along wale axis (only when no
            // children exist — solver-derived child positions are authoritative)
            if (child_positions.empty()) {
                Vec3 apex_offset = geom.apex - curr_pos;
                float wale_height = apex_offset.dot(wale);
                Vec3 lateral = apex_offset - wale * wale_height;
                float scaled_height = wale_height * geom.shape.apex_height_factor * geom.shape.height_multiplier;
                geom.apex = curr_pos + wale * scaled_height + lateral;
            }

            // Apply lean along stitch_axis (not world X) for decreases
            geom.apex = geom.apex + frames[i].stitch_axis * geom.shape.apex_lean_x;

            // 4. Entry/exit at base positions
            geom.apex_entry = curr_pos;
            geom.apex_exit = next_pos;

            // 5. U-shape through-opening waypoints: yarn dips below base.
            // No z_bulge here — the dip is at base Z.  The z_bulge appears
            // on the loop legs (between dip and apex), added at build time.
            // The dip direction is opposite to the apex direction (toward parents).
            float dip_depth = yarn_compressed_diameter;
            float apex_wale_sign = (geom.apex - curr_pos).dot(wale);
            Vec3 dip_dir = (apex_wale_sign >= 0.0f) ? wale * -1.0f : wale;

            geom.entry_through = curr_pos + dip_dir * dip_depth;
            geom.exit_through = next_pos + dip_dir * dip_depth;
        }

        Vec3 fnormal = frames[i].fabric_normal;
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

        // Use stitch_axis for slot distribution (guaranteed perpendicular to wale)
        Vec3 parent_travel = frames[i].stitch_axis;

        // Count total crossover slots needed:
        // loop-forming children need 2 (entry + exit), others need 1
        int total_slots = 0;
        for (SegmentId child_id : child_ids) {
            total_slots += segments[child_id].forms_loop ? 2 : 1;
        }

        // For parentless segments, place crossover slots at the base of
        // the child nodes (where children actually sit). For regular loops,
        // slots go at the parent's apex.
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
    const CurveAddedCallback& on_curve_added,
    const WaypointAddedCallback& on_waypoint_added) {

    if (state.running_spline.empty()) return;

    std::vector<Vec3> waypoints;
    std::vector<std::string> names;

    // Helper: add a waypoint only if it differs from the previous one
    auto push_waypoint = [&](const Vec3& pt, const std::string& name) {
        if (!waypoints.empty() && (pt - waypoints.back()).length() < 1e-4f) return;
        if (!waypoints.empty()) names.push_back(name);  // name labels the *curve* from prev to this
        waypoints.push_back(pt);
        if (on_waypoint_added) on_waypoint_added(name, pt);
    };

    // Start: current running spline position
    waypoints.push_back(state.running_spline.segments().back().end());

    // Entry: approach, then cross through parent opening (or just dip)
    if (!entry_crossovers.empty()) {
        Vec3 entry_in = entry_crossovers.front().entry;
        Vec3 entry_out = entry_crossovers.front().exit;
        Vec3 crossing_dir = safe_normalized(entry_out - entry_in, loop_geom.oriented_wale);

        Vec3 start = waypoints[0];
        Vec3 to_entry = entry_in - start;
        float entry_dist = to_entry.length();

        if (entry_dist >= 2.0f * state.yarn_compressed_radius) {
            // Offset back along stitch_axis AND down along crossing direction
            // so the child yarn body clears the parent yarn body
            Vec3 approach = entry_in
                - loop_geom.stitch_axis * state.yarn_compressed_radius
                - crossing_dir * state.yarn_compressed_radius;

            // Verify approach doesn't reverse direction relative to start,
            // which would create a U-turn and extreme curvature
            Vec3 travel_dir = safe_normalized(to_entry);
            float approach_proj = (approach - start).dot(travel_dir);
            if (approach_proj < state.yarn_compressed_radius * 0.5f) {
                // Approach would reverse; place it partway toward entry instead
                approach = start + travel_dir * (entry_dist * 0.33f)
                         - crossing_dir * state.yarn_compressed_radius;
            }
            push_waypoint(approach, "loop_approach");
            for (const auto& xover : entry_crossovers) {
                push_waypoint(xover.entry, "crossover_entry_in");
                push_waypoint(xover.exit, "crossover_entry_out");
            }
        } else {
            // Start is too close to crossover entry — skip approach and entry_in
            // to avoid a tight bend. Go directly to crossover exit.
            for (const auto& xover : entry_crossovers) {
                push_waypoint(xover.exit, "crossover_entry_out");
            }
        }
    } else {
        // No parent to cross through — dip below the entry to create a
        // U-shape, keeping the waypoint close to start for tangent continuity
        push_waypoint(loop_geom.apex_entry, "loop_approach");
        push_waypoint(loop_geom.entry_through, "loop_entry_dip");
    }

    // z_bulge offset for leg waypoints
    Vec3 leg_bulge = loop_geom.fabric_normal * loop_geom.shape.z_bulge;

    // Compute the convex wrapping clearance for child yarns.
    // Children passing through form a bundle of cylinders.  Rather than
    // lining them up and computing min/max extents, we model the bundle
    // as a single equivalent circle whose area equals N child circles:
    //   bundle_radius = yarn_compressed_radius * sqrt(N)
    // The parent loop wraps around this bundle.
    //
    // Use the precomputed stitch_axis (course direction) for wrap waypoint
    // distribution, NOT the direction from apex_entry to apex_exit — the
    // latter can point to the next row for end-of-row stitches.
    Vec3 travel = loop_geom.stitch_axis;
    bool has_children = !loop_geom.crossover_slots.empty();
    int num_slots = static_cast<int>(loop_geom.crossover_slots.size());
    float bundle_radius = state.yarn_compressed_radius * std::sqrt(static_cast<float>(std::max(num_slots, 1)));

    // The parent must clear the bundle plus its own radius in the
    // fabric-normal direction.  The wrap goes on the ANTI-bulge side:
    // knit loops have legs toward +fabric_normal, wrap toward -.
    // For loops without children, use the needle wrap radius for clearance.
    // Enforce a minimum of 1.5× yarn diameter so small child bundles
    // (e.g., single-child foundation loops) still have enough wrap room.
    float wrap_clearance;
    if (has_children) {
        wrap_clearance = std::max(bundle_radius + state.yarn_compressed_radius,
                                  state.yarn_compressed_diameter * 1.5f);
    } else {
        wrap_clearance = loop_geom.wrap_radius;
    }
    float bulge_sign = (loop_geom.shape.z_bulge >= 0.0f) ? -1.0f : 1.0f;
    Vec3 wrap_offset = loop_geom.fabric_normal * (wrap_clearance * bulge_sign);

    // The base of each leg: where the yarn emerges after crossing the
    // parent, or the dip point if there's no parent to cross through.
    Vec3 entry_leg_base;
    Vec3 exit_leg_base;
    if (!entry_crossovers.empty() || !exit_crossovers.empty()) {
        // Has parent: leg starts/ends at crossover openings
        entry_leg_base = entry_crossovers.back().exit;
        exit_leg_base = exit_crossovers.front().entry;
    } else {
        // No parent: leg starts/ends at dip points
        entry_leg_base = loop_geom.entry_through;
        exit_leg_base = loop_geom.exit_through;
    }

    // Compute wrap/apex targets first so legs can interpolate toward them.
    // Both cases (with/without children) use the same structure: the wrap
    // targets are offset from the apex along the travel direction and the
    // fabric normal. For children, clearance is based on the child bundle;
    // for no-children, it's based on the needle wrap radius.
    Vec3 bundle_entry = loop_geom.apex - travel * wrap_clearance;
    Vec3 bundle_exit = loop_geom.apex + travel * wrap_clearance;
    Vec3 entry_wrap_target = bundle_entry + wrap_offset;
    Vec3 exit_wrap_target = bundle_exit + wrap_offset;

    // Legs rise from the crossover base toward the wrap target.
    // Position = 3D midpoint between crossover base and wrap target + z_bulge.
    // This makes legs fan out laterally (along stitch_axis) from the narrow
    // crossover toward the wider wrap, matching the physical loop shape.

    bool is_parentless = entry_crossovers.empty() && exit_crossovers.empty();

    // RISING LEG — both parented and parentless loops cross to the bulge
    // side before wrapping, creating a symmetric U-shape.
    {
        Vec3 leg_mid = (entry_leg_base + entry_wrap_target) * 0.5f + leg_bulge;
        push_waypoint(leg_mid, "loop_entry_leg");
    }

    // Wrap approach/depart at the fabric plane level (1× wrap_offset from
    // wrap_entry/exit).  The wrap itself is on the anti-bulge side; the
    // approach/depart guide the yarn through the fabric-plane crossing.
    // Same logic for both parented and parentless — the wrap geometry is
    // the same, only the passthrough differs.
    {
        Vec3 wrap_approach = entry_wrap_target - wrap_offset;
        push_waypoint(wrap_approach, "loop_wrap_approach");
    }
    push_waypoint(entry_wrap_target, "loop_wrap_entry");
    push_waypoint(loop_geom.apex + wrap_offset, "loop_apex");
    push_waypoint(exit_wrap_target, "loop_wrap_exit");
    {
        Vec3 wrap_depart = exit_wrap_target - wrap_offset;
        push_waypoint(wrap_depart, "loop_wrap_depart");
    }

    // FALLING LEG
    // For parentless: exit leg keeps z_bulge so the yarn comes to the
    // front after the wrap before going back down.
    {
        Vec3 leg_mid = (exit_leg_base + exit_wrap_target) * 0.5f + leg_bulge;
        push_waypoint(leg_mid, "loop_exit_leg");
    }

    // Exit crossover: thread back through parent loop opening
    if (!exit_crossovers.empty()) {
        for (const auto& xover : exit_crossovers) {
            push_waypoint(xover.entry, "crossover_exit_in");
            push_waypoint(xover.exit, "crossover_exit_out");
        }
        Vec3 exit_in = exit_crossovers.back().entry;
        Vec3 exit_out = exit_crossovers.back().exit;
        Vec3 crossing_dir = safe_normalized(exit_in - exit_out, loop_geom.oriented_wale);
        // Offset forward along stitch_axis AND down along crossing direction
        Vec3 depart = exit_out
            + loop_geom.stitch_axis * state.yarn_compressed_radius
            - crossing_dir * state.yarn_compressed_radius;

        // Verify depart doesn't reverse direction relative to exit_out
        Vec3 depart_offset = depart - exit_out;
        if (depart_offset.dot(loop_geom.stitch_axis) < 0.0f) {
            // Depart would reverse; place it slightly forward of exit_out
            depart = exit_out + loop_geom.stitch_axis * state.yarn_compressed_radius;
        }
        push_waypoint(depart, "loop_depart");
    } else {
        // No parent to cross through — go straight from exit leg to the
        // depart level.  The exit_through dip is skipped; the exit leg
        // already brings the yarn down from the wrap.
        push_waypoint(loop_geom.apex_exit, "loop_exit");
    }

    // Need at least 2 waypoints for a chain
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
