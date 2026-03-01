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

Vec3 compute_fabric_normal(
    const std::vector<Vec3>& positions, size_t idx, float z_sign) {

    Vec3 prev = (idx > 0) ? positions[idx - 1] : positions[idx];
    Vec3 next = (idx + 1 < positions.size()) ? positions[idx + 1] : positions[idx];
    Vec3 tangent = safe_normalized(next - prev);

    // Cross tangent with up to get a vector perpendicular to the fabric
    Vec3 up = Vec3(0.0f, 1.0f, 0.0f);
    Vec3 raw_normal = tangent.cross(up);
    if (raw_normal.length() < 1e-6f) {
        raw_normal = Vec3(0.0f, 0.0f, 1.0f);  // fallback for vertical tangents
    }
    Vec3 normal = safe_normalized(raw_normal);

    // Orient so that positive normal = front side (matching z_bulge convention)
    if (normal.dot(Vec3(0, 0, z_sign)) < 0) {
        normal = normal * (-1.0f);
    }
    return normal;
}

std::map<SegmentId, PrecomputedLoopGeometry> precompute_loop_geometry(
    const std::vector<YarnSegment>& segments,
    const std::vector<Vec3>& positions,
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
        Vec3 curr_pos = positions[i];
        Vec3 next_pos = (i < positions.size() - 1) ? positions[i + 1] : positions[i];
        geom.apex = calculate_apex_position(curr_pos, child_positions,
                                            effective_loop_height, yarn_compressed_diameter);

        // Apply shape modifiers to apex
        geom.apex.x += geom.shape.apex_lean_x;
        geom.apex.y = curr_pos.y + (geom.apex.y - curr_pos.y) * geom.shape.apex_height_factor * geom.shape.height_multiplier;

        // 4. Entry/exit at base positions
        geom.apex_entry = curr_pos;
        geom.apex_exit = next_pos;

        geom.pass_through_end = curr_pos;
        geom.pass_through_dir = safe_normalized(geom.apex - curr_pos, Vec3(0, 1, 0));

        // 5. Calculate clearance radius
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

        // Offset parent loop entry/exit: even mix of fabric normal and travel direction
        Vec3 normal_entry = compute_fabric_normal(positions, i, z_sign);
        Vec3 normal_exit = compute_fabric_normal(positions, i + 1 < positions.size() ? i + 1 : i, z_sign);
        Vec3 offset_dir_entry = safe_normalized(normal_entry + parent_travel);
        Vec3 offset_dir_exit = safe_normalized(normal_exit + parent_travel);
        geom.apex_entry = geom.apex_entry + offset_dir_entry * yarn_compressed_radius;
        geom.apex_exit = geom.apex_exit + offset_dir_exit * yarn_compressed_radius;

        // Count total crossover slots needed:
        // loop-forming children need 2 (entry + exit), others need 1
        int total_slots = 0;
        for (SegmentId child_id : child_ids) {
            total_slots += segments[child_id].forms_loop ? 2 : 1;
        }

        // Distribute slots clustered at the apex, where the loop opening is widest.
        // Slots are spread along the travel direction by yarn_compressed_diameter
        // for clearance, centered on the apex position.
        Vec3 fabric_normal = compute_fabric_normal(positions, i, z_sign);

        for (int s = 0; s < total_slots; ++s) {
            float offset = (static_cast<float>(s) - (total_slots - 1) / 2.0f)
                         * yarn_compressed_diameter;

            CrossoverSlot slot;
            slot.position = geom.apex + parent_travel * offset
                          + fabric_normal * yarn_compressed_radius;
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

    auto segments = build_curvature_safe_hermite_segments(
        start, start_dir, end, end_dir, state.max_curvature);
    for (auto& seg : segments) {
        add_curve_if_valid(state, segment_spline, seg, description, on_curve_added);
    }
}

void build_surface_guided_loop(
    GeometryBuildState& state,
    BezierSpline& segment_spline,
    const Vec3& entry,
    const Vec3& apex,
    const Vec3& exit_pt,
    float z_bulge,
    const CurveAddedCallback& on_curve_added) {

    // Travel direction: use entry→exit if they differ, otherwise use entry→apex projected horizontal
    Vec3 entry_to_exit = exit_pt - entry;
    Vec3 travel_dir;
    if (entry_to_exit.length() > 1e-5f) {
        travel_dir = safe_normalized(entry_to_exit);
    } else {
        // Entry ≈ exit: loop is a vertical arch at the crossing point.
        // Use the horizontal component of entry→apex as the travel direction.
        Vec3 to_apex = apex - entry;
        Vec3 horizontal_to_apex = Vec3(to_apex.x, 0.0f, to_apex.z);
        travel_dir = safe_normalized(horizontal_to_apex, Vec3(1.0f, 0.0f, 0.0f));
    }

    // Tangent lean: even mix of Z (fabric normal) and travel direction.
    // The passthrough handles the pure front/back crossing; the loop lean
    // adds both Z depth and horizontal spread to avoid overlapping with it.
    Vec3 z_component = Vec3(0.0f, 0.0f, z_bulge);
    Vec3 x_component = travel_dir * z_bulge;
    Vec3 lean = (z_component + x_component) * 0.5f;
    Vec3 entry_dir = safe_normalized(travel_dir + lean * 0.5f, travel_dir);
    Vec3 apex_dir = travel_dir;
    Vec3 exit_dir = safe_normalized(travel_dir - lean * 0.5f, travel_dir);

    // Curve 1: entry -> apex (rising half with front-side Z bulge)
    auto rise_segs = build_curvature_safe_hermite_segments(
        entry, entry_dir, apex, apex_dir, state.max_curvature);
    for (auto& seg : rise_segs) {
        add_curve_if_valid(state, segment_spline, seg, "loop_rise", on_curve_added);
    }

    // Curve 2: apex -> exit (falling half with back-side Z bulge)
    auto fall_segs = build_curvature_safe_hermite_segments(
        apex, apex_dir, exit_pt, exit_dir, state.max_curvature);
    for (auto& seg : fall_segs) {
        add_curve_if_valid(state, segment_spline, seg, "loop_fall", on_curve_added);
    }
}

void build_surface_guided_loop_with_crossings(
    GeometryBuildState& state,
    BezierSpline& segment_spline,
    const Vec3& entry,
    const Vec3& apex,
    const Vec3& exit_pt,
    float z_bulge,
    const std::vector<CrossoverSlot>& crossover_slots,
    const CurveAddedCallback& on_curve_added) {

    // Travel direction
    Vec3 entry_to_exit = exit_pt - entry;
    Vec3 travel_dir;
    if (entry_to_exit.length() > 1e-5f) {
        travel_dir = safe_normalized(entry_to_exit);
    } else {
        Vec3 to_apex = apex - entry;
        Vec3 horizontal_to_apex = Vec3(to_apex.x, 0.0f, to_apex.z);
        travel_dir = safe_normalized(horizontal_to_apex, Vec3(1.0f, 0.0f, 0.0f));
    }

    // Z lean for entry/exit tangents
    Vec3 z_component = Vec3(0.0f, 0.0f, z_bulge);
    Vec3 x_component = travel_dir * z_bulge;
    Vec3 lean = (z_component + x_component) * 0.5f;

    // Sort slots by projection onto travel direction for correct ordering
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

    // Build waypoints: entry → sorted slot positions → exit
    // Slots are clustered at the apex, so they replace the explicit apex waypoint.
    std::vector<Vec3> waypoints;
    std::vector<std::string> names;

    waypoints.push_back(entry);
    names.push_back("loop_rise");

    for (const auto& sp : sorted) {
        waypoints.push_back(crossover_slots[sp.index].position);
        names.push_back("loop_crossing");
    }

    waypoints.push_back(exit_pt);

    // Compute tangent directions: Z-lean at entry/exit, travel_dir at slots
    int n = static_cast<int>(waypoints.size());
    std::vector<Vec3> dirs(n);
    dirs[0] = safe_normalized(travel_dir + lean * 0.5f, travel_dir);
    dirs[n - 1] = safe_normalized(travel_dir - lean * 0.5f, travel_dir);

    for (int i = 1; i < n - 1; ++i) {
        // Catmull-Rom: chord from prev to next
        dirs[i] = safe_normalized(waypoints[i + 1] - waypoints[i - 1], travel_dir);
    }

    // Build curve segments between consecutive waypoints
    for (int i = 0; i < n - 1; ++i) {
        auto segs = build_curvature_safe_hermite_segments(
            waypoints[i], dirs[i], waypoints[i + 1], dirs[i + 1], state.max_curvature);
        for (auto& s : segs) {
            add_curve_if_valid(state, segment_spline, s, names[i], on_curve_added);
        }
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

void initialize_running_spline(
    GeometryBuildState& state,
    const Vec3& start_pos,
    const Vec3& first_target) {

    Vec3 initial_dir = safe_normalized(first_target - start_pos);

    // Tail length: enough runway for smooth transitions
    float min_bend_radius = 1.0f / state.max_curvature;
    float init_length = std::max(state.effective_stitch_width, min_bend_radius * 3.0f);
    auto init_seg = build_curvature_safe_hermite(
        start_pos - initial_dir * init_length, initial_dir,
        start_pos, initial_dir,
        state.max_curvature);
    state.running_spline.add_segment(init_seg);
}

}  // namespace yarnpath
