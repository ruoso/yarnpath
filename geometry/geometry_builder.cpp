#include "geometry_builder.hpp"
#include "logging.hpp"
#include <cmath>
#include <algorithm>

namespace yarnpath {

// Helper struct to track state during geometry building
struct GeometryBuildState {
    BezierSpline running_spline;
    bool current_z_positive = true;
    float max_curvature;
    float yarn_radius;       // Radius of the yarn
    float yarn_diameter;     // Diameter = 2 * radius, minimum distance between yarn centers
    float wrap_offset;       // Offset for entry/exit points at apex
    
    GeometryBuildState(const YarnProperties& yarn)
        : max_curvature(yarn.max_curvature())
        , yarn_radius(yarn.radius)
        , yarn_diameter(yarn.radius * 2.0f)
        , wrap_offset(yarn.radius)  // Full radius for entry/exit separation
    {}
    
    float z_sign() const { return current_z_positive ? 1.0f : -1.0f; }
    void flip_z() { current_z_positive = !current_z_positive; }
};

// Callback type for reporting each curve as it's added
// Parameters: description of the curve, the running spline after adding
using CurveAddedCallback = std::function<void(const std::string& description)>;

// Calculate apex position from child positions or default height
// yarn_diameter is the minimum clearance needed between yarn centers
static Vec3 calculate_apex_position(
    const Vec3& curr_pos,
    const std::vector<Vec3>& child_positions,
    float yarn_diameter) {
    
    if (!child_positions.empty()) {
        Vec3 apex = vec3::zero();
        for (const auto& cp : child_positions) {
            apex += cp;
        }
        apex = apex * (1.0f / static_cast<float>(child_positions.size()));
        apex.y += yarn_diameter;  // Full diameter above children so yarns don't touch
        return apex;
    } else {
        // No children - small loop above current position
        float loop_height = yarn_diameter;  // One diameter height
        return curr_pos + Vec3(0.0f, loop_height, 0.0f);
    }
}

// Calculate average position of child segments
static Vec3 calculate_avg_child_position(const std::vector<Vec3>& child_positions) {
    Vec3 avg = vec3::zero();
    for (const auto& cp : child_positions) {
        avg += cp;
    }
    return avg * (1.0f / static_cast<float>(child_positions.size()));
}

// Build the upward leg of a loop (from current position to apex entry)
static void build_loop_up_leg(
    GeometryBuildState& state,
    BezierSpline& segment_spline,
    const Vec3& apex_entry,
    const std::vector<Vec3>& child_positions,
    const CurveAddedCallback& on_curve_added) {
    
    if (!child_positions.empty()) {
        // Use clearance to go around the child positions
        Vec3 avg_child = calculate_avg_child_position(child_positions);
        
        auto up_curve = create_continuation_segment_with_clearance(
            state.running_spline,
            apex_entry,
            Vec3(0.0f, 1.0f, 0.0f),  // Going up
            avg_child,
            state.yarn_diameter,  // Diameter clearance to prevent overlap
            state.max_curvature);
        state.running_spline.add_segment(up_curve);
        segment_spline.add_segment(up_curve);
        if (on_curve_added) on_curve_added("up_leg (with clearance)");
    } else {
        auto up_curve = create_continuation_segment(
            state.running_spline,
            apex_entry,
            Vec3(0.0f, 1.0f, 0.0f),  // Going up
            state.max_curvature);
        state.running_spline.add_segment(up_curve);
        segment_spline.add_segment(up_curve);
        if (on_curve_added) on_curve_added("up_leg");
    }
}

// Build the apex crossover (entry -> exit, curving around children)
// The crossover curves around the child segments that pass through this loop
static void build_apex_crossover(
    GeometryBuildState& state,
    BezierSpline& segment_spline,
    const Vec3& apex_entry,
    const Vec3& apex_exit,
    const std::vector<Vec3>& child_positions,
    const CurveAddedCallback& on_curve_added) {
    
    // Calculate clearance point - the children pass through below the apex
    Vec3 clearance_point;
    if (!child_positions.empty()) {
        clearance_point = calculate_avg_child_position(child_positions);
    } else {
        // Default to midpoint between entry and exit, offset down by diameter
        clearance_point = (apex_entry + apex_exit) * 0.5f - Vec3(0.0f, state.yarn_diameter, 0.0f);
    }
    
    // Single curve from entry to exit, arcing around the clearance point
    // Use yarn_diameter as clearance to prevent overlap
    auto apex_curve = create_continuation_segment_with_clearance(
        state.running_spline,
        apex_exit,
        Vec3(0.0f, -1.0f, 0.0f),  // Exiting downward
        clearance_point,
        state.yarn_diameter,
        state.max_curvature);
    state.running_spline.add_segment(apex_curve);
    segment_spline.add_segment(apex_curve);
    if (on_curve_added) on_curve_added("apex_crossover");
}

// Build the downward leg of a loop (from apex exit to next entry)
static void build_loop_down_leg(
    GeometryBuildState& state,
    BezierSpline& segment_spline,
    const Vec3& next_entry,
    const std::vector<Vec3>& child_positions,
    const CurveAddedCallback& on_curve_added) {
    
    if (!child_positions.empty()) {
        Vec3 avg_child = calculate_avg_child_position(child_positions);

        auto down_curve = create_continuation_segment_with_clearance(
            state.running_spline,
            next_entry,
            Vec3(0.0f, -1.0f, 0.0f),  // Going down
            avg_child,
            state.yarn_diameter,  // Diameter clearance to prevent overlap
            state.max_curvature);
        state.running_spline.add_segment(down_curve);
        segment_spline.add_segment(down_curve);
        if (on_curve_added) on_curve_added("down_leg (with clearance)");
    } else {
        auto down_curve = create_continuation_segment(
            state.running_spline,
            next_entry,
            Vec3(0.0f, -1.0f, 0.0f),  // Going down
            state.max_curvature);
        state.running_spline.add_segment(down_curve);
        segment_spline.add_segment(down_curve);
        if (on_curve_added) on_curve_added("down_leg");
    }
}

// Build a complete loop segment curve
static BezierSpline build_loop_segment(
    GeometryBuildState& state,
    const Vec3& curr_pos,
    const Vec3& next_pos,
    const std::vector<Vec3>& child_positions,
    const CurveAddedCallback& on_curve_added) {
    
    BezierSpline segment_spline;
    
    // Calculate apex position (using diameter for proper clearance)
    Vec3 apex = calculate_apex_position(curr_pos, child_positions, state.yarn_diameter);
    
    // Entry/exit points offset in Z for the crossover
    // Use wrap_offset (= yarn_radius) for separation
    float z_sign = state.z_sign();
    Vec3 apex_entry = apex + Vec3(0.0f, -state.wrap_offset, z_sign * state.wrap_offset);
    Vec3 apex_exit = apex + Vec3(0.0f, -state.wrap_offset, -z_sign * state.wrap_offset);
    
    // Build upward leg
    build_loop_up_leg(state, segment_spline, apex_entry, child_positions, on_curve_added);
    
    // Build apex crossover
    build_apex_crossover(state, segment_spline, apex_entry, apex_exit, child_positions, on_curve_added);
    
    // Calculate next entry point - offset by wrap_offset (yarn_radius) in Y and Z
    Vec3 next_entry = next_pos + Vec3(0.0f, state.wrap_offset, -z_sign * state.wrap_offset);
    
    // Build downward leg
    build_loop_down_leg(state, segment_spline, next_entry, child_positions, on_curve_added);
    
    // Flip Z for next segment after crossover
    state.flip_z();
    
    return segment_spline;
}

// Build a simple connector segment (non-loop or end-of-row)
static BezierSpline build_connector_segment(
    GeometryBuildState& state,
    const Vec3& curr_pos,
    const Vec3& next_pos,
    bool end_of_row,
    const CurveAddedCallback& on_curve_added) {
    
    BezierSpline segment_spline;
    
    Vec3 to_next = next_pos - curr_pos;
    Vec3 target_dir = (to_next.length() > 0.0001f) ? to_next.normalized() : Vec3(1.0f, 0.0f, 0.0f);

    auto connect_curve = create_continuation_segment(
        state.running_spline,
        next_pos,
        target_dir,
        state.max_curvature);
    state.running_spline.add_segment(connect_curve);
    segment_spline.add_segment(connect_curve);
    if (on_curve_added) on_curve_added("connector");

    if (end_of_row) {
        state.flip_z();
    }
    
    return segment_spline;
}

// Initialize the running spline with a starting segment
static void initialize_running_spline(
    GeometryBuildState& state,
    const std::vector<Vec3>& positions) {
    
    if (positions.empty()) return;
    
    Vec3 start_pos = positions[0];
    Vec3 initial_dir = (positions.size() > 1) 
        ? (positions[1] - positions[0]).normalized() 
        : Vec3(1.0f, 0.0f, 0.0f);
    
    // Create a minimal segment to establish the starting point and direction
    auto init_seg = CubicBezier::from_hermite(
        start_pos - initial_dir * 0.01f, initial_dir * 0.01f,
        start_pos, initial_dir * 0.01f);
    state.running_spline.add_segment(init_seg);
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

    // create a cache of which segments have parents
    std::vector<bool> segment_has_parent(segments.size(), false);
    for (size_t i = 0; i < segments.size(); ++i) {
        const auto& seg = segments[i];
        if (!seg.through.empty()) {
            segment_has_parent[i] = true;
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

    // Initialize geometry build state
    GeometryBuildState state(yarn);
    initialize_running_spline(state, positions);

    // Build geometry segment by segment
    for (size_t i = 0; i < segments.size(); ++i) {
        SegmentId seg_id = static_cast<SegmentId>(i);
        const auto& seg = segments[i];

        SegmentGeometry geom;
        geom.segment_id = seg_id;

        Vec3 curr_pos = positions[i];
        Vec3 next_pos = (i < positions.size() - 1) ? positions[i + 1] : positions[i];

        // Get child positions if this is a loop (segments that pass through this one)
        std::vector<Vec3> child_positions;
        auto it = loop_apex_positions.find(seg_id);
        if (it != loop_apex_positions.end()) {
            child_positions = it->second;
        }

        // Determine if this is end of row (next segment passes through this one)
        bool end_of_row = false;
        if (i < segments.size() - 1) {
            const auto& next_seg = segments[i + 1];
            if (std::find(next_seg.through.begin(), next_seg.through.end(), seg_id) != next_seg.through.end()) {
                end_of_row = true;
            }
        }

        // Create callback for reporting each curve as it's added
        CurveAddedCallback on_curve_added = nullptr;
        if (callback_ptr) {
            on_curve_added = [&](const std::string& description) {
                (*callback_ptr)(seg_id, description, state.running_spline);
            };
        }

        // Build curve for this segment
        if (seg.forms_loop && !end_of_row) {
            geom.curve = build_loop_segment(state, curr_pos, next_pos, child_positions, on_curve_added);
        } else {
            geom.curve = build_connector_segment(state, curr_pos, next_pos, end_of_row, on_curve_added);
        }

        // Calculate arc length and max curvature
        geom.arc_length = geom.curve.total_arc_length();
        geom.max_curvature = geom.curve.max_curvature();

        log->debug("  segment {}: forms_loop={}, children={}, arc_length={:.3f}, max_curvature={:.3f}, has_parent={}",
                   seg_id, seg.forms_loop, child_positions.size(),
                   geom.arc_length, geom.max_curvature,
                   segment_has_parent[seg_id] ? "true" : "false");

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
