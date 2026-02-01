#include "geometry_builder.hpp"
#include "logging.hpp"
#include <cmath>
#include <algorithm>

namespace yarnpath {

// Helper struct to track state during geometry building
struct GeometryBuildState {
    BezierSpline running_spline;
    float max_curvature;
    float yarn_compressed_radius;       // Radius of the yarn
    float yarn_compressed_diameter;     // Diameter = 2 * compressed_radius, minimum distance between yarn centers
    
    GeometryBuildState(const YarnProperties& yarn)
        : max_curvature(yarn.max_curvature())
        , yarn_compressed_radius(yarn.compressed_radius)
        , yarn_compressed_diameter(yarn.compressed_radius * 2.0f)
    {}
};

// Callback type for reporting each curve as it's added
// Parameters: description of the curve, the running spline after adding
using CurveAddedCallback = std::function<void(const std::string& description)>;

// Parameters for varying loop shape based on topology
struct LoopShapeParams {
    float z_bulge_factor = 1.0f;      // How far loop crosses in Z (affects depth)
    float apex_lean_x = 0.0f;          // X-offset for twist/lean in decreases
    float apex_height_factor = 1.0f;   // Multiplier for apex height
    bool symmetric_exit = true;        // Derived from orientation (Front=true, Back=false)
    float entry_tangent_scale = 1.0f;  // Adjust entry curve sharpness (unused for now)
};

// Derive loop shape parameters from segment topology
static LoopShapeParams get_loop_shape_params(
    const YarnSegment& segment,
    float yarn_compressed_radius) {

    LoopShapeParams params;

    // Orientation-based variation
    if (segment.orientation == YarnSegment::LoopOrientation::Front) {
        params.z_bulge_factor = 1.0f;     // Standard forward crossing (knit)
        params.symmetric_exit = true;      // Knit exits symmetrically
    } else if (segment.orientation == YarnSegment::LoopOrientation::Back) {
        params.z_bulge_factor = -0.8f;    // Backward crossing (purl)
        params.symmetric_exit = false;    // Purl exits asymmetrically (yarn wraps around)
    } else {  // Neutral
        params.z_bulge_factor = 0.5f;     // Shallow crossing (cast-on, yarn-over)
        params.apex_height_factor = 0.8f; // Lower apex
        params.symmetric_exit = true;      // Neutral loops exit symmetrically
    }

    // WrapDirection-based lean
    if (segment.wrap_direction == YarnSegment::WrapDirection::Clockwise) {
        params.apex_lean_x = yarn_compressed_radius * 0.5f;  // Right lean (K2tog, M1R)
    } else if (segment.wrap_direction == YarnSegment::WrapDirection::CounterClockwise) {
        params.apex_lean_x = -yarn_compressed_radius * 0.5f; // Left lean (SSK, S2KP, M1L)
    }

    // WorkType-based depth
    if (segment.work_type == YarnSegment::WorkType::Transferred) {
        params.z_bulge_factor *= 0.3f;    // Very shallow (slip stitch)
        params.apex_height_factor = 0.6f; // Much lower apex
    } else if (segment.work_type == YarnSegment::WorkType::Created) {
        params.entry_tangent_scale = 0.7f; // Gentler entry (not yet applied)
    }

    return params;
}

// Calculate apex position from child positions or default height
// yarn_compressed_diameter is the minimum clearance needed between yarn centers
static Vec3 calculate_apex_position(
    const Vec3& curr_pos,
    const std::vector<Vec3>& child_positions,
    float yarn_compressed_diameter) {
    
    if (!child_positions.empty()) {
        Vec3 apex = Vec3::zero();
        for (const auto& cp : child_positions) {
            apex += cp;
        }
        apex = apex * (1.0f / static_cast<float>(child_positions.size()));
        apex.y += yarn_compressed_diameter;  // Full diameter above children so yarns don't touch
        return apex;
    } else {
        // No children - small loop above current position
        float loop_height = yarn_compressed_diameter;  // One diameter height
        return curr_pos + Vec3(0.0f, loop_height, 0.0f);
    }
}

// Calculate average position of child segments
static Vec3 calculate_avg_child_position(const std::vector<Vec3>& child_positions) {
    Vec3 avg = Vec3::zero();
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
            state.yarn_compressed_diameter,  // Diameter clearance to prevent overlap
            state.max_curvature);
        for (auto& c : up_curve) {
            state.running_spline.add_segment(c);
            segment_spline.add_segment(c);
            if (on_curve_added) on_curve_added("up_leg (with clearance)");
        }
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

// Build the apex crossover (entry -> apex -> exit, curving around children)
// The crossover goes up to the apex and then down, maintaining clearance from children
// Note: apex_entry is where the running_spline currently ends (from build_loop_up_leg)
static void build_apex_crossover(
    GeometryBuildState& state,
    BezierSpline& segment_spline,
    const Vec3& apex,
    const Vec3& apex_exit,
    const std::vector<Vec3>& child_positions,
    const CurveAddedCallback& on_curve_added) {
    
    // Calculate clearance point - the children pass through below the apex
    Vec3 clearance_point;
    if (!child_positions.empty()) {
        clearance_point = calculate_avg_child_position(child_positions);
    } else {
        // Default to below the apex by diameter
        clearance_point = apex - Vec3(0.0f, state.yarn_compressed_diameter, 0.0f);
    }
    
    float z_sign = apex_exit.z - apex.z;
    
    // First curve: apex_entry -> apex (going up and crossing Z)
    // Target direction at apex is horizontal, crossing Z
    auto apex_curve1 = create_continuation_segment_with_clearance(
        state.running_spline,
        apex,
        Vec3(0.0f, 0.0f, -z_sign),  // At apex, moving across Z
        clearance_point,
        state.yarn_compressed_diameter,
        state.max_curvature);
    for (auto& c : apex_curve1) {
        state.running_spline.add_segment(c);
        segment_spline.add_segment(c);
        if (on_curve_added) on_curve_added("apex_up");
    }
    
    // Second curve: apex -> apex_exit (going down after crossing Z)
    auto apex_curve2 = create_continuation_segment_with_clearance(
        state.running_spline,
        apex_exit,
        Vec3(0.0f, -1.0f, 0.0f),  // Exiting downward
        clearance_point,
        state.yarn_compressed_diameter,
        state.max_curvature);
    for (auto& c : apex_curve2) {
        state.running_spline.add_segment(c);
        segment_spline.add_segment(c);
        if (on_curve_added) on_curve_added("apex_down");
    }
}

// Build the downward leg of a loop (from apex exit to next entry)
static void build_loop_down_leg(
    GeometryBuildState& state,
    BezierSpline& segment_spline,
    const Vec3& curr_pos,
    const Vec3& travel_dir,
    const std::vector<Vec3>& child_positions,
    const CurveAddedCallback& on_curve_added) {
    
    if (!child_positions.empty()) {
        Vec3 avg_child = calculate_avg_child_position(child_positions);

        auto down_curve = create_continuation_segment_with_clearance(
            state.running_spline,
            curr_pos,
            travel_dir,
            avg_child,
            state.yarn_compressed_diameter,  // Diameter clearance to prevent overlap
            state.max_curvature);
        for (auto& c : down_curve) {
            state.running_spline.add_segment(c);
            segment_spline.add_segment(c);
            if (on_curve_added) on_curve_added("down_leg (with clearance)");
        }
    } else {
        auto down_curve = create_continuation_segment(
            state.running_spline,
            curr_pos,
            travel_dir,
            state.max_curvature);
        state.running_spline.add_segment(down_curve);
        segment_spline.add_segment(down_curve);
        if (on_curve_added) on_curve_added("down_leg");
    }
}

// Build the pass-through segment to the next entry point
static void build_pass_through(
    GeometryBuildState& state,
    BezierSpline& segment_spline,
    const Vec3& next_entry,
    const Vec3& next_entry_dir,
    const CurveAddedCallback& on_curve_added) {
    
    auto pass_curve = create_continuation_segment(
        state.running_spline,
        next_entry,
        next_entry_dir,
        state.max_curvature);
    state.running_spline.add_segment(pass_curve);
    segment_spline.add_segment(pass_curve);
    if (on_curve_added) on_curve_added("pass_through");
}

// Build a complete loop segment curve
static BezierSpline build_loop_segment(
    GeometryBuildState& state,
    const Vec3& curr_pos,
    const Vec3& next_pos,
    const std::vector<Vec3>& child_positions,
    const YarnSegment& segment,
    const CurveAddedCallback& on_curve_added) {

    BezierSpline segment_spline;

    // Get topology-based shape parameters
    LoopShapeParams shape = get_loop_shape_params(segment, state.yarn_compressed_radius);

    // Calculate apex position (using diameter for proper clearance)
    Vec3 apex = calculate_apex_position(curr_pos, child_positions, state.yarn_compressed_diameter);

    // Calculate the X offset for apex entry and exit. The apex is centered.
    // We need to consider the current and next positions to determine the direction that
    // we are going. The entry should be closer to the next position and the exit should be
    // behind the current position, according to the direction of travel.
    Vec3 to_next = next_pos - curr_pos;
    Vec3 travel_dir = (to_next.length() > 0.0001f) ? to_next.normalized() : Vec3(1.0f, 0.0f, 0.0f);
    float apex_offset_x = travel_dir.x * state.yarn_compressed_radius;

    // Entry/exit points offset in Z for the crossover
    // Apply topology-based shape variation
    float z_bulge = state.yarn_compressed_diameter * shape.z_bulge_factor;
    float apex_x_lean = shape.apex_lean_x;

    Vec3 apex_entry = apex + Vec3(
        apex_offset_x + apex_x_lean,
        -state.yarn_compressed_diameter * shape.apex_height_factor,
        z_bulge
    );

    Vec3 apex_exit = apex + Vec3(
        shape.symmetric_exit ? -apex_offset_x + apex_x_lean : -apex_offset_x * 0.7f + apex_x_lean,
        -state.yarn_compressed_diameter * shape.apex_height_factor,
        shape.symmetric_exit ? -z_bulge : -z_bulge * 0.8f
    );
    
    Vec3 pass_through_end = curr_pos + Vec3(apex_offset_x, 0.0f, z_bulge);
    Vec3 pass_through_dir = (apex_entry - pass_through_end).normalized();

    build_pass_through(state, segment_spline, pass_through_end, pass_through_dir, on_curve_added);

    // Build upward leg
    build_loop_up_leg(state, segment_spline, apex_entry, child_positions, on_curve_added);
    
    // Build apex crossover (entry -> apex -> exit)
    build_apex_crossover(state, segment_spline, apex, apex_exit, child_positions, on_curve_added);
    
    // Calculate next entry point - offset by yarn_compressed_diameter in Y and Z
    Vec3 next_entry = next_pos + Vec3(0.0f, -state.yarn_compressed_diameter, z_bulge);
    Vec3 next_entry_dir = (next_pos - apex_exit).normalized();
    
    // Build downward leg
    build_loop_down_leg(state, segment_spline, next_entry, next_entry_dir, child_positions, on_curve_added);

    return segment_spline;
}

// Build a simple connector segment (non-loop or end-of-row)
static BezierSpline build_connector_segment(
    GeometryBuildState& state,
    const Vec3& curr_pos,
    const Vec3& next_pos,
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
                        min_y = std::min(min_y, child_pos.y - yarn.compressed_radius * 2);
                    }
                }
            }
            if (min_y > base_pos.y) {
                yarnpath::logging::get_logger()->debug(
                    "get_segment_base_position: segment {} has no parents, adjusting base position from {} to {}",
                    segment_id, base_pos.y, min_y - yarn.compressed_radius);
                base_pos.y = min_y - yarn.compressed_radius;
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
            pos_opt = Vec3::zero();
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

        // Create callback for reporting each curve as it's added
        CurveAddedCallback on_curve_added = nullptr;
        if (callback_ptr) {
            on_curve_added = [&](const std::string& description) {
                (*callback_ptr)(seg_id, description, state.running_spline);
            };
        }

        // Build curve for this segment
        if (seg.forms_loop) {
            geom.curve = build_loop_segment(state, curr_pos, next_pos, child_positions, seg, on_curve_added);
        } else {
            geom.curve = build_connector_segment(state, curr_pos, next_pos, on_curve_added);
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
