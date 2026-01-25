#include "geometry_builder.hpp"
#include "logging.hpp"
#include <cmath>
#include <algorithm>
#include <limits>

namespace yarnpath {

// ============================================================================
// Context creation
// ============================================================================

GeometryContext GeometryContext::create(const YarnPath& yarn_path,
                                         const YarnProperties& yarn,
                                         const Gauge& gauge) {
    return GeometryContext{
        .yarn_path = yarn_path,
        .yarn = yarn,
        .gauge = gauge,
        .loop_dims = LoopDimensions::calculate(yarn, gauge)
    };
}

// ============================================================================
// Helper functions
// ============================================================================

BezierSpline make_connector(const YarnProperties& yarn,
                            const Vec3& from, const Vec3& from_dir,
                            const Vec3& to, const Vec3& to_dir) {
    BezierSpline spline;

    Vec3 chord = to - from;
    float dist = chord.length();

    if (dist < 0.0001f) {
        // Degenerate case: start and end are the same
        spline.add_segment(CubicBezier(from, from, to, to));
        return spline;
    }

    // Scale tangents based on distance and curvature limit
    float max_k = yarn.max_curvature();
    float tangent_scale = std::min(dist * 0.4f, 1.0f / max_k);

    Vec3 t0 = from_dir.normalized() * tangent_scale;
    Vec3 t1 = to_dir.normalized() * tangent_scale;

    spline.add_segment(CubicBezier::from_hermite(from, t0, to, -t1));

    return spline;
}

PhysicalLoop create_loop_at_position(const GeometryContext& ctx,
                                      const Vec3& position) {
    return PhysicalLoop::from_properties(ctx.yarn, ctx.gauge, position);
}

PassThroughResult pass_through_loop(const GeometryContext& ctx,
                                     const PhysicalLoop& loop,
                                     const Vec3& from, const Vec3& from_dir) {
    PassThroughResult result;

    // The yarn passes through the parent loop at its apex (top).
    // We just record the apex position - the actual curve will be created
    // by the caller who knows both the start and end points.
    
    // The pass-through point is at the apex, in the middle of the opening
    result.exit_pos = loop.apex_point;
    result.exit_dir = Vec3(0.0f, 0.0f, 1.0f);  // Through direction is +Z
    
    // No spline segments here - the caller will create the smooth curve
    return result;
}

// ============================================================================
// Segment building functions
// ============================================================================

SegmentBuildResult build_cast_on_loop(const GeometryContext& ctx,
                                       const YarnState& state,
                                       SegmentId seg_id) {
    SegmentBuildResult result;
    result.new_state = state;

    // For cast-on, the loop forms at the current position
    // Position the loop center at the current Y level
    Vec3 loop_position = state.position + Vec3(ctx.gauge.stitch_width(), 0.0f, 0.0f);

    // Create the physical loop
    PhysicalLoop loop = create_loop_at_position(ctx, loop_position);

    // Build the spline that forms this loop
    BezierSpline spline;

    float needle_radius = ctx.gauge.needle_diameter * 0.5f;
    float wrap_radius = needle_radius + ctx.yarn.radius;
    float t = wrap_radius * 1.5f;

    // For cast-on, we need to wrap around the needle from the current position
    // to the loop entry at the front (-Z)
    // 
    // Current position is typically at Y=center level, Z=0 or coming from the left
    // Entry point is at front (Z = -wrap_radius)
    // 
    // Create a cylindrical wrap: pos → bottom → front (entry)
    
    Vec3 bottom_point = Vec3(
        loop.center.x - loop.loop_width * 0.5f,  // Left side where entry is
        loop.center.y - wrap_radius,              // Bottom of needle
        0.0f                                       // Z = 0 at bottom
    );
    
    // Tangent at bottom points toward front (-Z)
    Vec3 bottom_tangent = Vec3(0.0f, 0.0f, -1.0f);
    
    // First curve: pos → bottom (wrap down and around)
    spline.add_segment(CubicBezier::from_hermite(
        state.position, state.direction.normalized() * t,
        bottom_point, bottom_tangent * t
    ));
    
    // Second curve: bottom → entry (continue around to front)
    // Entry tangent points up (+Y)
    spline.add_segment(CubicBezier::from_hermite(
        bottom_point, bottom_tangent * t,
        loop.entry_point, loop.entry_tangent * t
    ));

    // Add the loop shape itself (front → top → back)
    for (const auto& seg : loop.shape.segments()) {
        spline.add_segment(seg);
    }

    // Build the result
    result.geometry.segment_id = seg_id;
    result.geometry.curve = std::move(spline);
    result.geometry.arc_length = result.geometry.curve.total_arc_length();
    result.geometry.max_curvature = result.geometry.curve.max_curvature();

    // Update state: yarn exits from loop exit point with the loop's exit tangent
    result.new_state.position = loop.exit_point;
    result.new_state.direction = loop.exit_tangent;
    result.new_state.formed_loops[seg_id] = loop;

    return result;
}

SegmentBuildResult build_through_and_form_loop(const GeometryContext& ctx,
                                                const YarnState& state,
                                                SegmentId seg_id,
                                                const std::vector<SegmentId>& through) {
    SegmentBuildResult result;
    result.new_state = state;

    BezierSpline spline;
    Vec3 pos = state.position;
    Vec3 dir = state.direction;

    // Track the parent loop with the highest apex for proper vertical placement
    const PhysicalLoop* highest_parent = nullptr;
    float highest_apex_y = -std::numeric_limits<float>::max();
    float parent_x = state.position.x;

    // Find the parent loops we need to pass through
    std::vector<const PhysicalLoop*> parent_loops;
    for (SegmentId parent_id : through) {
        auto it = state.formed_loops.find(parent_id);
        if (it != state.formed_loops.end()) {
            const PhysicalLoop& parent_loop = it->second;
            parent_loops.push_back(&parent_loop);

            if (parent_loop.apex_point.y > highest_apex_y) {
                highest_apex_y = parent_loop.apex_point.y;
                highest_parent = &parent_loop;
            }
            parent_x = parent_loop.center.x;
        }
    }

    // Calculate the new loop's center position
    // The new loop's entry point is at Y = center.y (needle center height)
    // For proper interlocking, position the child so its entry is just above the parent's apex
    float needle_radius = ctx.gauge.needle_diameter * 0.5f;
    float wrap_radius = needle_radius + ctx.yarn.radius;
    float yarn_diameter = 2.0f * ctx.yarn.radius;
    
    float new_center_y;
    if (highest_parent != nullptr) {
        // Child center at parent's apex + clearance
        new_center_y = highest_parent->apex_point.y + yarn_diameter;
    } else {
        new_center_y = state.position.y + ctx.gauge.row_height();
    }
    
    // Position the new loop at the same X as the parent loop center
    // to avoid X skew between rows
    Vec3 loop_position = Vec3(parent_x, new_center_y, 0.0f);
    PhysicalLoop new_loop = create_loop_at_position(ctx, loop_position);

    // Detect if we're moving up to a new row (Y increasing significantly)
    // This happens at the end of a row when we need to wrap around before going to the next row.
    // We detect this by checking if there's a big vertical gap between the current position
    // and the parent's apex - not just if the new loop is above the current pos.
    // Normal stitches go through the parent apex, which is only wrap_radius above the center.
    bool is_row_turn = false;
    if (highest_parent != nullptr) {
        // Row turn if we're significantly below the parent's apex
        // (i.e., not just coming from a loop at the same level)
        float vertical_gap = highest_parent->apex_point.y - pos.y;
        is_row_turn = vertical_gap > wrap_radius * 1.5f;
    } else {
        // No parent - check if going up significantly
        is_row_turn = (new_center_y - pos.y) > wrap_radius * 2.0f;
    }
    
    if (is_row_turn) {
        // At end of row, we need to complete the wrap around the needle
        // The yarn exits at back (+Z) going down, needs to continue around:
        // back (+Z) → bottom (Z=0) → front (-Z)
        // This completes the loop shape that was cut short
        
        // Use tangent scale matching the wrap radius
        float t = wrap_radius * 1.5f;
        
        // The exit point is at the back of the needle at the current Y level
        // We need to wrap down to the bottom, then to the front
        
        // Bottom of the wrap
        Vec3 bottom_point = Vec3(
            pos.x,
            pos.y - wrap_radius,  // Bottom of needle
            0.0f                   // Z = 0 at bottom
        );
        
        // Front of the wrap (where we'd normally enter a loop)
        Vec3 front_point = Vec3(
            pos.x,
            pos.y,                 // Same Y as exit
            -wrap_radius           // Front of needle
        );
        
        // Tangent at bottom points toward -Z (continuing around)
        Vec3 bottom_tangent = Vec3(0.0f, 0.0f, -1.0f);
        
        // Tangent at front points up (+Y, continuing around)
        Vec3 front_tangent = Vec3(0.0f, 1.0f, 0.0f);
        
        // First curve: exit (back) → bottom
        // dir points down (-Y) from the loop exit
        spline.add_segment(CubicBezier::from_hermite(
            pos, dir.normalized() * t,
            bottom_point, bottom_tangent * t
        ));
        
        // Second curve: bottom → front
        spline.add_segment(CubicBezier::from_hermite(
            bottom_point, bottom_tangent * t,
            front_point, front_tangent * t
        ));
        
        // Update pos/dir - now at front, going up
        pos = front_point;
        dir = front_tangent;
    }

    // Create the path from previous loop exit through parent's apex to new loop entry.
    //
    // Previous loop exits at the BACK (+Z), new loop enters at the FRONT (-Z)
    // So we need to pass through the parent's apex, which is at Z=0
    // This creates a smooth curve: back (+Z) → apex (Z=0) → front (-Z)
    //
    // For a natural wrap-around-needle look, the tangents should follow
    // the circular arc of the cylinder, not just point straight in Z.
    
    if (highest_parent != nullptr) {
        Vec3 apex = highest_parent->apex_point;
        Vec3 entry = new_loop.entry_point;
        
        // For a cylindrical wrap, use tangent scales based on wrap_radius
        // This creates arcs that look like they go around the needle
        float t = wrap_radius * 1.5f;
        
        // First curve: pos → apex
        // The tangent direction depends on where we're coming from:
        // - Normal case: from back (+Z), arriving tangent points up and toward front
        // - Row turn case: from front (-Z), arriving tangent points up and toward back
        Vec3 arriving_at_apex;
        if (is_row_turn) {
            // After row turn, we're at front going up, apex is at Z=0
            // Tangent should curve up and toward back (+Z)
            arriving_at_apex = Vec3(0.0f, 0.5f, 1.0f).normalized();
        } else {
            // Normal case: from back, curving up and toward front
            arriving_at_apex = Vec3(0.0f, 0.5f, -1.0f).normalized();
        }
        
        spline.add_segment(CubicBezier::from_hermite(
            pos, dir.normalized() * t,
            apex, arriving_at_apex * t
        ));
        
        // Second curve: apex → entry
        // Entry is ABOVE the apex (child loop is above parent)
        // So we need to go UP (+Y) and continue toward front (-Z)
        Vec3 leaving_apex = Vec3(0.0f, 1.0f, -1.0f).normalized();
        
        spline.add_segment(CubicBezier::from_hermite(
            apex, leaving_apex * t,
            entry, new_loop.entry_tangent * t
        ));
    } else {
        // No parent (yarn over) - wrap around the needle to the new loop entry
        // Similar to cast-on, create a cylindrical wrap
        float t = wrap_radius * 1.5f;
        
        // Bottom point of the wrap
        Vec3 bottom_point = Vec3(
            new_loop.center.x - new_loop.loop_width * 0.5f,  // Left side where entry is
            new_loop.center.y - wrap_radius,                  // Bottom of needle
            0.0f                                               // Z = 0 at bottom
        );
        
        // Tangent at bottom points toward front (-Z)
        Vec3 bottom_tangent = Vec3(0.0f, 0.0f, -1.0f);
        
        // First curve: pos → bottom (wrap down and around)
        spline.add_segment(CubicBezier::from_hermite(
            pos, dir.normalized() * t,
            bottom_point, bottom_tangent * t
        ));
        
        // Second curve: bottom → entry (continue around to front)
        spline.add_segment(CubicBezier::from_hermite(
            bottom_point, bottom_tangent * t,
            new_loop.entry_point, new_loop.entry_tangent * t
        ));
    }

    // Add the loop shape
    for (const auto& seg : new_loop.shape.segments()) {
        spline.add_segment(seg);
    }

    result.geometry.segment_id = seg_id;
    result.geometry.curve = std::move(spline);
    result.geometry.arc_length = result.geometry.curve.total_arc_length();
    result.geometry.max_curvature = result.geometry.curve.max_curvature();

    // Update state: yarn exits from loop exit point with the loop's exit tangent
    result.new_state.position = new_loop.exit_point;
    result.new_state.direction = new_loop.exit_tangent;
    result.new_state.formed_loops[seg_id] = new_loop;

    return result;
}

SegmentBuildResult build_through_only(const GeometryContext& ctx,
                                       const YarnState& state,
                                       SegmentId seg_id,
                                       const std::vector<SegmentId>& through) {
    SegmentBuildResult result;
    result.new_state = state;

    BezierSpline spline;
    Vec3 pos = state.position;
    Vec3 dir = state.direction;

    // Pass through each parent loop
    for (SegmentId parent_id : through) {
        auto it = state.formed_loops.find(parent_id);
        if (it != state.formed_loops.end()) {
            const PhysicalLoop& parent_loop = it->second;

            PassThroughResult pass_result = pass_through_loop(ctx, parent_loop, pos, dir);

            for (const auto& seg : pass_result.spline.segments()) {
                spline.add_segment(seg);
            }

            pos = pass_result.exit_pos;
            dir = pass_result.exit_dir;
        }
    }

    result.geometry.segment_id = seg_id;
    result.geometry.curve = std::move(spline);
    result.geometry.arc_length = result.geometry.curve.total_arc_length();
    result.geometry.max_curvature = result.geometry.curve.max_curvature();

    // Update state
    result.new_state.position = pos;
    result.new_state.direction = dir;

    return result;
}

SegmentBuildResult build_connector_segment(const GeometryContext& ctx,
                                            const YarnState& state,
                                            SegmentId seg_id) {
    SegmentBuildResult result;
    result.new_state = state;

    // Simple segment that just moves forward a bit
    Vec3 target = state.position + state.direction * ctx.gauge.stitch_width() * 0.5f;

    BezierSpline spline = make_connector(ctx.yarn, state.position, state.direction,
                                          target, state.direction);

    result.geometry.segment_id = seg_id;
    result.geometry.curve = std::move(spline);
    result.geometry.arc_length = result.geometry.curve.total_arc_length();
    result.geometry.max_curvature = result.geometry.curve.max_curvature();

    result.new_state.position = target;
    // Direction stays the same

    return result;
}

SegmentBuildResult build_segment(const GeometryContext& ctx,
                                  const YarnState& state,
                                  SegmentId seg_id,
                                  const YarnSegment& seg) {
    // Dispatch based on segment type
    if (seg.forms_loop) {
        if (seg.through.empty()) {
            // Cast-on: forms a loop without passing through any parent
            return build_cast_on_loop(ctx, state, seg_id);
        } else {
            // Normal stitch: passes through parent(s) and forms a new loop
            return build_through_and_form_loop(ctx, state, seg_id, seg.through);
        }
    } else {
        if (!seg.through.empty()) {
            // Passes through loops but doesn't form one (e.g., bind-off)
            return build_through_only(ctx, state, seg_id, seg.through);
        } else {
            // Simple connector (rare - just yarn moving between operations)
            return build_connector_segment(ctx, state, seg_id);
        }
    }
}

// ============================================================================
// Main entry point
// ============================================================================

GeometryPath build_geometry(const YarnPath& yarn_path,
                            const YarnProperties& yarn,
                            const Gauge& gauge) {
    auto log = yarnpath::logging::get_logger();
    log->info("build_geometry: building geometry for {} segments", yarn_path.segment_count());

    // Create the context (immutable during build)
    GeometryContext ctx = GeometryContext::create(yarn_path, yarn, gauge);

    // Initialize the yarn state
    YarnState state{
        .position = Vec3(0.0f, 0.0f, 0.0f),
        .direction = Vec3(1.0f, 0.0f, 0.0f),
        .formed_loops = {}
    };

    // Build geometry for each segment, threading state through
    GeometryPath result;
    const auto& segments = yarn_path.segments();

    for (size_t i = 0; i < segments.size(); ++i) {
        SegmentId seg_id = static_cast<SegmentId>(i);
        SegmentBuildResult seg_result = build_segment(ctx, state, seg_id, segments[i]);

        result.segments_.push_back(std::move(seg_result.geometry));
        state = std::move(seg_result.new_state);
    }

    log->info("build_geometry: built {} segment geometries", result.segments_.size());

    return result;
}

}  // namespace yarnpath
