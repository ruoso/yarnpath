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
        yarnpath::logging::get_logger()->debug("make_connector: degenerate case (dist={:.4f})", dist);
        spline.add_segment(CubicBezier(from, from, to, to));
        return spline;
    }

    // Scale tangents based on distance - use ~1/3 of distance for smooth curves
    // This gives good curvature control for typical 90° bends
    float tangent_scale = dist * 0.4f;

    Vec3 t0 = from_dir.normalized() * tangent_scale;
    Vec3 t1 = to_dir.normalized() * tangent_scale;

    auto bezier = CubicBezier::from_hermite(from, t0, to, -t1);
    float k = bezier.max_curvature();
    yarnpath::logging::get_logger()->debug("make_connector: dist={:.3f} tangent_scale={:.3f} max_curvature={:.3f}", 
                 dist, tangent_scale, k);
    spline.add_segment(bezier);

    return spline;
}

// Helper to create a smooth Bezier curve with distance-based tangent scaling
// Also considers the angle between tangents to ensure curvature control
CubicBezier make_smooth_curve(const Vec3& from, const Vec3& from_dir,
                               const Vec3& to, const Vec3& to_dir) {
    Vec3 chord = to - from;
    float dist = chord.length();
    
    // Calculate angle between tangents (0 = same direction, 1 = opposite)
    float cos_angle = from_dir.normalized().dot(to_dir.normalized());
    // For 180° turns, cos_angle = -1
    // For 90° turns, cos_angle = 0
    // For 0° turns, cos_angle = 1
    
    // Larger angles need longer tangents for smoother curves
    // angle_factor ranges from 1.0 (same dir) to 3.0 (opposite)
    float angle_factor = 1.0f + (1.0f - cos_angle);
    
    // Use generous tangent scaling for smooth curves
    float tangent_scale = std::max(dist * 0.6f * angle_factor, 2.0f);
    
    return CubicBezier::from_hermite(
        from, from_dir.normalized() * tangent_scale,
        to, to_dir.normalized() * tangent_scale
    );
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
    yarnpath::logging::get_logger()->debug("build_cast_on_loop: seg_id={} pos=({:.3f},{:.3f},{:.3f})",
                 seg_id, state.position.x, state.position.y, state.position.z);
    SegmentBuildResult result;
    result.new_state = state;

    // For cast-on, the loop forms at the current position
    // Position the loop center using loop_width for consistent spacing
    // Z is always 0 (at needle axis) - the state.position.z may be at the exit of previous loop
    Vec3 loop_position = Vec3(state.position.x + ctx.loop_dims.loop_width, 0.0f, 0.0f);

    // Create the physical loop
    PhysicalLoop loop = create_loop_at_position(ctx, loop_position);

    // Build the spline that forms this loop
    BezierSpline spline;

    float needle_radius = ctx.gauge.needle_diameter * 0.5f;
    float wrap_radius = needle_radius + ctx.yarn.radius;

    // For cast-on, we wrap around the needle from the current position to the loop entry.
    // The approach depends on whether this is the first cast-on or a subsequent one.
    //
    // For the FIRST cast-on (seg_id == 0), the yarn starts from "nowhere" - we position
    // it to flow naturally into the loop without any connector curve.
    // 
    // For subsequent cast-ons, the yarn is exiting the previous loop at the back (+Z)
    // and needs to wrap around to the entry of the next loop at the front (-Z).
    
    if (seg_id == 0) {
        // First cast-on: yarn starts directly at the loop entry
        // No connector curve needed - just the loop shape
    } else {
        // Subsequent cast-on: create connector from previous exit to this entry
        // The previous loop exits at back (+Z), this loop enters at front (-Z)
        //
        // This is a cylindrical wrap continuing from where the previous loop ended.
        // Use proper cylindrical arc approximations for smooth curvature.

        Vec3 exit_pos = state.position;

        // The yarn wraps around the needle cylinder from back to front (180°).
        // Break into two 90° arcs for better Bezier approximation.
        // Bezier factor for 90° arcs: k = (4/3) * tan(π/8) ≈ 0.5523
        constexpr float PI = 3.14159265f;
        float k = (4.0f / 3.0f) * std::tan(PI / 8.0f);

        // X position transitions linearly from exit to entry
        float x_exit = exit_pos.x;
        float x_entry = loop.entry_point.x;
        float x_bottom = (x_exit + x_entry) * 0.5f;

        // Cylinder center is at loop.center (Y coordinate)
        // θ = π/2 (back): Y = center, Z = +wrap_radius
        // θ = π (bottom): Y = center - wrap_radius, Z = 0
        // θ = 3π/2 (front): Y = center, Z = -wrap_radius

        // Arc 1: Back (θ = π/2) → Bottom (θ = π)
        Vec3 p0_arc1 = exit_pos;  // Already at back
        Vec3 p3_arc1 = Vec3(x_bottom, loop.center.y - wrap_radius, 0.0f);  // Bottom
        // Tangent at back points down (-Y)
        Vec3 t0_arc1 = Vec3(0.0f, -wrap_radius * k, 0.0f);
        // Tangent at bottom points toward front (-Z)
        Vec3 t3_arc1 = Vec3(0.0f, 0.0f, -wrap_radius * k);
        // Add X component to tangents
        float dx_arc1 = (x_bottom - x_exit) * 0.4f;
        CubicBezier arc1(p0_arc1, p0_arc1 + t0_arc1 + Vec3(dx_arc1, 0, 0),
                          p3_arc1 - t3_arc1 + Vec3(-dx_arc1, 0, 0), p3_arc1);
        yarnpath::logging::get_logger()->debug("  cast-on arc1 (back→bottom): max_curvature={:.3f}", arc1.max_curvature());
        spline.add_segment(arc1);

        // Arc 2: Bottom (θ = π) → Front (θ = 3π/2)
        Vec3 p0_arc2 = p3_arc1;  // Bottom
        Vec3 p3_arc2 = loop.entry_point;  // Front (entry)
        // Tangent at bottom points toward front (-Z)
        Vec3 t0_arc2 = Vec3(0.0f, 0.0f, -wrap_radius * k);
        // Tangent at front points up (+Y)
        Vec3 t3_arc2 = Vec3(0.0f, wrap_radius * k, 0.0f);
        // Add X component to tangents
        float dx_arc2 = (x_entry - x_bottom) * 0.4f;
        CubicBezier arc2(p0_arc2, p0_arc2 + t0_arc2 + Vec3(dx_arc2, 0, 0),
                          p3_arc2 - t3_arc2 + Vec3(-dx_arc2, 0, 0), p3_arc2);
        yarnpath::logging::get_logger()->debug("  cast-on arc2 (bottom→front): max_curvature={:.3f}", arc2.max_curvature());
        spline.add_segment(arc2);
    }

    // Add the loop shape itself (front → top → back)
    for (const auto& seg : loop.shape.segments()) {
        spline.add_segment(seg);
    }

    // Build the result
    result.geometry.segment_id = seg_id;
    result.geometry.curve = std::move(spline);
    result.geometry.arc_length = result.geometry.curve.total_arc_length();
    result.geometry.max_curvature = result.geometry.curve.max_curvature();
    yarnpath::logging::get_logger()->debug("  cast-on seg_id={} final max_curvature={:.3f} arc_length={:.3f}",
                 seg_id, result.geometry.max_curvature, result.geometry.arc_length);

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
    yarnpath::logging::get_logger()->debug("build_through_and_form_loop: seg_id={} through={} pos=({:.3f},{:.3f},{:.3f}) dir=({:.3f},{:.3f},{:.3f})",
                 seg_id, through.size(), 
                 state.position.x, state.position.y, state.position.z,
                 state.direction.x, state.direction.y, state.direction.z);
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
            yarnpath::logging::get_logger()->debug("  parent_id={} center=({:.3f},{:.3f},{:.3f}) apex=({:.3f},{:.3f},{:.3f})",
                         parent_id,
                         parent_loop.center.x, parent_loop.center.y, parent_loop.center.z,
                         parent_loop.apex_point.x, parent_loop.apex_point.y, parent_loop.apex_point.z);

            if (parent_loop.apex_point.y > highest_apex_y) {
                highest_apex_y = parent_loop.apex_point.y;
                highest_parent = &parent_loop;
            }
            parent_x = parent_loop.center.x;
        } else {
            yarnpath::logging::get_logger()->debug("  parent_id={} NOT FOUND in formed_loops!", parent_id);
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
        // No parent (yarn over) - stay at the same row level as surrounding stitches
        // state.position.y is the exit Y of the previous stitch, which equals its center Y
        new_center_y = state.position.y;
    }
    
    // Position the new loop offset by half loop_width from parent
    // This creates alternating positions for interlocking loops
    // The offset direction depends on which way we're knitting:
    // - If parent_x < last_parent_x, we're going left (decreasing X), offset negative
    // - If parent_x > last_parent_x, we're going right (increasing X), offset positive
    // - If no last_parent (first stitch of row), default to going left
    float loop_width = ctx.loop_dims.loop_width;
    float offset_x;

    if (state.last_parent_x.has_value()) {
        if (parent_x < state.last_parent_x.value() - 0.01f) {
            // Moving left (decreasing X)
            offset_x = -loop_width * 0.5f;
        } else if (parent_x > state.last_parent_x.value() + 0.01f) {
            // Moving right (increasing X)
            offset_x = loop_width * 0.5f;
        } else {
            // Same X as last parent - maintain previous direction or default left
            offset_x = -loop_width * 0.5f;
        }
    } else {
        // First stitch of row - default to going left (typical for row 1)
        // TODO: Get actual row direction from yarn path
        offset_x = -loop_width * 0.5f;
    }

    Vec3 loop_position = Vec3(parent_x + offset_x, new_center_y, 0.0f);
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
        yarnpath::logging::get_logger()->debug("  parent apex=({:.3f},{:.3f},{:.3f}) vertical_gap={:.3f} wrap_radius={:.3f} is_row_turn={}",
                     highest_parent->apex_point.x, highest_parent->apex_point.y, highest_parent->apex_point.z,
                     vertical_gap, wrap_radius, is_row_turn);
    } else {
        // No parent - check if going up significantly
        is_row_turn = (new_center_y - pos.y) > wrap_radius * 2.0f;
        yarnpath::logging::get_logger()->debug("  no parent, new_center_y={:.3f} is_row_turn={}", new_center_y, is_row_turn);
    }
    
    if (is_row_turn) {
        // At end of row, we need to complete the wrap around the needle
        // The yarn exits at back (+Z) going down, needs to continue around:
        // back (+Z) → bottom (Z=0) → front (-Z)
        // This completes the loop shape that was cut short
        
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
        auto row_turn_curve1 = make_smooth_curve(pos, dir, bottom_point, bottom_tangent);
        yarnpath::logging::get_logger()->debug("  row_turn curve1 (exit→bottom): max_curvature={:.3f}", row_turn_curve1.max_curvature());
        spline.add_segment(row_turn_curve1);
        
        // Second curve: bottom → front
        auto row_turn_curve2 = make_smooth_curve(bottom_point, bottom_tangent, front_point, front_tangent);
        yarnpath::logging::get_logger()->debug("  row_turn curve2 (bottom→front): max_curvature={:.3f}", row_turn_curve2.max_curvature());
        spline.add_segment(row_turn_curve2);
        
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

        // The connector passes through the parent loop opening and transitions
        // X from pos.x to entry.x. The loop shape (added after) has its own fixed X.
        //
        // Path: pos (back, +Z) → through parent opening → entry (front, -Z)
        // The X transition happens smoothly throughout this connector.

        float yarn_diameter = 2.0f * ctx.yarn.radius;
        constexpr float PI = 3.14159265f;
        float k = (4.0f / 3.0f) * std::tan(PI / 8.0f);

        // Pass-through point: inside parent loop opening, at PARENT'S X
        // The yarn must pass through the parent loop, so use the parent's X position
        Vec3 pass_through = Vec3(
            apex.x,                   // At parent loop's X position
            apex.y - yarn_diameter,   // Below apex, inside the loop opening
            0.0f                      // Z=0 (middle of the front-back transition)
        );

        // Arc 1: pos → pass_through
        // At pos (back, +Z): tangent points toward pass_through (up and forward)
        // At pass_through (Z=0): tangent points toward front (-Z)
        // X offset is based on actual X transition needed (may be zero if same X)
        Vec3 to_pass = (pass_through - pos).normalized();
        Vec3 t_pos = to_pass;  // Tangent follows natural path to pass-through
        Vec3 t_pass_in = Vec3(0.0f, 0.0f, -1.0f);
        float scale1 = wrap_radius * k;
        // Only add X offset if there's an actual X transition
        float dx1 = (pass_through.x - pos.x) * 0.5f;

        CubicBezier arc1(
            pos,
            pos + t_pos * scale1 + Vec3(dx1, 0, 0),
            pass_through - t_pass_in * scale1,
            pass_through
        );
        yarnpath::logging::get_logger()->debug("  arc1 (pos→pass): pos=({:.3f},{:.3f},{:.3f}) pass=({:.3f},{:.3f},{:.3f}) max_curvature={:.3f}",
                     pos.x, pos.y, pos.z, pass_through.x, pass_through.y, pass_through.z, arc1.max_curvature());
        spline.add_segment(arc1);

        // Arc 2: pass_through → entry
        // At pass_through: tangent points toward front (-Z)
        // At entry: tangent points up (+Y) to match loop entry
        Vec3 t_pass_out = Vec3(0.0f, 0.0f, -1.0f);
        Vec3 t_entry = Vec3(0.0f, 1.0f, 0.0f);
        float scale2 = wrap_radius * k;
        // X offset based on actual transition from pass_through to entry
        float dx2 = (entry.x - pass_through.x) * 0.5f;

        CubicBezier arc2(
            pass_through,
            pass_through + t_pass_out * scale2 + Vec3(dx2, 0, 0),
            entry - t_entry * scale2 + Vec3(-dx2, 0, 0),
            entry
        );
        yarnpath::logging::get_logger()->debug("  arc2 (pass→entry): entry=({:.3f},{:.3f},{:.3f}) max_curvature={:.3f}",
                     entry.x, entry.y, entry.z, arc2.max_curvature());
        spline.add_segment(arc2);
    } else {
        // No parent (yarn over) - wrap around the needle to the new loop entry
        // Similar to cast-on, create a cylindrical wrap
        
        // Bottom point of the wrap
        Vec3 bottom_point = Vec3(
            new_loop.center.x - new_loop.loop_width * 0.5f,  // Left side where entry is
            new_loop.center.y - wrap_radius,                  // Bottom of needle
            0.0f                                               // Z = 0 at bottom
        );
        
        // Tangent at bottom points toward front (-Z)
        Vec3 bottom_tangent = Vec3(0.0f, 0.0f, -1.0f);
        
        // First curve: pos → bottom (wrap down and around)
        auto yo_curve1 = make_smooth_curve(pos, dir, bottom_point, bottom_tangent);
        yarnpath::logging::get_logger()->debug("  yarn_over curve1 (pos→bottom): max_curvature={:.3f}", yo_curve1.max_curvature());
        spline.add_segment(yo_curve1);
        
        // Second curve: bottom → entry (continue around to front)
        auto yo_curve2 = make_smooth_curve(bottom_point, bottom_tangent, new_loop.entry_point, new_loop.entry_tangent);
        yarnpath::logging::get_logger()->debug("  yarn_over curve2 (bottom→entry): max_curvature={:.3f}", yo_curve2.max_curvature());
        spline.add_segment(yo_curve2);
    }

    // Add the loop shape
    for (const auto& seg : new_loop.shape.segments()) {
        spline.add_segment(seg);
    }

    result.geometry.segment_id = seg_id;
    result.geometry.curve = std::move(spline);
    result.geometry.arc_length = result.geometry.curve.total_arc_length();
    result.geometry.max_curvature = result.geometry.curve.max_curvature();
    yarnpath::logging::get_logger()->debug("  seg_id={} final max_curvature={:.3f} arc_length={:.3f} bezier_count={}",
                 seg_id, result.geometry.max_curvature, result.geometry.arc_length,
                 result.geometry.curve.segment_count());

    // Update state: yarn exits from loop exit point with the loop's exit tangent
    result.new_state.position = new_loop.exit_point;
    result.new_state.direction = new_loop.exit_tangent;
    result.new_state.formed_loops[seg_id] = new_loop;
    result.new_state.last_parent_x = parent_x;  // Track for direction detection

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
