#include "geometry_builder.hpp"
#include "physical_loop.hpp"
#include "logging.hpp"
#include <algorithm>
#include <cmath>

namespace yarnpath {

GeometryBuilder::GeometryBuilder(
    const YarnPath& yarn_path,
    const YarnProperties& yarn,
    const Gauge& gauge,
    const FabricSurface& surface)
    : yarn_path_(yarn_path)
    , yarn_(yarn)
    , gauge_(gauge)
    , surface_(surface) {}

GeometryPath GeometryBuilder::build() {
    auto log = yarnpath::logging::get_logger();
    GeometryPath result;

    if (yarn_path_.loops().empty()) {
        log->debug("GeometryBuilder: no loops to process");
        return result;
    }

    log->debug("GeometryBuilder: building geometry for {} loops", yarn_path_.loops().size());

    // Calculate physical loop dimensions from needle and yarn
    auto loop_dim = LoopDimensions::calculate(yarn_, gauge_);
    log->debug("GeometryBuilder: physical loop dimensions - opening={}, height={}, width={}",
               loop_dim.opening_diameter, loop_dim.loop_height, loop_dim.loop_width);

    // Phase 1: Create physical loops for each stitch
    // Positions are determined by topology (parent-child relationships)
    std::map<LoopId, PhysicalLoop> physical_loops;

    // First, position cast-on loops (they have no parents)
    float cast_on_x = 0.0f;
    for (const auto& loop : yarn_path_.loops()) {
        if (loop.kind == FormKind::CastOn) {
            // Cast-on loops are positioned horizontally
            Vec3 position(cast_on_x + loop_dim.loop_width * 0.5f, loop_dim.loop_height * 0.5f, 0.0f);
            physical_loops[loop.id] = PhysicalLoop::from_properties(yarn_, gauge_, position);

            log->debug("GeometryBuilder: cast-on loop {} at ({}, {}, {})",
                       loop.id, position.x, position.y, position.z);

            // Store position info
            LoopPosition pos;
            pos.loop_id = loop.id;
            pos.u = cast_on_x;
            pos.v = 0.0f;
            loop_positions_[loop.id] = pos;

            cast_on_x += loop_dim.loop_width;
        }
    }

    // Then, position child loops based on their parents
    // Use BFS to process in topological order
    std::set<LoopId> positioned;
    for (const auto& [id, _] : physical_loops) {
        positioned.insert(id);
    }

    std::queue<LoopId> to_process;
    for (const auto& loop : yarn_path_.loops()) {
        if (loop.kind != FormKind::CastOn && !loop.parent_loops.empty()) {
            to_process.push(loop.id);
        }
    }

    size_t max_iterations = yarn_path_.loops().size() * 2;
    size_t iteration = 0;

    while (!to_process.empty() && iteration < max_iterations) {
        ++iteration;
        LoopId loop_id = to_process.front();
        to_process.pop();

        if (positioned.count(loop_id) > 0) {
            continue;
        }

        const Loop* loop = yarn_path_.get_loop(loop_id);
        if (!loop) continue;

        // Check if all parents are positioned
        bool all_parents_ready = true;
        for (LoopId parent_id : loop->parent_loops) {
            if (positioned.count(parent_id) == 0) {
                all_parents_ready = false;
                break;
            }
        }

        if (!all_parents_ready) {
            to_process.push(loop_id);
            continue;
        }

        // Position this loop based on its parent(s)
        Vec3 position(0.0f, 0.0f, 0.0f);
        if (!loop->parent_loops.empty()) {
            // Average parent positions horizontally
            float avg_x = 0.0f;
            float max_parent_center_y = 0.0f;
            for (LoopId parent_id : loop->parent_loops) {
                const auto& parent = physical_loops[parent_id];
                avg_x += parent.center.x;
                max_parent_center_y = std::max(max_parent_center_y, parent.center.y);
            }
            avg_x /= loop->parent_loops.size();

            // Row spacing should be based on the loop height, which comes from wrapping
            // around the needle. When loops interlock, they overlap by approximately
            // half their height, so the row-to-row center spacing is about loop_height.
            //
            // The exact relationship is:
            // - Parent loop wraps around needle with wrap_radius
            // - Child loop also wraps with wrap_radius
            // - They interlock at the catch point (parent apex)
            // - Row spacing = loop_dim.loop_height (vertical extent of one loop)
            position.x = avg_x;
            position.y = max_parent_center_y + loop_dim.loop_height;
            position.z = 0.0f;
        }

        physical_loops[loop_id] = PhysicalLoop::from_properties(yarn_, gauge_, position);
        positioned.insert(loop_id);

        log->debug("GeometryBuilder: loop {} (kind={}) at ({}, {}, {})",
                   loop_id, static_cast<int>(loop->kind), position.x, position.y, position.z);

        // Store position info
        LoopPosition pos;
        pos.loop_id = loop_id;
        pos.u = position.x;
        pos.v = position.y;
        loop_positions_[loop_id] = pos;
    }

    // Handle loops without parents (like YarnOver, M1L, M1R)
    // These need to be positioned based on their neighbors in the yarn path
    for (const auto& loop : yarn_path_.loops()) {
        if (positioned.count(loop.id) > 0) continue;
        if (loop.kind == FormKind::CastOn) continue;

        // This loop has no parents - position it based on yarn neighbors
        Vec3 position(0.0f, 0.0f, 0.0f);
        int neighbor_count = 0;

        // Look at previous loop in yarn
        if (loop.prev_in_yarn) {
            auto prev_it = physical_loops.find(*loop.prev_in_yarn);
            if (prev_it != physical_loops.end()) {
                position.x += prev_it->second.center.x + loop_dim.loop_width;
                position.y += prev_it->second.center.y;
                neighbor_count++;
            }
        }

        // Look at next loop in yarn
        if (loop.next_in_yarn) {
            auto next_it = physical_loops.find(*loop.next_in_yarn);
            if (next_it != physical_loops.end()) {
                position.x += next_it->second.center.x - loop_dim.loop_width;
                position.y += next_it->second.center.y;
                neighbor_count++;
            }
        }

        if (neighbor_count > 0) {
            position.x /= neighbor_count;
            position.y /= neighbor_count;
        } else {
            // Fallback: position at origin with offset
            position.x = loop.id * loop_dim.loop_width;
            position.y = loop_dim.loop_height;
        }

        physical_loops[loop.id] = PhysicalLoop::from_properties(yarn_, gauge_, position);
        positioned.insert(loop.id);

        log->debug("GeometryBuilder: parentless loop {} (kind={}) at ({}, {}, {})",
                   loop.id, static_cast<int>(loop.kind), position.x, position.y, position.z);

        LoopPosition pos;
        pos.loop_id = loop.id;
        pos.u = position.x;
        pos.v = position.y;
        loop_positions_[loop.id] = pos;
    }

    // Phase 2: Generate yarn path by walking through loops in yarn order
    // The yarn forms loops and passes through parent loops
    log->debug("GeometryBuilder: Phase 2 - generating yarn path");

    // We build the path by directly accumulating Bezier segments
    // For loops with pre-computed shapes (like cast-on), we use those directly
    // For transitions between loops, we create connecting segments
    BezierSpline full_path;
    std::optional<Vec3> last_point;  // Track where we left off for connections

    // Find first loop
    LoopId first_loop = yarn_path_.first_loop();
    std::optional<LoopId> current_loop = first_loop;

    while (current_loop) {
        const Loop* loop = yarn_path_.get_loop(*current_loop);
        if (!loop) break;

        auto phys_it = physical_loops.find(*current_loop);
        if (phys_it == physical_loops.end()) {
            current_loop = loop->next_in_yarn;
            continue;
        }

        const PhysicalLoop& phys = phys_it->second;

        // For each loop, the yarn path goes:
        // 1. Entry point (coming from previous loop or start)
        // 2. Around the loop shape
        // 3. Through parent loop opening (if knit/purl)
        // 4. Exit point (going to next loop)

        bool is_knit = (loop->kind == FormKind::Knit);
        bool is_purl = (loop->kind == FormKind::Purl);
        bool is_cast_on = (loop->kind == FormKind::CastOn);

        if (is_cast_on) {
            // Cast-on: use the pre-computed cylinder-wrapped shape directly
            // This ensures the yarn properly wraps around the needle

            // If we have a previous point, connect to the loop entry
            if (last_point) {
                Vec3 entry = phys.shape.segments().empty()
                    ? phys.entry_point
                    : phys.shape.segments().front().start();
                Vec3 tangent = (entry - *last_point) * 0.4f;
                CubicBezier connector = CubicBezier::from_hermite(
                    *last_point, tangent, entry, tangent);
                full_path.add_segment(connector);
            }

            // Add all segments from the pre-computed shape
            for (const auto& seg : phys.shape.segments()) {
                full_path.add_segment(seg);
            }

            // Update last point to exit of the shape
            if (!phys.shape.segments().empty()) {
                last_point = phys.shape.segments().back().end();
            } else {
                last_point = phys.exit_point;
            }
        } else if (is_knit || is_purl) {
            // Knit/Purl: the yarn catches the parent loop at its apex, then wraps around
            // the new loop's needle.
            //
            // The direction of travel (from yarn topology) determines the wrap direction:
            // - Traveling LEFT (decreasing X): catch from front, exit to front
            // - Traveling RIGHT (increasing X): catch from back, exit to back
            //
            // After catching, the yarn wraps around the needle:
            // - If caught from front: yarn is at back -> wrap: back -> bottom -> front
            // - If caught from back: yarn is at front -> wrap: front -> bottom -> back

            // Determine travel direction from yarn topology
            bool traveling_left = false;
            bool traveling_right = false;

            if (loop->prev_in_yarn) {
                auto prev_it = physical_loops.find(*loop->prev_in_yarn);
                if (prev_it != physical_loops.end()) {
                    float prev_x = prev_it->second.center.x;
                    float curr_x = phys.center.x;
                    traveling_left = prev_x > curr_x;
                    traveling_right = prev_x < curr_x;
                    log->debug("GeometryBuilder: loop {} prev={} (x={}) curr_x={} -> traveling_{}",
                               *current_loop, *loop->prev_in_yarn, prev_x, curr_x,
                               traveling_left ? "left" : (traveling_right ? "right" : "none"));
                }
            }

            // Default to traveling left if we can't determine
            if (!traveling_left && !traveling_right) {
                traveling_left = true;
                log->debug("GeometryBuilder: loop {} defaulting to traveling_left", *current_loop);
            }

            // Get parent loop for catch point calculation
            if (!loop->parent_loops.empty()) {
                LoopId parent_id = loop->parent_loops[0];
                auto parent_it = physical_loops.find(parent_id);
                if (parent_it != physical_loops.end()) {
                    const PhysicalLoop& parent = parent_it->second;

                    // The yarn must:
                    // 1. Pass THROUGH the parent loop's opening (front to back for knit)
                    // 2. Then catch the parent at its apex
                    // 3. Then wrap around the new needle
                    //
                    // Parent loop geometry:
                    // - opening_center: center of the opening (at needle axis level)
                    // - apex_point: top of the loop where the catch happens
                    float wrap_radius = gauge_.needle_diameter * 0.5f + yarn_.radius;

                    // Z positions for front and back
                    float front_z = -wrap_radius;
                    float back_z = wrap_radius;

                    // Build transition based on travel direction
                    std::vector<YarnPathPoint> transition_points;

                    if (last_point) {
                        transition_points.push_back(YarnPathPoint(*last_point, 0.3f));
                    }

                    if (traveling_left) {
                        // Traveling left (knit on RS): yarn goes front to back through parent
                        //
                        // Path: last_point -> approach parent from front ->
                        //       PASS THROUGH parent opening -> catch at apex ->
                        //       back of new needle

                        // 1. Approach parent from front (at opening level)
                        Vec3 front_of_parent(
                            parent.opening_center.x,
                            parent.opening_center.y,
                            front_z * 0.8f  // Front of parent
                        );
                        transition_points.push_back(YarnPathPoint(front_of_parent, 0.4f));

                        // 2. Pass through parent opening (Z crosses 0)
                        Vec3 pass_through(
                            parent.opening_center.x,
                            parent.opening_center.y + yarn_.radius,  // Slightly above center for clearance
                            0.0f  // At Z=0, passing through
                        );
                        transition_points.push_back(YarnPathPoint(pass_through, 0.5f));

                        // 3. Exit parent to back, rising to catch at apex
                        Vec3 back_of_parent(
                            parent.opening_center.x,
                            parent.apex_point.y + yarn_.radius,  // At apex catch level
                            back_z * 0.8f  // Back of parent
                        );
                        transition_points.push_back(YarnPathPoint(back_of_parent, 0.5f));

                        // 4. Continue to back of new needle for wrap
                        Vec3 back_entry(phys.center.x, phys.center.y, back_z);
                        transition_points.push_back(YarnPathPoint(back_entry, 0.5f));

                        log->debug("GeometryBuilder: loop {} transition (traveling_left): "
                                   "front_of_parent=({},{},{}), pass_through=({},{},{}), "
                                   "back_of_parent=({},{},{}), back_entry=({},{},{})",
                                   *current_loop,
                                   front_of_parent.x, front_of_parent.y, front_of_parent.z,
                                   pass_through.x, pass_through.y, pass_through.z,
                                   back_of_parent.x, back_of_parent.y, back_of_parent.z,
                                   back_entry.x, back_entry.y, back_entry.z);

                    } else {
                        // Traveling right (purl on RS, or knit on WS): yarn goes back to front
                        //
                        // Path: last_point -> approach parent from back ->
                        //       PASS THROUGH parent opening -> catch at apex ->
                        //       front of new needle

                        // 1. Approach parent from back (at opening level)
                        Vec3 back_of_parent(
                            parent.opening_center.x,
                            parent.opening_center.y,
                            back_z * 0.8f  // Back of parent
                        );
                        transition_points.push_back(YarnPathPoint(back_of_parent, 0.4f));

                        // 2. Pass through parent opening (Z crosses 0)
                        Vec3 pass_through(
                            parent.opening_center.x,
                            parent.opening_center.y + yarn_.radius,
                            0.0f
                        );
                        transition_points.push_back(YarnPathPoint(pass_through, 0.5f));

                        // 3. Exit parent to front, rising to catch at apex
                        Vec3 front_of_parent(
                            parent.opening_center.x,
                            parent.apex_point.y + yarn_.radius,
                            front_z * 0.8f
                        );
                        transition_points.push_back(YarnPathPoint(front_of_parent, 0.5f));

                        // 4. Continue to front of new needle for wrap
                        Vec3 front_entry(phys.center.x, phys.center.y, front_z);
                        transition_points.push_back(YarnPathPoint(front_entry, 0.5f));
                    }

                    // Create transition spline
                    if (transition_points.size() >= 2) {
                        BezierSpline transition = BezierSpline::from_yarn_points(transition_points);
                        for (const auto& seg : transition.segments()) {
                            full_path.add_segment(seg);
                        }
                        last_point = transition.segments().back().end();
                    }
                }
            }

            // Add the appropriate portion of the loop shape based on travel direction
            // The shape has 4 segments:
            //   0: Front to Top
            //   1: Top to Back
            //   2: Back to Bottom
            //   3: Bottom to Front
            //
            // After catching, the yarn wraps OVER the needle (not under):
            // - Traveling left: entered at back -> wrap: back -> TOP -> front
            // - Traveling right: entered at front -> wrap: front -> TOP -> back
            const auto& segments = phys.shape.segments();
            if (segments.size() >= 4) {
                if (traveling_left) {
                    // Entered at back, wrap OVER the top: back -> top -> front
                    // Use segment 1 reversed (Back to Top), then segment 0 reversed (Top to Front)
                    Vec3 back_point = segments[1].end();  // Back position
                    Vec3 top_point = segments[1].start(); // Top position (= segments[0].end())
                    Vec3 front_point = segments[0].start(); // Front position

                    log->debug("GeometryBuilder: loop {} wrap (traveling_left) back=({},{},{}), top=({},{},{}), front=({},{},{})",
                               *current_loop,
                               back_point.x, back_point.y, back_point.z,
                               top_point.x, top_point.y, top_point.z,
                               front_point.x, front_point.y, front_point.z);

                    if (last_point) {
                        Vec3 tangent = (back_point - *last_point) * 0.4f;
                        CubicBezier connector = CubicBezier::from_hermite(
                            *last_point, tangent, back_point, tangent);
                        full_path.add_segment(connector);
                    }
                    full_path.add_segment(segments[1].reversed());  // Back to Top
                    full_path.add_segment(segments[0].reversed());  // Top to Front
                    last_point = segments[0].start();  // Front position
                } else {
                    // Entered at front, wrap OVER the top: front -> top -> back
                    // Use segment 0 (Front to Top), then segment 1 (Top to Back)
                    if (last_point) {
                        Vec3 front_point = segments[0].start();  // Front position
                        Vec3 tangent = (front_point - *last_point) * 0.4f;
                        CubicBezier connector = CubicBezier::from_hermite(
                            *last_point, tangent, front_point, tangent);
                        full_path.add_segment(connector);
                    }
                    full_path.add_segment(segments[0]);  // Front to Top
                    full_path.add_segment(segments[1]);  // Top to Back
                    last_point = segments[1].end();  // Back position
                }
            } else {
                // Fallback: use full shape
                Vec3 entry = segments.empty()
                    ? phys.entry_point
                    : segments.front().start();
                if (last_point) {
                    Vec3 tangent = (entry - *last_point) * 0.4f;
                    CubicBezier connector = CubicBezier::from_hermite(
                        *last_point, tangent, entry, tangent);
                    full_path.add_segment(connector);
                }
                for (const auto& seg : segments) {
                    full_path.add_segment(seg);
                }
                if (!segments.empty()) {
                    last_point = segments.back().end();
                } else {
                    last_point = phys.exit_point;
                }
            }
        } else {
            // Other stitch types: use the pre-computed shape
            if (last_point) {
                Vec3 entry = phys.shape.segments().empty()
                    ? phys.entry_point
                    : phys.shape.segments().front().start();
                Vec3 tangent = (entry - *last_point) * 0.4f;
                CubicBezier connector = CubicBezier::from_hermite(
                    *last_point, tangent, entry, tangent);
                full_path.add_segment(connector);
            }

            for (const auto& seg : phys.shape.segments()) {
                full_path.add_segment(seg);
            }

            if (!phys.shape.segments().empty()) {
                last_point = phys.shape.segments().back().end();
            } else {
                last_point = phys.exit_point;
            }
        }

        current_loop = loop->next_in_yarn;
    }

    log->debug("GeometryBuilder: built path with {} segments", full_path.segments().size());

    // Phase 3: Store the path
    if (!full_path.segments().empty()) {
        SegmentGeometry seg_geom;
        seg_geom.segment_id = 0;
        seg_geom.curve = std::move(full_path);
        seg_geom.arc_length = seg_geom.curve.total_arc_length();
        seg_geom.max_curvature = seg_geom.curve.max_curvature();

        result.segment_index_[seg_geom.segment_id] = result.segments_.size();
        result.segments_.push_back(std::move(seg_geom));
    }

    // Store loop positions in result
    for (const auto& [loop_id, pos] : loop_positions_) {
        result.loop_positions_.push_back(pos);
        result.loop_position_index_[loop_id] = result.loop_positions_.size() - 1;
    }

    size_t total_bezier_segments = 0;
    for (const auto& seg : result.segments_) {
        total_bezier_segments += seg.curve.segments().size();
    }
    log->debug("GeometryBuilder: build complete - {} loops, {} geometry segments, {} bezier segments",
               result.loop_positions_.size(), result.segments_.size(), total_bezier_segments);
    return result;
}

float GeometryBuilder::get_z_offset(PassMode mode) const {
    float thickness = gauge_.fabric_thickness * yarn_.radius;

    switch (mode) {
        case PassMode::KnitWise:
            return -thickness * 0.3f;
        case PassMode::PurlWise:
            return thickness * 0.3f;
        case PassMode::ThroughBackLoop:
            return -thickness * 0.4f;
        case PassMode::ThroughFrontLoop:
            return thickness * 0.4f;
        default:
            return 0.0f;
    }
}

Vec3 GeometryBuilder::to_world_position(float u, float v, float z) const {
    return surface_.local_to_world(u, v, 0.0f, 0.0f, z);
}

}  // namespace yarnpath
