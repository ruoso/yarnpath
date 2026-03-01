#include "geometry_builder.hpp"
#include "geometry_build_state.hpp"
#include "loop_precompute.hpp"
#include "position_resolver.hpp"
#include "geometry_validator.hpp"
#include "crossover_geometry.hpp"
#include <math/curvature_utils.hpp>
#include "logging.hpp"
#include <cmath>
#include <algorithm>
#include <limits>
#include <set>

namespace yarnpath {

// Build geometry with callback for visualization/debugging
GeometryPath build_geometry_with_callback(
    const YarnPath& yarn_path,
    const SurfaceGraph& surface,
    const YarnProperties& yarn,
    const Gauge& gauge,
    const GeometryBuildCallback& callback) {

    const GeometryBuildCallback* callback_ptr = callback ? &callback : nullptr;

    auto log = yarnpath::logging::get_logger();
    log->info("build_geometry: building geometry for {} segments using surface positions",
              yarn_path.segment_count());

    GeometryPath result;
    const auto& segments = yarn_path.segments();

    if (segments.empty()) {
        return result;
    }

    // Build reverse lookup of parents for a given segment
    std::map<SegmentId, std::vector<SegmentId>> segment_parents;
    for (size_t i = 0; i < segments.size(); ++i) {
        const auto& seg = segments[i];
        for (SegmentId parent_id : seg.through) {
            segment_parents[i].push_back(parent_id);
        }
    }

    // Also a simple lookup of children for a given segment
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

    // Create a cache of which segments have parents
    std::vector<bool> segment_has_parent(segments.size(), false);
    for (size_t i = 0; i < segments.size(); ++i) {
        const auto& seg = segments[i];
        if (!seg.through.empty()) {
            segment_has_parent[i] = true;
        }
    }

    // Build reverse lookup: loop_segment_id -> positions of segments that pass through it
    std::map<SegmentId, std::vector<Vec3>> loop_apex_positions;
    std::map<SegmentId, std::vector<SegmentId>> loop_child_ids;
    for (size_t i = 0; i < segments.size(); ++i) {
        const auto& seg = segments[i];
        SegmentId child_id = static_cast<SegmentId>(i);
        for (SegmentId parent_id : seg.through) {
            loop_apex_positions[parent_id].push_back(positions[i]);
            loop_child_ids[parent_id].push_back(child_id);
        }
    }

    log->debug("build_geometry: built reverse lookup for {} loops",
               loop_apex_positions.size());

    // Initialize geometry build state with gauge-derived dimensions
    GeometryBuildState state(yarn, gauge);
    log->debug("build_geometry: effective_loop_height={:.3f}, effective_stitch_width={:.3f}, effective_opening={:.3f}",
               state.effective_loop_height, state.effective_stitch_width, state.effective_opening_diameter);

    // Pre-compute all loop geometry before spline generation
    auto precomputed_loops = precompute_loop_geometry(
        segments, positions, loop_apex_positions, loop_child_ids,
        yarn, gauge,
        state.effective_loop_height,
        state.yarn_compressed_diameter, state.yarn_compressed_radius);
    log->debug("build_geometry: pre-computed geometry for {} loops", precomputed_loops.size());

    // Compute the first target point for the init tail direction.
    // This ensures the init tail points where segment 0 actually goes,
    // so there's no sharp direction change at the start.
    Vec3 first_target = positions.size() > 1 ? positions[1] : positions[0] + Vec3(1, 0, 0);
    if (!segments.empty() && segments[0].forms_loop) {
        auto precomp_it = precomputed_loops.find(0);
        if (precomp_it != precomputed_loops.end()) {
            // Point init toward the apex — the loop rises toward the apex,
            // so this gives the smoothest entry into the first loop.
            first_target = precomp_it->second.apex;
        }
    }
    initialize_running_spline(state, positions[0], first_target);

    // Slot claiming state: tracks which crossover slots have been consumed per parent
    std::map<SegmentId, std::set<size_t>> claimed_slots;
    // Store claimed crossover data so loop_entry/loop_exit can reference it
    using ParentChildKey = std::pair<SegmentId, SegmentId>;
    std::map<ParentChildKey, CrossoverData> claimed_entry_crossovers;
    std::map<ParentChildKey, CrossoverData> claimed_exit_crossovers;

    // Build geometry segment by segment
    for (size_t i = 0; i < segments.size(); ++i) {
        SegmentId seg_id = static_cast<SegmentId>(i);
        const auto& seg = segments[i];

        SegmentGeometry geom;
        geom.segment_id = seg_id;

        Vec3 curr_pos = positions[i];
        Vec3 next_pos = (i < positions.size() - 1) ? positions[i + 1] : positions[i];

        // Get child positions if this is a loop
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
        BezierSpline segment_spline;

        // Claim all crossover slots upfront (both entry and exit) so that
        // loop_entry/loop_exit can be set correctly before building the loop.
        for (SegmentId parent_id : seg.through) {
            auto parent_geom_it = precomputed_loops.find(parent_id);
            if (parent_geom_it != precomputed_loops.end() &&
                !parent_geom_it->second.crossover_slots.empty()) {
                // Claim entry slot
                auto entry_crossover = claim_nearest_slot(
                    parent_geom_it->second.crossover_slots,
                    claimed_slots[parent_id],
                    positions[seg_id],
                    state.yarn_compressed_radius,
                    /*as_entry=*/true);
                claimed_entry_crossovers[{parent_id, seg_id}] = entry_crossover;

                // Claim exit slot (only if this segment forms a loop)
                if (seg.forms_loop) {
                    auto exit_crossover = claim_nearest_slot(
                        parent_geom_it->second.crossover_slots,
                        claimed_slots[parent_id],
                        positions[seg_id],
                        state.yarn_compressed_radius,
                        /*as_entry=*/false);
                    claimed_exit_crossovers[{parent_id, seg_id}] = exit_crossover;
                }
            }
        }

        // Build entry passthrough through parent crossover points
        for (SegmentId parent_id : seg.through) {
            auto entry_it = claimed_entry_crossovers.find({parent_id, seg_id});
            if (entry_it != claimed_entry_crossovers.end()) {
                build_parent_passthrough(state, segment_spline, entry_it->second, on_curve_added);
            }
        }

        // Then build this segment's own geometry
        if (seg.forms_loop) {
            auto precomp_it = precomputed_loops.find(seg_id);
            if (precomp_it != precomputed_loops.end()) {
                auto& precomp = precomp_it->second;

                // Determine loop entry/exit positions.
                // Use claimed crossover points so the loop connects to crossings.
                Vec3 loop_entry = precomp.apex_entry;
                Vec3 loop_exit = precomp.apex_exit;
                for (SegmentId parent_id : seg.through) {
                    auto entry_it = claimed_entry_crossovers.find({parent_id, seg_id});
                    if (entry_it != claimed_entry_crossovers.end()) {
                        loop_entry = entry_it->second.exit;  // entry crossover exit = loop start
                    }
                    auto exit_it = claimed_exit_crossovers.find({parent_id, seg_id});
                    if (exit_it != claimed_exit_crossovers.end()) {
                        loop_exit = exit_it->second.entry;  // exit crossover entry = loop end
                    }
                }

                // Update crossover slot positions to cluster at apex on the actual
                // loop path. Uses the actual loop travel direction so children
                // (processed later) claim correctly positioned slots.
                {
                    int num_slots = static_cast<int>(precomp.crossover_slots.size());
                    Vec3 travel = safe_normalized(loop_exit - loop_entry, Vec3(1, 0, 0));
                    for (int s = 0; s < num_slots; ++s) {
                        float offset = (static_cast<float>(s) - (num_slots - 1) / 2.0f)
                                     * state.yarn_compressed_diameter;
                        Vec3 normal = precomp.crossover_slots[s].crossing_normal;
                        precomp.crossover_slots[s].position = precomp.apex + travel * offset
                                                            + normal * state.yarn_compressed_radius;
                        precomp.crossover_slots[s].tangent = travel;
                    }
                }

                // Phase B: Surface-guided loop construction

                // 1. Connector from running spline end to loop entry
                if (!state.running_spline.empty()) {
                    Vec3 spline_end = state.running_spline.segments().back().end();
                    Vec3 spline_dir = state.running_spline.segments().back().tangent(1.0f);
                    Vec3 entry_dir = safe_normalized(loop_exit - loop_entry, Vec3(1, 0, 0));
                    add_connector_with_curvature_check(
                        state, segment_spline,
                        spline_end, spline_dir,
                        loop_entry, entry_dir,
                        "loop_entry_connector", on_curve_added);
                }

                // 2. Build the loop: route through crossover slot waypoints if any,
                //    otherwise use simple 2-segment loop
                if (!precomp.crossover_slots.empty()) {
                    build_surface_guided_loop_with_crossings(
                        state, segment_spline,
                        loop_entry, precomp.apex, loop_exit,
                        precomp.shape.z_bulge,
                        precomp.crossover_slots,
                        on_curve_added);
                } else {
                    build_surface_guided_loop(
                        state, segment_spline,
                        loop_entry, precomp.apex, loop_exit,
                        precomp.shape.z_bulge,
                        on_curve_added);
                }

            } else {
                // Fallback: simple connector if no precomputed data
                if (!state.running_spline.empty()) {
                    Vec3 spline_end = state.running_spline.segments().back().end();
                    Vec3 spline_dir = state.running_spline.segments().back().tangent(1.0f);
                    Vec3 target_dir = safe_normalized(next_pos - curr_pos, Vec3(1, 0, 0));
                    add_connector_with_curvature_check(
                        state, segment_spline,
                        spline_end, spline_dir,
                        next_pos, target_dir,
                        "loop_fallback_connector", on_curve_added);
                }
            }
        } else {
            // Non-loop segment: simple connector
            if (!state.running_spline.empty()) {
                Vec3 spline_end = state.running_spline.segments().back().end();
                Vec3 spline_dir = state.running_spline.segments().back().tangent(1.0f);
                Vec3 target_dir = safe_normalized(next_pos - curr_pos, Vec3(1, 0, 0));
                add_connector_with_curvature_check(
                    state, segment_spline,
                    spline_end, spline_dir,
                    next_pos, target_dir,
                    "connector", on_curve_added);
            }
        }

        // Build exit passthrough (only if this segment forms a loop,
        // since non-loop children only cross once through the parent).
        if (seg.forms_loop) {
            for (SegmentId parent_id : seg.through) {
                auto exit_it = claimed_exit_crossovers.find({parent_id, seg_id});
                if (exit_it != claimed_exit_crossovers.end()) {
                    build_parent_passthrough(state, segment_spline, exit_it->second, on_curve_added);
                }
            }
        }

        geom.curve = segment_spline;

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

    // Phase E: Center geometry in X after building all segments
    center_geometry_x(result.segments_);

    // Validation: report geometry quality issues per segment
    validate_geometry(result.segments_, state.max_curvature);

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
