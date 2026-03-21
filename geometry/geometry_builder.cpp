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

    // Resolve full segment frames (position + local axes) from the surface
    auto frames = resolve_segment_frames(
        yarn_path, surface, yarn, segment_parents, segment_children);

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
            loop_apex_positions[parent_id].push_back(frames[i].position);
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
        segments, frames, loop_apex_positions, loop_child_ids,
        yarn, gauge,
        state.effective_loop_height,
        state.yarn_compressed_diameter, state.yarn_compressed_radius);
    log->debug("build_geometry: pre-computed geometry for {} loops", precomputed_loops.size());

    // Compute the first target point for the init tail direction.
    // This ensures the init tail points where segment 0 actually goes,
    // so there's no sharp direction change at the start.
    Vec3 first_target = frames.size() > 1 ? frames[1].position : frames[0].position + Vec3(1, 0, 0);
    if (!segments.empty() && segments[0].forms_loop) {
        auto precomp_it = precomputed_loops.find(0);
        if (precomp_it != precomputed_loops.end()) {
            // Point init toward the apex — the loop rises toward the apex,
            // so this gives the smoothest entry into the first loop.
            first_target = precomp_it->second.apex;
        }
    }
    // Initialize the running spline with a tail segment.
    // We route it through the callback so every curve is tracked.
    {
        BezierSpline init_spline;
        CurveAddedCallback init_cb = nullptr;
        if (callback_ptr) {
            init_cb = [&](const std::string& description) {
                (*callback_ptr)(0, description, state.running_spline);
            };
        }
        initialize_running_spline(state, frames[0].position, first_target,
                                  init_spline, init_cb);
    }

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

        Vec3 curr_pos = frames[i].position;
        Vec3 next_pos = (i < frames.size() - 1) ? frames[i + 1].position : frames[i].position;

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
        // stitch_axis already points in the yarn's travel direction (computed
        // from yarn path continuity neighbors), so entry slots are claimed on
        // the approach side and exit slots on the departure side.
        for (SegmentId parent_id : seg.through) {
            auto parent_geom_it = precomputed_loops.find(parent_id);
            if (parent_geom_it != precomputed_loops.end() &&
                !parent_geom_it->second.crossover_slots.empty()) {
                // Claim entry slot
                auto entry_crossover = claim_nearest_slot(
                    parent_geom_it->second.crossover_slots,
                    claimed_slots[parent_id],
                    frames[seg_id].position,
                    state.yarn_compressed_radius,
                    /*as_entry=*/true,
                    frames[seg_id].wale_axis,
                    frames[seg_id].stitch_axis);
                claimed_entry_crossovers[{parent_id, seg_id}] = entry_crossover;

                // Claim exit slot (only if this segment forms a loop)
                if (seg.forms_loop) {
                    auto exit_crossover = claim_nearest_slot(
                        parent_geom_it->second.crossover_slots,
                        claimed_slots[parent_id],
                        frames[seg_id].position,
                        state.yarn_compressed_radius,
                        /*as_entry=*/false,
                        frames[seg_id].wale_axis,
                        frames[seg_id].stitch_axis);
                    claimed_exit_crossovers[{parent_id, seg_id}] = exit_crossover;
                }
            }
        }

        // Then build this segment's own geometry
        if (seg.forms_loop) {
            auto precomp_it = precomputed_loops.find(seg_id);
            if (precomp_it != precomputed_loops.end()) {
                auto& precomp = precomp_it->second;

                // Collect entry and exit crossover data for the unified chain
                std::vector<CrossoverData> entry_xovers;
                std::vector<CrossoverData> exit_xovers;
                for (SegmentId parent_id : seg.through) {
                    auto entry_it = claimed_entry_crossovers.find({parent_id, seg_id});
                    if (entry_it != claimed_entry_crossovers.end()) {
                        entry_xovers.push_back(entry_it->second);
                    }
                    auto exit_it = claimed_exit_crossovers.find({parent_id, seg_id});
                    if (exit_it != claimed_exit_crossovers.end()) {
                        exit_xovers.push_back(exit_it->second);
                    }
                }

                // Build unified chain: crossover entry → loop → crossover exit
                // All waypoints in one natural spline for globally smooth tangents.
                build_full_loop_chain(
                    state, segment_spline, precomp,
                    entry_xovers, exit_xovers,
                    on_curve_added);

            } else {
                // Fallback: entry passthroughs + simple connector
                for (SegmentId parent_id : seg.through) {
                    auto entry_it = claimed_entry_crossovers.find({parent_id, seg_id});
                    if (entry_it != claimed_entry_crossovers.end()) {
                        build_parent_passthrough(state, segment_spline, entry_it->second, on_curve_added);
                    }
                }
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
            // Non-loop segment: entry passthroughs + simple connector
            for (SegmentId parent_id : seg.through) {
                auto entry_it = claimed_entry_crossovers.find({parent_id, seg_id});
                if (entry_it != claimed_entry_crossovers.end()) {
                    build_parent_passthrough(state, segment_spline, entry_it->second, on_curve_added);
                }
            }
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
    result.x_center_offset_ = center_geometry_x(result.segments_);

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
