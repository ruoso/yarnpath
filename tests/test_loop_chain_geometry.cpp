#include <gtest/gtest.h>
#include <geometry/loop_precompute.hpp>
#include <geometry/geometry_build_state.hpp>
#include <geometry/position_resolver.hpp>
#include <geometry/crossover_geometry.hpp>
#include <surface/surface_builder.hpp>
#include <surface/surface_solver.hpp>
#include "test_helpers.hpp"

#include <cmath>
#include <map>
#include <set>
#include <iostream>

using namespace yarnpath;
using namespace yarnpath::test;


// ---------------------------------------------------------------------------
// Helper: build full pipeline and run build_full_loop_chain for a loop segment
// ---------------------------------------------------------------------------
struct ChainTestData {
    YarnPath yarn_path;
    SurfaceGraph surface;
    std::vector<SegmentFrame> frames;
    std::map<SegmentId, PrecomputedLoopGeometry> loops;
    std::map<SegmentId, std::vector<SegmentId>> parent_map;
    std::map<SegmentId, std::vector<SegmentId>> children_map;
    GeometryBuildState* state_ptr;  // Owned below
    std::unique_ptr<GeometryBuildState> state_owner;
};

static ChainTestData build_chain_data(const std::vector<std::string>& rows) {
    YarnProperties yarn = default_yarn();
    Gauge gauge = default_gauge();

    PatternInstructions pattern = create_pattern(rows);
    StitchGraph graph = StitchGraph::from_instructions(pattern);
    YarnPath yarn_path = YarnPath::from_stitch_graph(graph, yarn, gauge);

    SurfaceBuildConfig build_config;
    build_config.random_seed = 42;
    SurfaceGraph surface = SurfaceBuilder::from_yarn_path(yarn_path, yarn, gauge, build_config);

    SolveConfig solve_config;
    solve_config.max_iterations = 1000;
    solve_config.convergence_threshold = 1e-4f;
    SurfaceSolver::solve(surface, yarn, gauge, solve_config);

    const auto& segments = yarn_path.segments();
    std::map<SegmentId, std::vector<SegmentId>> parent_map;
    std::map<SegmentId, std::vector<SegmentId>> children_map;
    for (size_t i = 0; i < segments.size(); ++i) {
        for (SegmentId parent_id : segments[i].through) {
            parent_map[i].push_back(parent_id);
            children_map[parent_id].push_back(i);
        }
    }

    auto frames = resolve_segment_frames(yarn_path, surface, yarn, parent_map, children_map);

    std::map<SegmentId, std::vector<Vec3>> loop_child_positions;
    std::map<SegmentId, std::vector<SegmentId>> loop_child_ids;
    for (size_t i = 0; i < segments.size(); ++i) {
        for (SegmentId pid : segments[i].through) {
            loop_child_positions[pid].push_back(frames[i].position);
            loop_child_ids[pid].push_back(i);
        }
    }

    auto state_owner = std::make_unique<GeometryBuildState>(yarn, gauge);
    auto* state_ptr = state_owner.get();

    auto loops = precompute_loop_geometry(
        segments, frames, loop_child_positions, loop_child_ids,
        yarn, gauge,
        state_ptr->effective_loop_height,
        state_ptr->yarn_compressed_diameter, state_ptr->yarn_compressed_radius);

    return ChainTestData{
        std::move(yarn_path), std::move(surface), std::move(frames),
        std::move(loops), std::move(parent_map), std::move(children_map),
        state_ptr, std::move(state_owner)
    };
}

// A named waypoint captured from build_full_loop_chain
struct NamedWaypoint {
    std::string name;
    Vec3 position;
};

// Result from building a chain for a segment, including crossover data
struct ChainBuildResult {
    CatmullRomSpline spline;
    std::vector<CrossoverData> entry_crossovers;
    std::vector<CrossoverData> exit_crossovers;
    std::vector<NamedWaypoint> waypoints;
};

// Helper: initialize state and build chain for a specific loop segment
static ChainBuildResult build_chain_for_segment(
    ChainTestData& data, SegmentId seg_id) {

    auto& state = *data.state_ptr;
    const auto& segments = data.yarn_path.segments();
    const auto& geom = data.loops.at(seg_id);

    // Initialize the running spline if empty
    if (state.running_spline.empty()) {
        Vec3 start = data.frames[0].position;
        state.running_spline.add_waypoint(start);
    }

    // Do NOT add a connector to the child's position here. In the real
    // pipeline (geometry_builder.cpp), build_full_loop_chain is called
    // with the running spline ending at the previous segment's exit —
    // not at the current child's position. Adding a connector to the
    // child's position would cause loop_approach to be deduplicated
    // (since apex_entry == child position), masking bugs where the
    // approach rises above the crossover entry.

    // Claim crossover slots with directional bias along stitch_axis
    Vec3 travel_dir = data.frames[seg_id].stitch_axis;
    std::map<SegmentId, std::set<size_t>> claimed_slots;
    std::vector<CrossoverData> entry_xovers, exit_xovers;
    for (SegmentId parent_id : segments[seg_id].through) {
        auto parent_it = data.loops.find(parent_id);
        if (parent_it != data.loops.end() && !parent_it->second.crossover_slots.empty()) {
            auto entry = claim_nearest_slot(
                parent_it->second.crossover_slots, claimed_slots[parent_id],
                data.frames[seg_id].position, state.yarn_compressed_radius, true,
                data.frames[seg_id].wale_axis, travel_dir);
            entry_xovers.push_back(entry);

            if (segments[seg_id].forms_loop) {
                auto exit = claim_nearest_slot(
                    parent_it->second.crossover_slots, claimed_slots[parent_id],
                    data.frames[seg_id].position, state.yarn_compressed_radius, false,
                    data.frames[seg_id].wale_axis, travel_dir);
                exit_xovers.push_back(exit);
            }
        }
    }

    CatmullRomSpline segment_spline;
    std::vector<NamedWaypoint> named_waypoints;
    WaypointAddedCallback wp_cb = [&](const std::string& name, const Vec3& pos) {
        named_waypoints.push_back({name, pos});
    };
    build_full_loop_chain(state, segment_spline, geom, entry_xovers, exit_xovers, nullptr, wp_cb);
    return ChainBuildResult{std::move(segment_spline), std::move(entry_xovers), std::move(exit_xovers), std::move(named_waypoints)};
}

// ---------------------------------------------------------------------------
// Test 1: The apex region of the loop forms a smooth arch (no W-shape)
//
// Physical invariant: between the entry crossover dip and the exit crossover
// dip, the yarn rises to the apex and comes back down in a smooth arch.
// The wale projection should rise monotonically to the apex, then fall
// monotonically. A W-shape (two peaks with a valley in between) indicates
// the Hermite chain is overshooting at wrap waypoint transitions.
//
// Even a simple 2-row stockinette triggers this because the wrap waypoints
// create step-function jumps in the fabric_normal direction that the
// Hermite interpolation cannot follow smoothly.
// ---------------------------------------------------------------------------
TEST(LoopChainGeometryTest, ApexRegionIsSmoothlyCurved) {
    auto data = build_chain_data({"CCC", "KKK", "KKK"});
    const auto& segments = data.yarn_path.segments();

    int checked = 0;
    for (size_t i = 0; i < segments.size(); ++i) {
        if (!segments[i].forms_loop || segments[i].through.empty()) continue;
        SegmentId seg_id = static_cast<SegmentId>(i);

        auto loop_it = data.loops.find(seg_id);
        if (loop_it == data.loops.end()) continue;

        auto result = build_chain_for_segment(data, seg_id);
        if (result.spline.empty()) continue;
        const auto& spline = result.spline;

        Vec3 wale = data.frames[seg_id].wale_axis;
        Vec3 base = data.frames[seg_id].position;

        // Sample the wale projection densely
        std::vector<float> wale_samples;
        auto polyline = spline.to_polyline_fixed(50);
        for (const auto& pt : polyline) {
            wale_samples.push_back((pt - base).dot(wale));
        }
        ASSERT_GE(wale_samples.size(), 10u) << "Segment " << seg_id;

        // Find the global apex (highest wale projection)
        auto apex_it = std::max_element(wale_samples.begin(), wale_samples.end());
        size_t apex_idx = std::distance(wale_samples.begin(), apex_it);
        float w_apex = *apex_it;

        // The rise (start to apex) should be monotonically increasing.
        // Count significant direction reversals — each one is a valley/peak
        // that shouldn't be there. We use a threshold proportional to the
        // loop height so that sub-visual wiggles from the C2 spline are
        // ignored while genuine W-shapes (valleys > 10% of loop height)
        // are caught.
        float wale_range = w_apex - std::min(wale_samples.front(), wale_samples.back());
        float noise_threshold = wale_range * 0.10f;

        int rise_reversals = 0;
        float rise_max_so_far = wale_samples[0];
        for (size_t j = 1; j <= apex_idx; ++j) {
            if (wale_samples[j] > rise_max_so_far) {
                rise_max_so_far = wale_samples[j];
            } else if (rise_max_so_far - wale_samples[j] > noise_threshold) {
                rise_reversals++;
                rise_max_so_far = wale_samples[j];  // reset after detecting reversal
            }
        }

        // The fall (apex to end) should be monotonically decreasing.
        int fall_reversals = 0;
        float fall_min_so_far = wale_samples[apex_idx];
        for (size_t j = apex_idx + 1; j < wale_samples.size(); ++j) {
            if (wale_samples[j] < fall_min_so_far) {
                fall_min_so_far = wale_samples[j];
            } else if (wale_samples[j] - fall_min_so_far > noise_threshold) {
                fall_reversals++;
                fall_min_so_far = wale_samples[j];
            }
        }

        // Debug output
        float w_start = wale_samples.front();
        float w_end = wale_samples.back();
        std::cerr << "  Seg " << seg_id
            << " wale: start=" << w_start << " apex=" << w_apex << " end=" << w_end
            << " rise_reversals=" << rise_reversals
            << " fall_reversals=" << fall_reversals << "\n";

        // The entry/exit dips through the parent opening are expected reversals
        // (wale drops ~1 yarn diameter before rising). The C2 spline may also
        // produce small oscillations at scale. We allow a handful of reversals
        // but catch gross W-shapes (which had 20+ at 0.1mm threshold).
        EXPECT_LE(rise_reversals, 5)
            << "Segment " << seg_id << ": rise to apex has " << rise_reversals
            << " reversals (W-shape on entry side)";
        EXPECT_LE(fall_reversals, 5)
            << "Segment " << seg_id << ": fall from apex has " << fall_reversals
            << " reversals (W-shape on exit side)";

        checked++;
    }
    EXPECT_GT(checked, 0) << "Expected at least one loop-forming segment with parents";
}

// ---------------------------------------------------------------------------
// Test 2: Leg midpoints have z_bulge offset
// ---------------------------------------------------------------------------
TEST(LoopChainGeometryTest, LegMidpointsHaveZBulge) {
    auto data = build_chain_data({"CCC", "KKK"});
    const auto& segments = data.yarn_path.segments();

    for (const auto& [seg_id, geom] : data.loops) {
        if (!segments[seg_id].forms_loop) continue;

        // The precomputed geometry has fabric_normal and shape.z_bulge
        // Verify that z_bulge is nonzero for worked stitches
        if (segments[seg_id].work_type == WorkType::Worked &&
            segments[seg_id].orientation != LoopOrientation::Neutral) {
            EXPECT_NE(geom.shape.z_bulge, 0.0f)
                << "Segment " << seg_id << ": worked loop should have nonzero z_bulge";
        }
    }
}

// ---------------------------------------------------------------------------
// Test 3: Knit and purl have opposite z_bulge direction
// ---------------------------------------------------------------------------
TEST(LoopChainGeometryTest, KnitAndPurlOppositeBulge) {
    auto data = build_chain_data({"CCCC", "KKPP"});
    const auto& segments = data.yarn_path.segments();

    std::vector<float> knit_bulge, purl_bulge;
    for (const auto& [seg_id, geom] : data.loops) {
        if (segments[seg_id].orientation == LoopOrientation::Front) {
            knit_bulge.push_back(geom.shape.z_bulge);
        } else if (segments[seg_id].orientation == LoopOrientation::Back) {
            purl_bulge.push_back(geom.shape.z_bulge);
        }
    }

    EXPECT_FALSE(knit_bulge.empty()) << "Expected Front-oriented (knit) loops";
    EXPECT_FALSE(purl_bulge.empty()) << "Expected Back-oriented (purl) loops";

    for (float zb : knit_bulge) EXPECT_GT(zb, 0.0f)
        << "Knit (Front) loops should have positive z_bulge";
    for (float zb : purl_bulge) EXPECT_LT(zb, 0.0f)
        << "Purl (Back) loops should have negative z_bulge";
}

// ---------------------------------------------------------------------------
// Test 4: Wrap is on opposite side of children from z_bulge
// ---------------------------------------------------------------------------
TEST(LoopChainGeometryTest, WrapOppositeToChildBulge) {
    // Physical invariant: in stockinette, the visible V-shaped legs of a knit
    // stitch bulge toward the viewer (+fabric_normal when z_bulge > 0).
    // The wrap at the top of the loop — where the yarn turns around the
    // needle — must go behind the fabric (-fabric_normal for knit).
    // For purl, the reverse: legs bulge backward, wrap comes forward.
    //
    // 3 rows so the middle row's loops are both parents (have children) and
    // children (have parents), giving them nonzero z_bulge and crossover slots.
    auto data = build_chain_data({"CCC", "KKK", "KKK"});
    const auto& segments = data.yarn_path.segments();

    int checked = 0;
    for (size_t i = 0; i < segments.size(); ++i) {
        SegmentId seg_id = static_cast<SegmentId>(i);
        auto loop_it = data.loops.find(seg_id);
        if (loop_it == data.loops.end()) continue;
        const auto& geom = loop_it->second;
        if (geom.crossover_slots.empty() || geom.shape.z_bulge == 0.0f) continue;
        if (!segments[i].forms_loop || segments[i].through.empty()) continue;

        auto result = build_chain_for_segment(data, seg_id);
        if (result.spline.empty()) continue;
        const auto& spline = result.spline;

        Vec3 base = data.frames[seg_id].position;
        Vec3 wale = data.frames[seg_id].wale_axis;
        Vec3 fnormal = geom.fabric_normal;

        // The highest point along wale IS the wrap — the top of the U-shape
        // where yarn turns around the needle. In a correct loop, nothing
        // should be higher than the wrap.
        float max_wale = -1e10f;
        Vec3 wrap_point = base;
        auto wrap_polyline = spline.to_polyline_fixed(50);
        for (const auto& pt : wrap_polyline) {
            float w = (pt - base).dot(wale);
            if (w > max_wale) {
                max_wale = w;
                wrap_point = pt;
            }
        }

        // The wrap should be on the opposite side of the fabric from the legs.
        // z_bulge > 0 (knit): legs toward +fnormal, wrap toward -fnormal
        // z_bulge < 0 (purl): legs toward -fnormal, wrap toward +fnormal
        float wrap_fn = (wrap_point - base).dot(fnormal);
        if (geom.shape.z_bulge > 0.0f) {
            EXPECT_LT(wrap_fn, 0.0f)
                << "Segment " << seg_id << ": knit wrap should be behind fabric"
                << " (wrap_fn=" << wrap_fn << ")";
        } else {
            EXPECT_GT(wrap_fn, 0.0f)
                << "Segment " << seg_id << ": purl wrap should be in front of fabric"
                << " (wrap_fn=" << wrap_fn << ")";
        }
        checked++;
    }
    EXPECT_GT(checked, 0) << "Expected at least one loop with crossover slots and nonzero z_bulge";
}

// ---------------------------------------------------------------------------
// Test 5: Yarn crosses through parent loop opening
//
// Physical invariant: a child stitch's yarn must enter the parent loop from
// one side of the fabric, form its own loop, and exit back through the parent
// loop on the other side. This means the fabric_normal projection of the
// spline relative to the crossover slot center must change sign at least
// twice (once for entry crossing, once for exit crossing).
// ---------------------------------------------------------------------------
TEST(LoopChainGeometryTest, YarnCrossesThroughParentLoop) {
    auto data = build_chain_data({"CCC", "KKK", "KKK"});
    const auto& segments = data.yarn_path.segments();

    int checked = 0;
    for (size_t i = 0; i < segments.size(); ++i) {
        if (!segments[i].forms_loop || segments[i].through.empty()) continue;
        SegmentId seg_id = static_cast<SegmentId>(i);

        auto loop_it = data.loops.find(seg_id);
        if (loop_it == data.loops.end()) continue;

        auto result = build_chain_for_segment(data, seg_id);
        if (result.spline.empty()) continue;
        if (result.entry_crossovers.empty() && result.exit_crossovers.empty()) continue;

        // Use the first entry crossover as the reference point
        Vec3 xover_center = result.entry_crossovers[0].entry;
        if (!result.entry_crossovers.empty()) {
            // Midpoint between entry and exit of the crossover
            xover_center = (result.entry_crossovers[0].entry + result.entry_crossovers[0].exit) * 0.5f;
        }

        // Get the parent's fabric_normal
        SegmentId parent_id = segments[seg_id].through[0];
        auto parent_it = data.loops.find(parent_id);
        ASSERT_NE(parent_it, data.loops.end()) << "Parent loop geometry not found";
        Vec3 fnormal = parent_it->second.fabric_normal;

        // Sample the spline and compute fabric_normal projection relative to crossover center
        std::vector<float> fn_projections;
        auto fn_polyline = result.spline.to_polyline_fixed(50);
        for (const auto& pt : fn_polyline) {
            fn_projections.push_back((pt - xover_center).dot(fnormal));
        }

        // Count sign changes in the fabric_normal projection
        int sign_changes = 0;
        for (size_t j = 1; j < fn_projections.size(); ++j) {
            if (fn_projections[j-1] * fn_projections[j] < 0.0f) {
                sign_changes++;
            }
        }

        std::cerr << "  Seg " << seg_id << " fabric_normal sign changes: " << sign_changes << "\n";

        // The yarn must cross through the parent opening at least twice
        // (once entering, once exiting)
        EXPECT_GE(sign_changes, 2)
            << "Segment " << seg_id << ": yarn must thread through parent loop opening "
            << "(expected >= 2 fabric_normal sign changes, got " << sign_changes << ")";

        checked++;
    }
    EXPECT_GT(checked, 0) << "Expected at least one loop-forming segment with parents and crossovers";
}

// ---------------------------------------------------------------------------
// Test 7: Crossover entry/exit points are on consistent wale sides
//
// Physical invariant: the yarn enters the parent loop from below (−wale)
// and exits above (+wale) at the entry crossover. At the exit crossover,
// it enters from above (+wale) and exits below (−wale). So:
//   entry_in and exit_out are both on the −wale side (below the opening)
//   entry_out and exit_in are both on the +wale side (above the opening)
// ---------------------------------------------------------------------------
TEST(LoopChainGeometryTest, CrossoverPairsOnSameWaleSide) {
    auto data = build_chain_data({"CCC", "KKK", "KKK"});
    const auto& segments = data.yarn_path.segments();

    int checked = 0;
    for (size_t i = 0; i < segments.size(); ++i) {
        if (!segments[i].forms_loop || segments[i].through.empty()) continue;
        SegmentId seg_id = static_cast<SegmentId>(i);

        auto loop_it = data.loops.find(seg_id);
        if (loop_it == data.loops.end()) continue;

        auto result = build_chain_for_segment(data, seg_id);
        if (result.entry_crossovers.empty() || result.exit_crossovers.empty()) continue;

        // Use the child's wale_axis for wale projection
        Vec3 wale = data.frames[seg_id].wale_axis;

        for (size_t j = 0; j < result.entry_crossovers.size() && j < result.exit_crossovers.size(); ++j) {
            const auto& entry_xover = result.entry_crossovers[j];
            const auto& exit_xover = result.exit_crossovers[j];

            // Use each crossover's own center as the reference, since
            // entry and exit use different slots at potentially different
            // wale positions.
            Vec3 entry_ref = (entry_xover.entry + entry_xover.exit) * 0.5f;
            Vec3 exit_ref = (exit_xover.entry + exit_xover.exit) * 0.5f;

            float entry_in_proj = (entry_xover.entry - entry_ref).dot(wale);
            float entry_out_proj = (entry_xover.exit - entry_ref).dot(wale);
            float exit_in_proj = (exit_xover.entry - exit_ref).dot(wale);
            float exit_out_proj = (exit_xover.exit - exit_ref).dot(wale);

            std::cerr << "  Seg " << seg_id << " crossover " << j
                << ": entry_in=" << entry_in_proj << " entry_out=" << entry_out_proj
                << " exit_in=" << exit_in_proj << " exit_out=" << exit_out_proj << "\n";

            // entry_in and exit_out on same side (below, −wale)
            EXPECT_GT(entry_in_proj * exit_out_proj, 0.0f)
                << "Segment " << seg_id << " crossover " << j
                << ": entry_in and exit_out must be on the same wale side (below opening)";

            // entry_out and exit_in on same side (above, +wale)
            EXPECT_GT(entry_out_proj * exit_in_proj, 0.0f)
                << "Segment " << seg_id << " crossover " << j
                << ": entry_out and exit_in must be on the same wale side (above opening)";

            // The two pairs must be on opposite sides from each other
            EXPECT_LT(entry_in_proj * entry_out_proj, 0.0f)
                << "Segment " << seg_id << " crossover " << j
                << ": entry_in and entry_out must be on opposite wale sides";
        }

        checked++;
    }
    EXPECT_GT(checked, 0) << "Expected at least one segment with both entry and exit crossovers";
}

// ---------------------------------------------------------------------------
// Test 8: Waypoints progress along stitch_axis except in loop interior
//
// Physical invariant: the yarn travels along the stitch_axis (course
// direction). All consecutive waypoint pairs must have non-decreasing
// stitch_axis projection, EXCEPT when the yarn curves into and out of
// the loop body (the entry/exit legs and the crossover exit-in point).
// ---------------------------------------------------------------------------
TEST(LoopChainGeometryTest, WaypointsProgressAlongStitchAxis) {
    auto data = build_chain_data({"CCC", "KKK", "KKK"});
    const auto& segments = data.yarn_path.segments();

    // Waypoint names where backward motion (decreasing stitch_axis projection)
    // is expected — these are the loop interior transitions where the yarn
    // curves back through the parent opening.
    const std::set<std::string> backward_allowed = {
        "loop_entry_leg",
        "loop_wrap_approach",
        "loop_exit_leg",
        "crossover_exit_in"
    };

    int checked = 0;
    for (size_t i = 0; i < segments.size(); ++i) {
        if (!segments[i].forms_loop || segments[i].through.empty()) continue;
        SegmentId seg_id = static_cast<SegmentId>(i);

        auto loop_it = data.loops.find(seg_id);
        if (loop_it == data.loops.end()) continue;
        if (loop_it->second.crossover_slots.empty()) continue;

        // Only test segments that have children (wrap waypoints)
        auto children_it = data.children_map.find(seg_id);
        if (children_it == data.children_map.end() || children_it->second.empty()) {
            // This segment has no children — check if it has wrap waypoints
            // by building the chain and checking
        }

        auto result = build_chain_for_segment(data, seg_id);
        if (result.waypoints.size() < 2) continue;

        Vec3 stitch_axis = data.frames[seg_id].stitch_axis;
        Vec3 start = result.waypoints[0].position;

        // Debug: print frame alignment between child and parent
        for (SegmentId parent_id : segments[i].through) {
            Vec3 parent_fn = data.frames[parent_id].fabric_normal;
            Vec3 parent_sa = data.frames[parent_id].stitch_axis;
            float dot_fn_sa = parent_fn.dot(stitch_axis);
            std::cerr << "  Seg " << seg_id << " parent=" << parent_id
                << " child_sa=(" << stitch_axis.x << "," << stitch_axis.y << "," << stitch_axis.z << ")"
                << " parent_fn=(" << parent_fn.x << "," << parent_fn.y << "," << parent_fn.z << ")"
                << " parent_sa=(" << parent_sa.x << "," << parent_sa.y << "," << parent_sa.z << ")"
                << " parent_fn.dot(child_sa)=" << dot_fn_sa << "\n";
        }

        auto stitch_proj = [&](const Vec3& pt) -> float {
            return (pt - start).dot(stitch_axis);
        };

        // Tolerance for floating-point precision: fabric_normal is computed
        // via cross product which guarantees perpendicularity mathematically,
        // but normalization introduces sub-micron rounding errors.
        const float epsilon = 1e-4f;

        for (size_t j = 0; j + 1 < result.waypoints.size(); ++j) {
            const auto& wp_curr = result.waypoints[j];
            const auto& wp_next = result.waypoints[j + 1];

            // If the next waypoint is in the backward-allowed set, skip
            if (backward_allowed.count(wp_next.name)) continue;

            float proj_curr = stitch_proj(wp_curr.position);
            float proj_next = stitch_proj(wp_next.position);

            EXPECT_GE(proj_next, proj_curr - epsilon)
                << "Segment " << seg_id
                << ": waypoint '" << wp_next.name << "' (index " << (j + 1)
                << ") has stitch_proj " << proj_next
                << " < previous waypoint '" << wp_curr.name
                << "' stitch_proj " << proj_curr
                << " (backward motion not allowed here)";
        }

        // Additional check: the exit crossover must be ahead of the entry
        // crossover along stitch_axis. The consecutive-pair check above
        // cannot catch this because all intermediate loop-interior waypoints
        // are backward-allowed, masking the overall regression.
        auto find_wp = [&](const std::string& name) -> const Vec3* {
            for (const auto& wp : result.waypoints) {
                if (wp.name == name) return &wp.position;
            }
            return nullptr;
        };
        const Vec3* entry_in = find_wp("crossover_entry_in");
        const Vec3* exit_out = find_wp("crossover_exit_out");
        if (entry_in && exit_out) {
            EXPECT_GE(stitch_proj(*exit_out), stitch_proj(*entry_in) - epsilon)
                << "Segment " << seg_id
                << ": crossover_exit_out (stitch_proj=" << stitch_proj(*exit_out)
                << ") is behind crossover_entry_in (stitch_proj=" << stitch_proj(*entry_in)
                << ") — exit crossover must be ahead of entry along stitch_axis";
        }

        checked++;
    }
    EXPECT_GT(checked, 0) << "Expected at least one loop-forming segment with crossovers";
}

// ---------------------------------------------------------------------------
// Test 9: No waypoint between crossover_exit_out and crossover_entry_in
//         rises above crossover_entry_in in wale
//
// Physical invariant: between the exit of one crossover and the entry of the
// next, the yarn travels at or below the crossover entry level. If any
// intermediate waypoint (e.g. loop_approach) rises above crossover_entry_in
// in wale, the yarn creates an unphysical bump — rising to the child's
// position then dropping back down to enter the crossover.
//
// The previous version of this test only checked that crossover_entry_in was
// below loop_approach, which trivially passes (the crossover is always offset
// down by yarn_compressed_radius). The correct check is the opposite: no
// waypoint in the transition region should exceed the crossover entry level.
// ---------------------------------------------------------------------------
TEST(LoopChainGeometryTest, ThroughOpeningDipIsUShape) {
    auto data = build_chain_data({"CCC", "KKK", "KKK"});
    const auto& segments = data.yarn_path.segments();

    const float epsilon = 1e-4f;

    int checked = 0;
    for (size_t i = 0; i < segments.size(); ++i) {
        if (!segments[i].forms_loop || segments[i].through.empty()) continue;
        SegmentId seg_id = static_cast<SegmentId>(i);

        auto loop_it = data.loops.find(seg_id);
        if (loop_it == data.loops.end()) continue;
        if (loop_it->second.crossover_slots.empty()) continue;

        auto result = build_chain_for_segment(data, seg_id);
        if (result.waypoints.size() < 3) continue;

        Vec3 wale = data.frames[seg_id].wale_axis;
        Vec3 base = data.frames[seg_id].position;

        auto wale_proj = [&](const Vec3& pt) -> float {
            return (pt - base).dot(wale);
        };

        // Collect waypoint indices for crossover boundaries
        // We check: all waypoints between crossover_exit_out and the next
        // crossover_entry_in must have wale <= wale(crossover_entry_in).
        // For the first crossover, we also check waypoints from the start
        // up to crossover_entry_in.

        // Find all crossover_exit_out and crossover_entry_in indices
        std::vector<size_t> exit_out_indices;
        std::vector<size_t> entry_in_indices;
        for (size_t j = 0; j < result.waypoints.size(); ++j) {
            if (result.waypoints[j].name == "crossover_exit_out")
                exit_out_indices.push_back(j);
            if (result.waypoints[j].name == "crossover_entry_in")
                entry_in_indices.push_back(j);
        }

        if (entry_in_indices.empty()) continue;

        // For the first crossover_entry_in, check all waypoints from
        // index 0 up to (but not including) the entry_in itself
        {
            size_t end_idx = entry_in_indices[0];
            float w_entry_in = wale_proj(result.waypoints[end_idx].position);

            for (size_t j = 0; j < end_idx; ++j) {
                float w_wp = wale_proj(result.waypoints[j].position);
                std::cerr << "  Seg " << seg_id
                    << " [before first entry_in] wp[" << j << "] '"
                    << result.waypoints[j].name
                    << "' wale=" << w_wp
                    << " crossover_entry_in wale=" << w_entry_in << "\n";

                EXPECT_LE(w_wp, w_entry_in + epsilon)
                    << "Segment " << seg_id
                    << ": waypoint '" << result.waypoints[j].name
                    << "' (wale=" << w_wp
                    << ") is above crossover_entry_in (wale=" << w_entry_in
                    << ") — yarn bumps up before entering crossover";
            }
        }

        // For each crossover_exit_out, check waypoints up to the next
        // crossover_entry_in
        for (size_t ex = 0; ex < exit_out_indices.size(); ++ex) {
            size_t start_idx = exit_out_indices[ex];

            // Find the next crossover_entry_in after this exit_out
            size_t end_idx = result.waypoints.size(); // default: end of list
            float w_ceiling = std::numeric_limits<float>::max();
            for (size_t ei : entry_in_indices) {
                if (ei > start_idx) {
                    end_idx = ei;
                    w_ceiling = wale_proj(result.waypoints[ei].position);
                    break;
                }
            }
            if (w_ceiling == std::numeric_limits<float>::max()) continue;

            for (size_t j = start_idx + 1; j < end_idx; ++j) {
                float w_wp = wale_proj(result.waypoints[j].position);
                std::cerr << "  Seg " << seg_id
                    << " [exit_out.." << "entry_in] wp[" << j << "] '"
                    << result.waypoints[j].name
                    << "' wale=" << w_wp
                    << " crossover_entry_in wale=" << w_ceiling << "\n";

                EXPECT_LE(w_wp, w_ceiling + epsilon)
                    << "Segment " << seg_id
                    << ": waypoint '" << result.waypoints[j].name
                    << "' (wale=" << w_wp
                    << ") is above next crossover_entry_in (wale=" << w_ceiling
                    << ") — yarn bumps up between crossovers";
            }
        }

        checked++;
    }
    EXPECT_GT(checked, 0) << "Expected at least one loop-forming segment with crossovers";
}

// ---------------------------------------------------------------------------
// Test 10: Crossover entry/exit have correct wale direction
//
// Physical invariant: at each crossover, the yarn passes through the parent
// loop opening in a consistent wale direction. At the entry crossover, the
// yarn goes upward (from below the parent loop into the loop body):
//   wale_proj(crossover_entry_in) < wale_proj(crossover_entry_out)
// At the exit crossover, the yarn goes downward (from the loop body back
// below the parent loop):
//   wale_proj(crossover_exit_in) > wale_proj(crossover_exit_out)
// If these are reversed, the spline must loop around at the crossover,
// creating an unphysical zigzag.
// ---------------------------------------------------------------------------
TEST(LoopChainGeometryTest, CrossoverWaleDirectionIsCorrect) {
    auto data = build_chain_data({"CCC", "KKK", "KKK"});
    const auto& segments = data.yarn_path.segments();

    int checked = 0;
    for (size_t i = 0; i < segments.size(); ++i) {
        if (!segments[i].forms_loop || segments[i].through.empty()) continue;
        SegmentId seg_id = static_cast<SegmentId>(i);

        auto loop_it = data.loops.find(seg_id);
        if (loop_it == data.loops.end()) continue;
        if (loop_it->second.crossover_slots.empty()) continue;

        auto result = build_chain_for_segment(data, seg_id);

        Vec3 wale = data.frames[seg_id].wale_axis;
        Vec3 base = data.frames[seg_id].position;

        auto wale_proj = [&](const Vec3& pt) -> float {
            return (pt - base).dot(wale);
        };

        auto find_wp = [&](const std::string& name) -> const Vec3* {
            for (const auto& wp : result.waypoints) {
                if (wp.name == name) return &wp.position;
            }
            return nullptr;
        };

        // Entry crossover: yarn goes upward through parent loop
        const Vec3* entry_in = find_wp("crossover_entry_in");
        const Vec3* entry_out = find_wp("crossover_entry_out");
        if (entry_in && entry_out) {
            EXPECT_LT(wale_proj(*entry_in), wale_proj(*entry_out))
                << "Segment " << seg_id
                << ": entry crossover should go upward in wale"
                << " (entry_in wale=" << wale_proj(*entry_in)
                << ", entry_out wale=" << wale_proj(*entry_out) << ")";
        }

        // Exit crossover: yarn goes downward through parent loop
        const Vec3* exit_in = find_wp("crossover_exit_in");
        const Vec3* exit_out = find_wp("crossover_exit_out");
        if (exit_in && exit_out) {
            EXPECT_GT(wale_proj(*exit_in), wale_proj(*exit_out))
                << "Segment " << seg_id
                << ": exit crossover should go downward in wale"
                << " (exit_in wale=" << wale_proj(*exit_in)
                << ", exit_out wale=" << wale_proj(*exit_out) << ")";
        }

        checked++;
    }
    EXPECT_GT(checked, 0) << "Expected at least one loop-forming segment with crossovers";
}

// ---------------------------------------------------------------------------
// Test 11: loop_approach and loop_depart clear the parent yarn body
//
// Physical invariant: the child yarn body must not overlap the parent yarn
// body at the approach and departure points. The crossover entry/exit points
// are at the parent yarn's bottom edge (one radius below slot center). The
// approach/depart must be at least one additional radius below that so the
// child yarn's top edge clears the parent yarn's bottom edge.
// ---------------------------------------------------------------------------
TEST(LoopChainGeometryTest, ApproachAndDepartClearParentYarn) {
    auto data = build_chain_data({"CCC", "KKK", "KKK"});
    const auto& segments = data.yarn_path.segments();

    const float epsilon = 1e-4f;

    int checked = 0;
    for (size_t i = 0; i < segments.size(); ++i) {
        if (!segments[i].forms_loop || segments[i].through.empty()) continue;
        SegmentId seg_id = static_cast<SegmentId>(i);

        auto loop_it = data.loops.find(seg_id);
        if (loop_it == data.loops.end()) continue;
        if (loop_it->second.crossover_slots.empty()) continue;

        auto result = build_chain_for_segment(data, seg_id);
        if (result.waypoints.empty()) continue;

        auto find_wp = [&](const std::string& name) -> const Vec3* {
            for (const auto& wp : result.waypoints) {
                if (wp.name == name) return &wp.position;
            }
            return nullptr;
        };

        // --- Entry side ---
        const Vec3* entry_in = find_wp("crossover_entry_in");
        const Vec3* entry_out = find_wp("crossover_entry_out");
        const Vec3* approach = find_wp("loop_approach");
        if (entry_in && entry_out && approach) {
            Vec3 wale_dir = safe_normalized(*entry_out - *entry_in);
            float radius = (*entry_out - *entry_in).length() / 2.0f;

            // Project approach relative to entry_in onto the crossing wale direction
            float approach_proj = (*approach - *entry_in).dot(wale_dir);

            std::cerr << "  Seg " << seg_id
                << " entry: approach_proj=" << approach_proj
                << " -radius=" << -radius << "\n";

            EXPECT_LE(approach_proj, -radius + epsilon)
                << "Segment " << seg_id
                << ": loop_approach (proj=" << approach_proj
                << ") must be at least one radius (" << radius
                << ") below crossover_entry_in to clear parent yarn body";
        }

        // --- Exit side ---
        const Vec3* exit_in = find_wp("crossover_exit_in");
        const Vec3* exit_out = find_wp("crossover_exit_out");
        const Vec3* depart = find_wp("loop_depart");
        if (exit_in && exit_out && depart) {
            Vec3 exit_wale_dir = safe_normalized(*exit_in - *exit_out);
            float radius = (*exit_in - *exit_out).length() / 2.0f;

            // Project depart relative to exit_out onto the exit wale direction
            float depart_proj = (*depart - *exit_out).dot(exit_wale_dir);

            std::cerr << "  Seg " << seg_id
                << " exit: depart_proj=" << depart_proj
                << " -radius=" << -radius << "\n";

            EXPECT_LE(depart_proj, -radius + epsilon)
                << "Segment " << seg_id
                << ": loop_depart (proj=" << depart_proj
                << ") must be at least one radius (" << radius
                << ") below crossover_exit_out to clear parent yarn body";
        }

        checked++;
    }
    EXPECT_GT(checked, 0) << "Expected at least one loop-forming segment with crossovers";
}

// ---------------------------------------------------------------------------
// Test 12: Leg waypoints are between crossover and wrap in stitch_axis distance
//
// Physical invariant: the legs of the loop connect the crossover at the base
// to the wrap at the top. The stitch_axis distance of each leg endpoint must
// be between the crossover spread and the wrap spread.
// ---------------------------------------------------------------------------
TEST(LoopChainGeometryTest, LegWidthBetweenCrossoverAndWrap) {
    auto data = build_chain_data({"CCC", "KKK", "KKK"});
    const auto& segments = data.yarn_path.segments();

    int checked = 0;
    for (size_t i = 0; i < segments.size(); ++i) {
        if (!segments[i].forms_loop || segments[i].through.empty()) continue;
        SegmentId seg_id = static_cast<SegmentId>(i);

        auto loop_it = data.loops.find(seg_id);
        if (loop_it == data.loops.end()) continue;
        if (loop_it->second.crossover_slots.empty()) continue;

        auto result = build_chain_for_segment(data, seg_id);
        if (result.waypoints.empty()) continue;
        if (result.entry_crossovers.empty() || result.exit_crossovers.empty()) continue;

        // Find waypoints by name
        auto find_wp = [&](const std::string& name) -> const Vec3* {
            for (const auto& wp : result.waypoints) {
                if (wp.name == name) return &wp.position;
            }
            return nullptr;
        };

        const Vec3* entry_leg = find_wp("loop_entry_leg");
        const Vec3* exit_leg = find_wp("loop_exit_leg");
        const Vec3* wrap_entry = find_wp("loop_wrap_entry");
        const Vec3* wrap_exit = find_wp("loop_wrap_exit");
        const Vec3* xover_entry_out = find_wp("crossover_entry_out");
        const Vec3* xover_exit_in = find_wp("crossover_exit_in");

        if (!entry_leg || !exit_leg || !wrap_entry || !wrap_exit ||
            !xover_entry_out || !xover_exit_in) continue;

        float leg_dist = (*entry_leg - *exit_leg).length();
        float wrap_dist = (*wrap_entry - *wrap_exit).length();
        float xover_dist = (*xover_entry_out - *xover_exit_in).length();

        std::cerr << "  Seg " << seg_id
            << " xover_dist=" << xover_dist
            << " leg_dist=" << leg_dist
            << " wrap_dist=" << wrap_dist << "\n";

        EXPECT_GE(leg_dist, xover_dist)
            << "Segment " << seg_id
            << ": leg width must be >= crossover width";
        EXPECT_LE(leg_dist, wrap_dist)
            << "Segment " << seg_id
            << ": leg width must be <= wrap width";

        checked++;
    }
    EXPECT_GT(checked, 0) << "Expected at least one segment with legs, wraps, and crossovers";
}

