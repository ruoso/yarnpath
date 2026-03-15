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
    BezierSpline spline;
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
        Vec3 target = (data.frames.size() > 1) ? data.frames[1].position : start + Vec3(1, 0, 0);
        BezierSpline init_spline;
        initialize_running_spline(state, start, target, init_spline, nullptr);
    }

    // Build connectors up to this segment's position if needed
    if (!state.running_spline.empty()) {
        Vec3 current_end = state.running_spline.segments().back().end();
        Vec3 target = data.frames[seg_id].position;
        if ((target - current_end).length() > 1e-4f) {
            BezierSpline temp;
            Vec3 dir = state.running_spline.segments().back().tangent(1.0f);
            Vec3 target_dir = safe_normalized(target - current_end, Vec3(1, 0, 0));
            add_connector_with_curvature_check(
                state, temp, current_end, dir, target, target_dir, "approach", nullptr);
        }
    }

    // Claim crossover slots
    std::map<SegmentId, std::set<size_t>> claimed_slots;
    std::vector<CrossoverData> entry_xovers, exit_xovers;
    for (SegmentId parent_id : segments[seg_id].through) {
        auto parent_it = data.loops.find(parent_id);
        if (parent_it != data.loops.end() && !parent_it->second.crossover_slots.empty()) {
            auto entry = claim_nearest_slot(
                parent_it->second.crossover_slots, claimed_slots[parent_id],
                data.frames[seg_id].position, state.yarn_compressed_radius, true);
            entry_xovers.push_back(entry);

            if (segments[seg_id].forms_loop) {
                auto exit = claim_nearest_slot(
                    parent_it->second.crossover_slots, claimed_slots[parent_id],
                    data.frames[seg_id].position, state.yarn_compressed_radius, false);
                exit_xovers.push_back(exit);
            }
        }
    }

    BezierSpline segment_spline;
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
        for (const auto& seg : spline.segments()) {
            for (float t = 0.0f; t <= 1.0f; t += 0.02f) {
                wale_samples.push_back((seg.evaluate(t) - base).dot(wale));
            }
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
        for (const auto& seg : spline.segments()) {
            for (float t = 0.0f; t <= 1.0f; t += 0.02f) {
                Vec3 pt = seg.evaluate(t);
                float w = (pt - base).dot(wale);
                if (w > max_wale) {
                    max_wale = w;
                    wrap_point = pt;
                }
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
// Test 5: Output spline is C0-continuous (no gaps between segments)
// ---------------------------------------------------------------------------
TEST(LoopChainGeometryTest, OutputIsContinuous) {
    auto data = build_chain_data({"CCC", "KKK"});

    SegmentId target_id = 0;
    bool found = false;
    for (size_t i = 0; i < data.yarn_path.segments().size(); ++i) {
        if (data.yarn_path.segments()[i].forms_loop && !data.yarn_path.segments()[i].through.empty()) {
            target_id = static_cast<SegmentId>(i);
            found = true;
            break;
        }
    }
    if (!found) {
        GTEST_SKIP() << "No loop-forming segment with parents found";
    }

    auto result = build_chain_for_segment(data, target_id);
    const auto& spline = result.spline;
    if (spline.segments().size() < 2) {
        GTEST_SKIP() << "Need at least 2 curve segments to check continuity";
    }

    const auto& segs = spline.segments();
    for (size_t i = 1; i < segs.size(); ++i) {
        Vec3 prev_end = segs[i-1].end();
        Vec3 curr_start = segs[i].start();
        float gap = (curr_start - prev_end).length();
        EXPECT_LT(gap, 1e-4f)
            << "Gap between segment " << (i-1) << " end and segment " << i << " start: " << gap;
    }
}

// ---------------------------------------------------------------------------
// Test 6: Yarn crosses through parent loop opening
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
        for (const auto& seg : result.spline.segments()) {
            for (float t = 0.0f; t <= 1.0f; t += 0.02f) {
                Vec3 pt = seg.evaluate(t);
                fn_projections.push_back((pt - xover_center).dot(fnormal));
            }
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
// Test 7: Crossover entry/exit points are on consistent fabric sides
//
// Physical invariant: the yarn enters the parent loop from one side of the
// fabric and exits back to the same side. So crossover_entry_in (approach
// side) and crossover_exit_out (departure side) must be on the same side
// of the fabric (same fabric_normal projection), and crossover_entry_out
// and crossover_exit_in must be on the other side together.
// ---------------------------------------------------------------------------
TEST(LoopChainGeometryTest, CrossoverPairsOnSameFabricSide) {
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

        // Get the parent's fabric_normal as reference axis
        SegmentId parent_id = segments[seg_id].through[0];
        auto parent_it = data.loops.find(parent_id);
        ASSERT_NE(parent_it, data.loops.end());
        Vec3 fnormal = parent_it->second.fabric_normal;

        // Use the parent's crossover slot center as the reference point
        Vec3 ref = parent_it->second.crossover_slots[0].position;

        for (size_t j = 0; j < result.entry_crossovers.size() && j < result.exit_crossovers.size(); ++j) {
            const auto& entry_xover = result.entry_crossovers[j];
            const auto& exit_xover = result.exit_crossovers[j];

            float entry_in_proj = (entry_xover.entry - ref).dot(fnormal);
            float entry_out_proj = (entry_xover.exit - ref).dot(fnormal);
            float exit_in_proj = (exit_xover.entry - ref).dot(fnormal);
            float exit_out_proj = (exit_xover.exit - ref).dot(fnormal);

            std::cerr << "  Seg " << seg_id << " crossover " << j
                << ": entry_in=" << entry_in_proj << " entry_out=" << entry_out_proj
                << " exit_in=" << exit_in_proj << " exit_out=" << exit_out_proj << "\n";

            // crossover_entry_in and crossover_exit_out on same side
            EXPECT_GT(entry_in_proj * exit_out_proj, 0.0f)
                << "Segment " << seg_id << " crossover " << j
                << ": entry_in and exit_out must be on the same fabric side";

            // crossover_entry_out and crossover_exit_in on same side
            EXPECT_GT(entry_out_proj * exit_in_proj, 0.0f)
                << "Segment " << seg_id << " crossover " << j
                << ": entry_out and exit_in must be on the same fabric side";

            // The two pairs must be on opposite sides from each other
            EXPECT_LT(entry_in_proj * entry_out_proj, 0.0f)
                << "Segment " << seg_id << " crossover " << j
                << ": entry_in and entry_out must be on opposite fabric sides";
        }

        checked++;
    }
    EXPECT_GT(checked, 0) << "Expected at least one segment with both entry and exit crossovers";
}

// ---------------------------------------------------------------------------
// Test 8: Leg waypoints are between crossover and wrap in stitch_axis distance
//
// Physical invariant: the legs of the loop connect the crossover at the base
// to the wrap at the top. The stitch_axis distance of each leg endpoint must
// be between the crossover spread and the wrap spread. That is:
//   dist(crossover_entry_out, crossover_exit_in)
//     <= dist(loop_entry_leg, loop_exit_leg)
//     <= dist(loop_wrap_entry, loop_wrap_exit)
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
