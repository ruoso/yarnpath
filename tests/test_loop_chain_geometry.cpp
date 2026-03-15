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

// Helper: initialize state and build chain for a specific loop segment
static BezierSpline build_chain_for_segment(
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
    build_full_loop_chain(state, segment_spline, geom, entry_xovers, exit_xovers, nullptr);
    return segment_spline;
}

// ---------------------------------------------------------------------------
// Test 1: Chain forms a U-shape (wale projection goes down then up then down)
// ---------------------------------------------------------------------------
TEST(LoopChainGeometryTest, ChainFormsUShape) {
    auto data = build_chain_data({"CCC", "KKK"});
    const auto& segments = data.yarn_path.segments();

    // Find first loop-forming segment in row 1
    SegmentId target_id = 0;
    bool found = false;
    for (size_t i = 0; i < segments.size(); ++i) {
        if (segments[i].forms_loop && !segments[i].through.empty()) {
            target_id = static_cast<SegmentId>(i);
            found = true;
            break;
        }
    }
    if (!found) {
        GTEST_SKIP() << "No loop-forming segment with parents found";
    }

    auto spline = build_chain_for_segment(data, target_id);
    ASSERT_FALSE(spline.empty()) << "Chain should produce at least one curve";

    // The U-shape is along the wale axis derived from the surface frame.
    // Project spline samples onto the surface-derived wale to detect the U.
    Vec3 wale = data.frames[target_id].wale_axis;
    Vec3 base = data.frames[target_id].position;

    std::vector<float> wale_samples;
    for (const auto& seg : spline.segments()) {
        for (float t = 0.0f; t <= 1.0f; t += 0.1f) {
            wale_samples.push_back((seg.evaluate(t) - base).dot(wale));
        }
    }

    // Find minimum and maximum wale projection
    float w_min = *std::min_element(wale_samples.begin(), wale_samples.end());
    float w_max = *std::max_element(wale_samples.begin(), wale_samples.end());
    float w_start = wale_samples.front();

    // U-shape: the spline should extend on both sides of the start point
    // along the wale axis (dip on one side, apex on the other).
    float w_range = w_max - w_min;
    EXPECT_GT(w_range, 0.5f) << "U-shape should have significant extent along wale";

    // The samples should span both sides of w_start (dip below, apex above)
    EXPECT_GT(w_max, w_start + 0.1f) << "Apex should be above start along wale";
    EXPECT_LT(w_min, w_start - 0.1f) << "Dip should be below start along wale";
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

        auto spline = build_chain_for_segment(data, seg_id);
        if (spline.empty()) continue;

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

    auto spline = build_chain_for_segment(data, target_id);
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
