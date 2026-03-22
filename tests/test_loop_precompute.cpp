#include <gtest/gtest.h>
#include <geometry/loop_precompute.hpp>
#include <geometry/geometry_build_state.hpp>
#include <geometry/position_resolver.hpp>
#include <surface/surface_builder.hpp>
#include <surface/surface_solver.hpp>
#include "test_helpers.hpp"

#include <cmath>
#include <map>

using namespace yarnpath;
using namespace yarnpath::test;

// ---------------------------------------------------------------------------
// Helper: build pipeline through precomputed loop geometry
// ---------------------------------------------------------------------------
struct LoopPrecomputeTestData {
    YarnPath yarn_path;
    SurfaceGraph surface;
    std::vector<SegmentFrame> frames;
    std::map<SegmentId, PrecomputedLoopGeometry> loops;
    std::map<SegmentId, std::vector<SegmentId>> parent_map;
    std::map<SegmentId, std::vector<SegmentId>> children_map;
    float effective_loop_height;
    float yarn_compressed_diameter;
    float yarn_compressed_radius;
};

static LoopPrecomputeTestData build_loop_data(
    const std::vector<std::string>& rows,
    const YarnProperties& yarn,
    const Gauge& gauge) {

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

    // Build child position and child ID maps (same as geometry_builder)
    std::map<SegmentId, std::vector<Vec3>> loop_child_positions;
    std::map<SegmentId, std::vector<SegmentId>> loop_child_ids;
    for (size_t i = 0; i < segments.size(); ++i) {
        SegmentId child_id = static_cast<SegmentId>(i);
        for (SegmentId pid : segments[i].through) {
            loop_child_positions[pid].push_back(frames[i].position);
            loop_child_ids[pid].push_back(child_id);
        }
    }

    GeometryBuildState state(yarn, gauge);

    auto loops = precompute_loop_geometry(
        segments, frames, loop_child_positions, loop_child_ids,
        yarn, gauge,
        state.effective_loop_height,
        state.yarn_compressed_diameter, state.yarn_compressed_radius);

    return LoopPrecomputeTestData{
        std::move(yarn_path), std::move(surface), std::move(frames),
        std::move(loops), std::move(parent_map), std::move(children_map),
        state.effective_loop_height, state.yarn_compressed_diameter,
        state.yarn_compressed_radius
    };
}

static LoopPrecomputeTestData build_loop_data(const std::vector<std::string>& rows) {
    return build_loop_data(rows, default_yarn(), default_gauge());
}


// ---------------------------------------------------------------------------
// Test 1: Apex is above base in wale direction
// ---------------------------------------------------------------------------
TEST(LoopPrecomputeTest, ApexAboveBase) {
    auto data = build_loop_data({"CCC", "KKK"});

    int loops_checked = 0;
    for (const auto& [seg_id, geom] : data.loops) {
        // The apex must be displaced from the base along the wale axis.
        // The frame's wale_axis encodes the surface orientation; the apex
        // should have a nonzero wale projection with consistent sign.
        Vec3 curr_pos = data.frames[seg_id].position;
        Vec3 wale = data.frames[seg_id].wale_axis;
        float apex_wale_proj = (geom.apex - curr_pos).dot(wale);
        EXPECT_NE(apex_wale_proj, 0.0f)
            << "Segment " << seg_id << ": apex should be displaced from base along wale";

        // The dip should be on the opposite side of the base from the apex
        float dip_wale_proj = (geom.entry_through - curr_pos).dot(wale);
        EXPECT_TRUE((apex_wale_proj > 0.0f && dip_wale_proj < 0.0f) ||
                    (apex_wale_proj < 0.0f && dip_wale_proj > 0.0f))
            << "Segment " << seg_id << ": apex and dip should be on opposite sides of base"
            << " (apex_proj=" << apex_wale_proj << ", dip_proj=" << dip_wale_proj << ")";
        loops_checked++;
    }
    EXPECT_GT(loops_checked, 0) << "Expected at least one loop";
}

// ---------------------------------------------------------------------------
// Test 2: Apex is at child wale height when children exist
// ---------------------------------------------------------------------------
TEST(LoopPrecomputeTest, ApexAtChildWaleHeight) {
    auto data = build_loop_data({"CCC", "KKK"});

    int checked = 0;
    for (const auto& [seg_id, geom] : data.loops) {
        auto child_it = data.children_map.find(seg_id);
        if (child_it == data.children_map.end() || child_it->second.empty()) continue;

        // Skip parentless segments — their apex is intentionally flat
        // (foundation, YO, M1L/M1R loops stay near the base).
        const auto& seg = data.yarn_path.segments()[seg_id];
        if (seg.through.empty()) continue;

        // Apex wale projection should match the average child wale projection.
        // The child stitch sits at the top of the parent loop — the apex
        // should be AT the child position. 3D wrapping around the child
        // is handled by build_full_loop_chain().
        Vec3 wale = data.frames[seg_id].wale_axis;
        Vec3 base = data.frames[seg_id].position;
        float avg_child_wale = 0.0f;
        for (SegmentId child_id : child_it->second) {
            avg_child_wale += (data.frames[child_id].position - base).dot(wale);
        }
        avg_child_wale /= static_cast<float>(child_it->second.size());

        float apex_wale = (geom.apex - base).dot(wale);
        // Apex wale projection should match average child wale projection
        EXPECT_NEAR(apex_wale, avg_child_wale, 0.05f)
            << "Segment " << seg_id << ": apex should be at child wale height"
            << " (apex=" << apex_wale << ", avg_child=" << avg_child_wale << ")";
        checked++;
    }
    EXPECT_GT(checked, 0) << "Expected at least one loop with children";
}

// ---------------------------------------------------------------------------
// Test 3: Apex height scales with gauge
// ---------------------------------------------------------------------------
TEST(LoopPrecomputeTest, ApexHeightScalesWithGauge) {
    YarnProperties yarn = default_yarn();

    Gauge small_gauge{3.0f};  // 3mm needles
    Gauge large_gauge{8.0f};  // 8mm needles

    auto data_small = build_loop_data({"CCC", "KKK"}, yarn, small_gauge);
    auto data_large = build_loop_data({"CCC", "KKK"}, yarn, large_gauge);

    // Find corresponding loop segments and compare apex heights
    // Both should have loops at the same segment indices
    for (const auto& [seg_id, geom_small] : data_small.loops) {
        auto it = data_large.loops.find(seg_id);
        if (it == data_large.loops.end()) continue;
        const auto& geom_large = it->second;

        // Compare absolute wale projection magnitude (sign may differ between gauges)
        Vec3 wale_s = data_small.frames[seg_id].wale_axis;
        Vec3 wale_l = data_large.frames[seg_id].wale_axis;
        float height_small = std::abs((geom_small.apex - data_small.frames[seg_id].position).dot(wale_s));
        float height_large = std::abs((geom_large.apex - data_large.frames[seg_id].position).dot(wale_l));

        EXPECT_GT(height_large, height_small)
            << "Segment " << seg_id << ": larger gauge should produce taller loops";
    }
}

// ---------------------------------------------------------------------------
// Test 4: Apex lean for decreases (K2tog leans right, SSK leans left)
// ---------------------------------------------------------------------------
TEST(LoopPrecomputeTest, ApexLeanForDecreases) {
    // 5 cast-on, row with K2tog(2) + SSK(2) + Knit(1) = 5 consumed
    auto data = build_loop_data({"CCCCC", "2SK"});
    const auto& segments = data.yarn_path.segments();

    // Find segments by wrap direction
    std::vector<SegmentId> k2tog_ids, ssk_ids;
    for (const auto& [seg_id, geom] : data.loops) {
        if (segments[seg_id].wrap_direction == WrapDirection::Clockwise) {
            k2tog_ids.push_back(seg_id);
        } else if (segments[seg_id].wrap_direction == WrapDirection::CounterClockwise) {
            ssk_ids.push_back(seg_id);
        }
    }

    // Verify lean directions: K2tog should lean right (+stitch_axis), SSK left (-stitch_axis)
    for (SegmentId id : k2tog_ids) {
        EXPECT_GT(data.loops.at(id).shape.apex_lean_x, 0.0f)
            << "K2tog should have positive apex_lean_x";
    }
    for (SegmentId id : ssk_ids) {
        EXPECT_LT(data.loops.at(id).shape.apex_lean_x, 0.0f)
            << "SSK should have negative apex_lean_x";
    }

    // Equal magnitudes
    if (!k2tog_ids.empty() && !ssk_ids.empty()) {
        float k2tog_mag = std::abs(data.loops.at(k2tog_ids[0]).shape.apex_lean_x);
        float ssk_mag = std::abs(data.loops.at(ssk_ids[0]).shape.apex_lean_x);
        EXPECT_NEAR(k2tog_mag, ssk_mag, 0.01f)
            << "K2tog and SSK lean magnitudes should be equal";
    }
}

// ---------------------------------------------------------------------------
// Test 5: Slip stitch apex is taller than knit stitch apex
// ---------------------------------------------------------------------------
TEST(LoopPrecomputeTest, SlipApexTaller) {
    auto data = build_loop_data({"CC", "KL"});
    const auto& segments = data.yarn_path.segments();

    // Separate knit (Worked) and slip (Transferred) loops
    float knit_height = 0.0f;
    float slip_height = 0.0f;
    int knit_count = 0, slip_count = 0;

    for (const auto& [seg_id, geom] : data.loops) {
        Vec3 wale = data.frames[seg_id].wale_axis;
        float height = std::abs((geom.apex - data.frames[seg_id].position).dot(wale));
        if (segments[seg_id].work_type == WorkType::Transferred) {
            slip_height += height;
            slip_count++;
        } else if (segments[seg_id].work_type == WorkType::Worked &&
                   segments[seg_id].orientation != LoopOrientation::Neutral) {
            knit_height += height;
            knit_count++;
        }
    }

    if (knit_count > 0 && slip_count > 0) {
        knit_height /= knit_count;
        slip_height /= slip_count;
        EXPECT_GT(slip_height, knit_height)
            << "Slip stitch apex should be taller than knit stitch apex";
    } else {
        // At least verify we found the expected stitch types
        EXPECT_GT(knit_count + slip_count, 0) << "Expected knit or slip loops in pattern";
    }
}

// ---------------------------------------------------------------------------
// Test 6: Dip points are below base in wale direction
// ---------------------------------------------------------------------------
TEST(LoopPrecomputeTest, DipBelowBase) {
    auto data = build_loop_data({"CCC", "KKK"});

    int checked = 0;
    for (const auto& [seg_id, geom] : data.loops) {
        Vec3 curr_pos = data.frames[seg_id].position;
        Vec3 wale = data.frames[seg_id].wale_axis;

        // Dip should be on the opposite side of base from the apex
        float apex_proj = (geom.apex - curr_pos).dot(wale);
        float entry_dip = (geom.entry_through - curr_pos).dot(wale);
        EXPECT_TRUE(apex_proj * entry_dip < 0.0f)
            << "Segment " << seg_id << ": entry dip and apex should be on opposite sides of base"
            << " (apex_proj=" << apex_proj << ", dip_proj=" << entry_dip << ")";

        // exit_through is relative to next_pos (the next segment's position in the
        // yarn path), since the loop's exit leg terminates there.
        size_t next_i = seg_id + 1;
        Vec3 next_pos = (next_i < data.frames.size()) ? data.frames[next_i].position : curr_pos;
        float exit_dip = (geom.exit_through - next_pos).dot(wale);
        EXPECT_TRUE(apex_proj * exit_dip < 0.0f)
            << "Segment " << seg_id << ": exit dip and apex should be on opposite sides"
            << " (apex_proj=" << apex_proj << ", exit_dip=" << exit_dip << ")";

        checked++;
    }
    EXPECT_GT(checked, 0);
}

// ---------------------------------------------------------------------------
// Test 7: Dip depth equals yarn_compressed_diameter
// ---------------------------------------------------------------------------
TEST(LoopPrecomputeTest, DipDepthIsYarnDiameter) {
    auto data = build_loop_data({"CCC", "KKK"});

    for (const auto& [seg_id, geom] : data.loops) {
        Vec3 curr_pos = data.frames[seg_id].position;
        Vec3 wale = data.frames[seg_id].wale_axis;

        // Dip depth along wale axis (absolute value, sign-independent)
        float entry_depth = std::abs((geom.entry_through - curr_pos).dot(wale));
        EXPECT_NEAR(entry_depth, data.yarn_compressed_diameter, 0.01f)
            << "Segment " << seg_id << ": entry dip depth should equal yarn_compressed_diameter";
    }
}

// ---------------------------------------------------------------------------
// Test 8: Crossover slot count matches child topology
// ---------------------------------------------------------------------------
TEST(LoopPrecomputeTest, CrossoverSlotCount) {
    auto data = build_loop_data({"CCC", "KKK"});
    const auto& segments = data.yarn_path.segments();

    for (const auto& [seg_id, geom] : data.loops) {
        auto child_it = data.children_map.find(seg_id);
        if (child_it == data.children_map.end() || child_it->second.empty()) {
            EXPECT_TRUE(geom.crossover_slots.empty())
                << "Segment " << seg_id << ": no children, no slots expected";
            continue;
        }

        // Count expected slots: 2 per loop-forming child, 1 per non-loop child
        int expected_slots = 0;
        for (SegmentId child_id : child_it->second) {
            expected_slots += segments[child_id].forms_loop ? 2 : 1;
        }

        EXPECT_EQ(static_cast<int>(geom.crossover_slots.size()), expected_slots)
            << "Segment " << seg_id << ": slot count mismatch";
    }
}

// ---------------------------------------------------------------------------
// Test 9: Crossover slots are at apex height
// ---------------------------------------------------------------------------
TEST(LoopPrecomputeTest, CrossoverSlotsAtApexHeight) {
    auto data = build_loop_data({"CCC", "KKK"});

    for (const auto& [seg_id, geom] : data.loops) {
        if (geom.crossover_slots.empty()) continue;

        // Slots should be at the same wale-axis height as the apex.
        // Offsets between slots are along stitch_axis (perpendicular to wale),
        // so their wale projection should match the apex's.
        Vec3 curr_pos = data.frames[seg_id].position;
        Vec3 wale = data.frames[seg_id].wale_axis;
        float apex_wale = (geom.apex - curr_pos).dot(wale);

        for (size_t s = 0; s < geom.crossover_slots.size(); ++s) {
            float slot_wale = (geom.crossover_slots[s].position - curr_pos).dot(wale);
            EXPECT_NEAR(slot_wale, apex_wale, 0.01f)
                << "Segment " << seg_id << " slot " << s
                << ": slot wale-axis height should match apex wale-axis height";
        }
    }
}

// ---------------------------------------------------------------------------
// Test 10: Crossover slots are spread along course direction
// ---------------------------------------------------------------------------
TEST(LoopPrecomputeTest, CrossoverSlotsSpreadAlongCourse) {
    auto data = build_loop_data({"CCC", "KKK"});

    for (const auto& [seg_id, geom] : data.loops) {
        if (geom.crossover_slots.size() < 2) continue;

        // Check that consecutive slots are spaced by yarn_compressed_diameter
        // along the stitch axis (course direction)
        Vec3 stitch = data.frames[seg_id].stitch_axis;

        for (size_t s = 1; s < geom.crossover_slots.size(); ++s) {
            Vec3 diff = geom.crossover_slots[s].position - geom.crossover_slots[s-1].position;
            float spacing = diff.dot(stitch);
            EXPECT_NEAR(spacing, data.yarn_compressed_diameter, 0.01f)
                << "Segment " << seg_id << " slots " << (s-1) << "-" << s
                << ": spacing should be yarn_compressed_diameter";
        }
    }
}

// ---------------------------------------------------------------------------
// Test 11: Crossover slot normals match fabric normal
// ---------------------------------------------------------------------------
TEST(LoopPrecomputeTest, CrossoverSlotNormalsMatchFabric) {
    auto data = build_loop_data({"CCC", "KKK"});

    for (const auto& [seg_id, geom] : data.loops) {
        Vec3 fnormal = data.frames[seg_id].fabric_normal;

        for (size_t s = 0; s < geom.crossover_slots.size(); ++s) {
            Vec3 slot_normal = geom.crossover_slots[s].crossing_normal;
            float dot = slot_normal.dot(fnormal);
            EXPECT_NEAR(dot, 1.0f, 0.01f)
                << "Segment " << seg_id << " slot " << s
                << ": crossing_normal should match fabric_normal";
        }
    }
}
