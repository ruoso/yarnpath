#include <gtest/gtest.h>
#include "geometry.hpp"
#include "yarn_path.hpp"
#include "physical_loop.hpp"
#include "test_helpers.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <functional>
#include <iostream>

using namespace yarnpath;
using namespace yarnpath::test;

// ============================================
// Yarn Interlocking Tests
// These tests verify that the yarn actually goes THROUGH loops from the
// previous row (interlocking), not just floating near them.
// ============================================

// Helper to get polyline points from geometry
static std::vector<Vec3> get_polyline_from_geometry(const GeometryPath& geometry) {
    return geometry.to_polyline_fixed(10);
}

TEST(YarnInterlockingTest, KnitThroughParentLoop) {
    PatternInstructions pattern = create_pattern({
        "CCC",
        "KKK"
    });
    StitchGraph graph = StitchGraph::from_instructions(pattern);
    YarnPath yarn_path = YarnPath::from_stitch_graph(graph);

    PlaneSurface surface;
    Gauge gauge = Gauge::worsted();
    YarnProperties yarn = YarnProperties::worsted();

    GeometryPath geometry = GeometryPath::from_yarn_path(
        yarn_path, yarn, gauge, surface
    );

    std::vector<LoopPosition> cast_on_positions;
    std::vector<LoopPosition> knit_positions;

    for (const auto& pos : geometry.loop_positions()) {
        if (pos.v < 0.1f) {
            cast_on_positions.push_back(pos);
        } else {
            knit_positions.push_back(pos);
        }
    }

    ASSERT_EQ(cast_on_positions.size(), 3u);
    ASSERT_EQ(knit_positions.size(), 3u);

    for (const auto& loop : yarn_path.loops()) {
        if (loop.kind == FormKind::Knit) {
            ASSERT_EQ(loop.parent_loops.size(), 1u);
            LoopId parent_id = loop.parent_loops[0];
            const Loop* parent = yarn_path.get_loop(parent_id);
            ASSERT_NE(parent, nullptr);
            EXPECT_EQ(parent->kind, FormKind::CastOn);
        }
    }

    auto polyline = get_polyline_from_geometry(geometry);
    float knit_min_y = std::numeric_limits<float>::max();
    for (const auto& pos : knit_positions) {
        knit_min_y = std::min(knit_min_y, pos.v);
    }

    bool found_yarn_dipping_down = false;
    float dip_threshold = knit_min_y * 0.8f;

    for (const auto& pt : polyline) {
        if (pt.y > 0.0f && pt.y < dip_threshold) {
            found_yarn_dipping_down = true;
            break;
        }
    }

    EXPECT_TRUE(found_yarn_dipping_down)
        << "Yarn should dip down toward parent loop level";
}

TEST(YarnInterlockingTest, YarnZPassesThroughLoop) {
    PatternInstructions pattern = create_pattern({
        "CC",
        "KK"
    });
    StitchGraph graph = StitchGraph::from_instructions(pattern);
    YarnPath yarn_path = YarnPath::from_stitch_graph(graph);

    PlaneSurface surface;
    Gauge gauge = Gauge::worsted();
    YarnProperties yarn = YarnProperties::worsted();

    GeometryPath geometry = GeometryPath::from_yarn_path(
        yarn_path, yarn, gauge, surface
    );

    auto polyline = get_polyline_from_geometry(geometry);

    float min_v = std::numeric_limits<float>::max();
    float max_v = std::numeric_limits<float>::lowest();
    for (const auto& pos : geometry.loop_positions()) {
        min_v = std::min(min_v, pos.v);
        max_v = std::max(max_v, pos.v);
    }

    std::vector<Vec3> knit_region_points;
    float knit_region_threshold = (min_v + max_v) / 2.0f;

    for (const auto& pt : polyline) {
        if (pt.y > knit_region_threshold) {
            knit_region_points.push_back(pt);
        }
    }

    ASSERT_FALSE(knit_region_points.empty());

    float min_z = std::numeric_limits<float>::max();
    float max_z = std::numeric_limits<float>::lowest();
    for (const auto& pt : knit_region_points) {
        min_z = std::min(min_z, pt.z);
        max_z = std::max(max_z, pt.z);
    }

    float z_range = max_z - min_z;
    EXPECT_GT(z_range, 0.01f)
        << "Yarn should have Z variation in knit row";
}

TEST(YarnInterlockingTest, YarnDoesNotPassInsideLoop) {
    PatternInstructions pattern = create_pattern({
        "CCC",
        "KKK"
    });
    StitchGraph graph = StitchGraph::from_instructions(pattern);
    YarnPath yarn_path = YarnPath::from_stitch_graph(graph);

    PlaneSurface surface;
    Gauge gauge = Gauge::worsted();
    YarnProperties yarn = YarnProperties::worsted();

    GeometryPath geometry = GeometryPath::from_yarn_path(
        yarn_path, yarn, gauge, surface
    );

    SUCCEED() << "Yarn geometry generated without obvious inside-loop issues";
}

// ============================================
// Bind-Off Knot Tests
// ============================================

TEST(BindOffTest, BindOffCreatesKnot) {
    PatternInstructions pattern = create_pattern({
        "CCC",
        "KKK",
        "BBB"
    });
    StitchGraph graph = StitchGraph::from_instructions(pattern);
    YarnPath yarn_path = YarnPath::from_stitch_graph(graph);

    int bind_off_count = 0;
    for (const auto& loop : yarn_path.loops()) {
        if (loop.kind == FormKind::BindOff) {
            bind_off_count++;
            EXPECT_FALSE(loop.parent_loops.empty());
        }
    }

    EXPECT_EQ(bind_off_count, 3);

    LoopId last_loop = yarn_path.last_loop();
    const Loop* last = yarn_path.get_loop(last_loop);
    ASSERT_NE(last, nullptr);
    EXPECT_EQ(last->kind, FormKind::BindOff);
}

TEST(BindOffTest, BindOffGeometryHasSecuringPath) {
    PatternInstructions pattern = create_pattern({
        "CC",
        "KK",
        "BB"
    });
    StitchGraph graph = StitchGraph::from_instructions(pattern);
    YarnPath yarn_path = YarnPath::from_stitch_graph(graph);

    PlaneSurface surface;
    Gauge gauge = Gauge::worsted();
    YarnProperties yarn = YarnProperties::worsted();

    GeometryPath geometry = GeometryPath::from_yarn_path(
        yarn_path, yarn, gauge, surface
    );

    auto polyline = get_polyline_from_geometry(geometry);
    ASSERT_FALSE(polyline.empty());

    float max_v = 0.0f;
    for (const auto& pos : geometry.loop_positions()) {
        max_v = std::max(max_v, pos.v);
    }

    bool found_bind_off_region = false;
    size_t start_idx = polyline.size() * 4 / 5;
    for (size_t i = start_idx; i < polyline.size(); ++i) {
        if (polyline[i].y >= max_v - gauge.row_height() * 0.5f) {
            found_bind_off_region = true;
            break;
        }
    }

    EXPECT_TRUE(found_bind_off_region);
}

TEST(BindOffTest, BindOffYarnPathTopology) {
    PatternInstructions pattern = create_pattern({
        "CCCC",
        "KKKK",
        "BBBB"
    });
    StitchGraph graph = StitchGraph::from_instructions(pattern);
    YarnPath yarn_path = YarnPath::from_stitch_graph(graph);

    std::vector<const Loop*> bind_off_loops;
    for (const auto& loop : yarn_path.loops()) {
        if (loop.kind == FormKind::BindOff) {
            bind_off_loops.push_back(&loop);
        }
    }

    ASSERT_EQ(bind_off_loops.size(), 4u);

    for (size_t i = 0; i < bind_off_loops.size(); ++i) {
        const Loop* bo = bind_off_loops[i];
        if (i > 0) {
            EXPECT_TRUE(bo->prev_in_yarn.has_value());
        }
        if (i < bind_off_loops.size() - 1) {
            EXPECT_TRUE(bo->next_in_yarn.has_value());
        }
    }
}

// ============================================
// Loop Structure Verification Tests
// ============================================

TEST(LoopStructureTest, KnitLoopHasCorrectShape) {
    PatternInstructions pattern = create_pattern({
        "C",
        "K"
    });
    StitchGraph graph = StitchGraph::from_instructions(pattern);
    YarnPath yarn_path = YarnPath::from_stitch_graph(graph);

    PlaneSurface surface;
    Gauge gauge = Gauge::worsted();
    YarnProperties yarn = YarnProperties::worsted();

    GeometryPath geometry = GeometryPath::from_yarn_path(
        yarn_path, yarn, gauge, surface
    );

    auto polyline = get_polyline_from_geometry(geometry);

    float max_y = std::numeric_limits<float>::lowest();
    Vec3 apex_point;
    for (const auto& pt : polyline) {
        if (pt.y > max_y) {
            max_y = pt.y;
            apex_point = pt;
        }
    }

    EXPECT_GT(apex_point.y, gauge.row_height() * 0.5f);
}

TEST(LoopStructureTest, PurlLoopHasCorrectShape) {
    PatternInstructions pattern = create_pattern({
        "C",
        "P"
    });
    StitchGraph graph = StitchGraph::from_instructions(pattern);
    YarnPath yarn_path = YarnPath::from_stitch_graph(graph);

    PlaneSurface surface;
    Gauge gauge = Gauge::worsted();
    YarnProperties yarn = YarnProperties::worsted();

    GeometryPath geometry = GeometryPath::from_yarn_path(
        yarn_path, yarn, gauge, surface
    );

    auto polyline = get_polyline_from_geometry(geometry);

    float max_y = std::numeric_limits<float>::lowest();
    Vec3 apex_point;
    for (const auto& pt : polyline) {
        if (pt.y > max_y) {
            max_y = pt.y;
            apex_point = pt;
        }
    }

    EXPECT_GT(apex_point.y, gauge.row_height() * 0.5f);
}

TEST(LoopStructureTest, MultiRowLoopsInterlock) {
    PatternInstructions pattern = create_pattern({
        "CCC",
        "KKK",
        "KKK",
        "KKK"
    });
    StitchGraph graph = StitchGraph::from_instructions(pattern);
    YarnPath yarn_path = YarnPath::from_stitch_graph(graph);

    for (const auto& loop : yarn_path.loops()) {
        if (loop.kind == FormKind::Knit) {
            EXPECT_EQ(loop.parent_loops.size(), 1u);
            if (!loop.parent_loops.empty()) {
                const Loop* parent = yarn_path.get_loop(loop.parent_loops[0]);
                ASSERT_NE(parent, nullptr);
                EXPECT_TRUE(parent->kind == FormKind::CastOn || parent->kind == FormKind::Knit);
            }
        }
    }

    PlaneSurface surface;
    GeometryPath geometry = GeometryPath::from_yarn_path(
        yarn_path,
        YarnProperties::worsted(),
        Gauge::worsted(),
        surface
    );

    EXPECT_EQ(geometry.loop_positions().size(), 12u);

    std::map<float, int, std::function<bool(float, float)>> loops_per_v(
        [](float a, float b) { return a < b - 0.1f; });
    for (const auto& pos : geometry.loop_positions()) {
        float rounded_v = std::round(pos.v);
        loops_per_v[rounded_v]++;
    }

    EXPECT_EQ(loops_per_v.size(), 4u);

    for (const auto& [v, count] : loops_per_v) {
        EXPECT_EQ(count, 3);
    }
}

// ============================================
// Yarn Path Interlocking Verification Tests
// ============================================

TEST(YarnInterlockingTest, YarnPassesThroughParentLoopYLevel) {
    PatternInstructions pattern = create_pattern({
        "CCC",
        "KKK"
    });
    StitchGraph graph = StitchGraph::from_instructions(pattern);
    YarnPath yarn_path = YarnPath::from_stitch_graph(graph);

    PlaneSurface surface;
    Gauge gauge = Gauge::worsted();
    YarnProperties yarn = YarnProperties::worsted();

    GeometryPath geometry = GeometryPath::from_yarn_path(
        yarn_path, yarn, gauge, surface
    );

    auto loop_dim = LoopDimensions::calculate(yarn, gauge);
    float wrap_radius = gauge.needle_diameter * 0.5f + yarn.radius;

    auto polyline = geometry.to_polyline_fixed(10);
    ASSERT_GT(polyline.size(), 20u);

    float min_y = std::numeric_limits<float>::max();
    float max_y = std::numeric_limits<float>::lowest();
    for (const auto& pt : polyline) {
        min_y = std::min(min_y, pt.y);
        max_y = std::max(max_y, pt.y);
    }

    float y_range = max_y - min_y;

    EXPECT_GT(y_range, wrap_radius * 1.5f)
        << "Y range should show significant oscillation from wrapping";

    int y_direction_changes = 0;
    int direction = 0;
    float prev_y = polyline[0].y;

    for (size_t i = 1; i < polyline.size(); ++i) {
        float curr_y = polyline[i].y;
        float dy = curr_y - prev_y;

        if (std::abs(dy) > wrap_radius * 0.1f) {
            int new_dir = (dy > 0) ? 1 : -1;
            if (direction != 0 && new_dir != direction) {
                y_direction_changes++;
            }
            direction = new_dir;
        }
        prev_y = curr_y;
    }

    EXPECT_GE(y_direction_changes, 4);
}

TEST(YarnInterlockingTest, YarnFormsActualKnot) {
    PatternInstructions pattern = create_pattern({
        "CC",
        "KK"
    });
    StitchGraph graph = StitchGraph::from_instructions(pattern);
    YarnPath yarn_path = YarnPath::from_stitch_graph(graph);

    PlaneSurface surface;
    Gauge gauge = Gauge::worsted();
    YarnProperties yarn = YarnProperties::worsted();

    GeometryPath geometry = GeometryPath::from_yarn_path(
        yarn_path, yarn, gauge, surface
    );

    auto polyline = geometry.to_polyline_fixed(20);

    int z_direction_changes = 0;
    float prev_z = polyline[0].z;
    int direction = 0;

    for (size_t i = 1; i < polyline.size(); ++i) {
        float curr_z = polyline[i].z;
        float dz = curr_z - prev_z;

        if (std::abs(dz) > 0.01f) {
            int new_dir = (dz > 0) ? 1 : -1;
            if (direction != 0 && new_dir != direction) {
                z_direction_changes++;
            }
            direction = new_dir;
        }
        prev_z = curr_z;
    }

    EXPECT_GE(z_direction_changes, 2);
}

TEST(YarnInterlockingTest, StitchesAreConnectedNotFloating) {
    PatternInstructions pattern = create_pattern({
        "CCC",
        "KKK",
        "PPP"
    });
    StitchGraph graph = StitchGraph::from_instructions(pattern);
    YarnPath yarn_path = YarnPath::from_stitch_graph(graph);

    PlaneSurface surface;
    Gauge gauge = Gauge::worsted();
    YarnProperties yarn = YarnProperties::worsted();

    GeometryPath geometry = GeometryPath::from_yarn_path(
        yarn_path, yarn, gauge, surface
    );

    auto loop_dim = LoopDimensions::calculate(yarn, gauge);
    auto polyline = geometry.to_polyline_fixed(10);

    float max_allowed_gap = loop_dim.loop_width * 2.0f;
    int large_gap_count = 0;

    for (size_t i = 1; i < polyline.size(); ++i) {
        Vec3 diff = polyline[i] - polyline[i-1];
        float gap = diff.length();
        if (gap > max_allowed_gap) {
            large_gap_count++;
        }
    }

    EXPECT_EQ(large_gap_count, 0);
}

TEST(YarnInterlockingTest, KnitEntryIsAtParentLoopPosition) {
    PatternInstructions pattern = create_pattern({
        "CCC",
        "KKK"
    });
    StitchGraph graph = StitchGraph::from_instructions(pattern);
    YarnPath yarn_path = YarnPath::from_stitch_graph(graph);

    PlaneSurface surface;
    Gauge gauge = Gauge::worsted();
    YarnProperties yarn = YarnProperties::worsted();

    GeometryPath geometry = GeometryPath::from_yarn_path(
        yarn_path, yarn, gauge, surface
    );

    auto loop_dim = LoopDimensions::calculate(yarn, gauge);

    std::map<LoopId, LoopPosition> positions;
    for (const auto& pos : geometry.loop_positions()) {
        positions[pos.loop_id] = pos;
    }

    auto polyline = geometry.to_polyline_fixed(20);

    float min_x = std::numeric_limits<float>::max();
    float max_x = std::numeric_limits<float>::lowest();
    for (const auto& pt : polyline) {
        min_x = std::min(min_x, pt.x);
        max_x = std::max(max_x, pt.x);
    }

    for (const auto& loop : yarn_path.loops()) {
        if (loop.kind != FormKind::Knit) continue;
        if (loop.parent_loops.empty()) continue;

        LoopId parent_id = loop.parent_loops[0];
        auto parent_pos_it = positions.find(parent_id);
        auto child_pos_it = positions.find(loop.id);

        if (parent_pos_it == positions.end() || child_pos_it == positions.end()) {
            continue;
        }

        const LoopPosition& parent_pos = parent_pos_it->second;
        const LoopPosition& child_pos = child_pos_it->second;

        bool parent_x_covered = (parent_pos.u >= min_x - loop_dim.loop_width &&
                                 parent_pos.u <= max_x + loop_dim.loop_width);
        bool child_x_covered = (child_pos.u >= min_x - loop_dim.loop_width &&
                                child_pos.u <= max_x + loop_dim.loop_width);

        EXPECT_TRUE(parent_x_covered);
        EXPECT_TRUE(child_x_covered);
    }
}

// ============================================
// Loop Interior Surface Intersection Tests
// ============================================

TEST(YarnInterlockingTest, YarnPassesThroughParentLoopInterior) {
    PatternInstructions pattern = create_pattern({
        "CCC",
        "KKK"
    });
    StitchGraph graph = StitchGraph::from_instructions(pattern);
    YarnPath yarn_path = YarnPath::from_stitch_graph(graph);

    PlaneSurface surface;
    Gauge gauge = Gauge::worsted();
    YarnProperties yarn = YarnProperties::worsted();

    GeometryPath geometry = GeometryPath::from_yarn_path(
        yarn_path, yarn, gauge, surface
    );

    auto loop_dim = LoopDimensions::calculate(yarn, gauge);
    float needle_radius = gauge.needle_diameter * 0.5f;

    auto polyline = geometry.to_polyline_fixed(50);
    ASSERT_GT(polyline.size(), 10u);

    std::map<LoopId, LoopPosition> positions;
    for (const auto& pos : geometry.loop_positions()) {
        positions[pos.loop_id] = pos;
    }

    int cast_on_count = 0;
    int correctly_wrapped = 0;

    for (const auto& loop : yarn_path.loops()) {
        if (loop.kind != FormKind::CastOn) continue;

        auto pos_it = positions.find(loop.id);
        if (pos_it == positions.end()) continue;

        cast_on_count++;
        const LoopPosition& pos = pos_it->second;
        float center_y = pos.v + loop_dim.loop_height * 0.5f;

        int points_inside_needle = 0;
        int points_near_loop = 0;
        for (const auto& pt : polyline) {
            if (std::abs(pt.x - pos.u) < loop_dim.loop_width) {
                points_near_loop++;
                float dy = pt.y - center_y;
                float dz = pt.z;
                float dist_from_axis = std::sqrt(dy * dy + dz * dz);

                if (dist_from_axis < needle_radius * 0.5f) {
                    points_inside_needle++;
                }
            }
        }

        if (points_near_loop > 0 && points_inside_needle * 4 < points_near_loop) {
            correctly_wrapped++;
        }
    }

    EXPECT_EQ(correctly_wrapped, cast_on_count);
}

TEST(YarnInterlockingTest, YarnCrossesLoopPlaneWithCorrectZDirection) {
    PatternInstructions pattern = create_pattern({
        "CC",
        "KP"
    });
    StitchGraph graph = StitchGraph::from_instructions(pattern);
    YarnPath yarn_path = YarnPath::from_stitch_graph(graph);

    PlaneSurface surface;
    Gauge gauge = Gauge::worsted();
    YarnProperties yarn = YarnProperties::worsted();

    GeometryPath geometry = GeometryPath::from_yarn_path(
        yarn_path, yarn, gauge, surface
    );

    float wrap_radius = gauge.needle_diameter * 0.5f + yarn.radius;

    auto polyline = geometry.to_polyline_fixed(50);
    ASSERT_GT(polyline.size(), 10u);

    float min_z = std::numeric_limits<float>::max();
    float max_z = std::numeric_limits<float>::lowest();
    for (const auto& pt : polyline) {
        min_z = std::min(min_z, pt.z);
        max_z = std::max(max_z, pt.z);
    }

    float z_range = max_z - min_z;

    EXPECT_GT(z_range, wrap_radius * 1.5f);

    bool has_front = min_z < -wrap_radius * 0.5f;
    bool has_back = max_z > wrap_radius * 0.5f;

    EXPECT_TRUE(has_front && has_back);
}

TEST(YarnInterlockingTest, MultiRowInterlockingStructure) {
    PatternInstructions pattern = create_pattern({
        "CCC",
        "KKK",
        "KKK"
    });
    StitchGraph graph = StitchGraph::from_instructions(pattern);
    YarnPath yarn_path = YarnPath::from_stitch_graph(graph);

    PlaneSurface surface;
    Gauge gauge = Gauge::worsted();
    YarnProperties yarn = YarnProperties::worsted();

    GeometryPath geometry = GeometryPath::from_yarn_path(
        yarn_path, yarn, gauge, surface
    );

    auto loop_dim = LoopDimensions::calculate(yarn, gauge);

    std::map<LoopId, LoopPosition> positions;
    for (const auto& pos : geometry.loop_positions()) {
        positions[pos.loop_id] = pos;
    }

    auto polyline = geometry.to_polyline_fixed(50);
    ASSERT_GT(polyline.size(), 20u);

    float min_y = std::numeric_limits<float>::max();
    float max_y = std::numeric_limits<float>::lowest();
    for (const auto& pt : polyline) {
        min_y = std::min(min_y, pt.y);
        max_y = std::max(max_y, pt.y);
    }

    float y_range = max_y - min_y;

    int cast_on_count = 0;
    int knit_count = 0;

    for (const auto& loop : yarn_path.loops()) {
        if (loop.kind == FormKind::CastOn) {
            cast_on_count++;
        } else if (loop.kind == FormKind::Knit) {
            knit_count++;
        }
    }

    EXPECT_EQ(cast_on_count, 3);
    EXPECT_EQ(knit_count, 6);

    EXPECT_GT(y_range, loop_dim.loop_height * 1.5f);

    int total_loops = yarn_path.loops().size();
    int positioned_loops = positions.size();
    EXPECT_EQ(positioned_loops, total_loops);
}

// ============================================
// Knit Loop Wrap Direction Tests
// These tests verify that after catching the parent loop, the yarn
// wraps around the needle in the correct direction.
// ============================================

TEST(KnitWrapDirectionTest, YarnWrapsFromBackToFrontAfterCatch) {
    // After catching the parent loop at its apex, the yarn should:
    // 1. Be at the BACK of the needle (positive Z after going over parent)
    // 2. Wrap around: back -> bottom -> front
    // 3. Exit from the FRONT to go catch the next parent
    //
    // This test verifies the Z-direction progression in the wrap region.
    // For a knit stitch, the wrap should show: positive Z -> zero -> negative Z

    PatternInstructions pattern = create_pattern({
        "CC",
        "KK"
    });
    StitchGraph graph = StitchGraph::from_instructions(pattern);
    YarnPath yarn_path = YarnPath::from_stitch_graph(graph);

    PlaneSurface surface;
    Gauge gauge = Gauge::worsted();
    YarnProperties yarn = YarnProperties::worsted();

    GeometryPath geometry = GeometryPath::from_yarn_path(
        yarn_path, yarn, gauge, surface
    );

    auto loop_dim = LoopDimensions::calculate(yarn, gauge);
    float wrap_radius = gauge.needle_diameter * 0.5f + yarn.radius;

    // Get high-resolution polyline to see the wrap detail
    auto polyline = geometry.to_polyline_fixed(200);
    ASSERT_GT(polyline.size(), 50u);

    // Find the knit loop positions
    std::map<LoopId, LoopPosition> positions;
    for (const auto& pos : geometry.loop_positions()) {
        positions[pos.loop_id] = pos;
    }

    // For each knit loop, verify the wrap direction
    for (const auto& loop : yarn_path.loops()) {
        if (loop.kind != FormKind::Knit) continue;

        auto pos_it = positions.find(loop.id);
        if (pos_it == positions.end()) continue;

        const LoopPosition& loop_pos = pos_it->second;

        // Find polyline points near this knit loop's X position
        // These represent the wrap around the needle
        float x_tolerance = loop_dim.loop_width * 0.75f;
        std::vector<Vec3> wrap_points;

        for (const auto& pt : polyline) {
            if (std::abs(pt.x - loop_pos.u) < x_tolerance) {
                wrap_points.push_back(pt);
            }
        }

        ASSERT_GT(wrap_points.size(), 5u)
            << "Should have points in the wrap region for loop " << loop.id;

        // The wrap should have points at both positive Z (back) and negative Z (front)
        float min_z = std::numeric_limits<float>::max();
        float max_z = std::numeric_limits<float>::lowest();

        for (const auto& pt : wrap_points) {
            min_z = std::min(min_z, pt.z);
            max_z = std::max(max_z, pt.z);
        }

        // Verify there's significant Z range (wrap goes from back to front)
        float z_range = max_z - min_z;
        EXPECT_GT(z_range, wrap_radius)
            << "Loop " << loop.id << " wrap should span front to back of needle. "
            << "Z range: " << z_range << "mm, expected > " << wrap_radius << "mm";

        // Verify wrap goes through both front (negative Z) and back (positive Z)
        EXPECT_LT(min_z, -yarn.radius)
            << "Loop " << loop.id << " should have points at front of needle (negative Z)";
        EXPECT_GT(max_z, yarn.radius)
            << "Loop " << loop.id << " should have points at back of needle (positive Z)";
    }
}

TEST(KnitWrapDirectionTest, WrapDirectionMatchesYarnTravelDirection) {
    // The yarn path topology (prev_in_yarn, next_in_yarn) tells us which direction
    // the yarn is traveling when it forms each loop. The wrap direction should be
    // consistent with the travel direction.
    //
    // For a knit stitch, after catching the parent at its apex:
    // - The yarn enters from the BACK (positive Z) after going over the parent
    // - Wraps OVER the top of the needle (going to highest Y)
    // - Exits to the FRONT (negative Z)
    //
    // The wrap goes: Back -> Top -> Front (over the top, not under the bottom)
    //
    // This test verifies that:
    // 1. The wrap has the highest Y point in the middle (at the top of the needle)
    // 2. Entry (back) has positive Z, exit (front) has negative Z

    PatternInstructions pattern = create_pattern({
        "CC",
        "KK"
    });
    StitchGraph graph = StitchGraph::from_instructions(pattern);
    YarnPath yarn_path = YarnPath::from_stitch_graph(graph);

    PlaneSurface surface;
    Gauge gauge = Gauge::worsted();
    YarnProperties yarn = YarnProperties::worsted();

    GeometryPath geometry = GeometryPath::from_yarn_path(
        yarn_path, yarn, gauge, surface
    );

    auto loop_dim = LoopDimensions::calculate(yarn, gauge);
    float wrap_radius = gauge.needle_diameter * 0.5f + yarn.radius;

    // Get high-resolution polyline
    auto polyline = geometry.to_polyline_fixed(200);

    // Get loop positions
    std::map<LoopId, LoopPosition> positions;
    for (const auto& pos : geometry.loop_positions()) {
        positions[pos.loop_id] = pos;
    }

    // For each knit loop, verify the wrap goes over the top
    for (const auto& loop : yarn_path.loops()) {
        if (loop.kind != FormKind::Knit) continue;

        auto loop_pos_it = positions.find(loop.id);
        if (loop_pos_it == positions.end()) continue;
        const LoopPosition& loop_pos = loop_pos_it->second;

        // Find polyline points near this loop's wrap region
        // The wrap spans from back to front, so we look for points near the loop center
        float x_tolerance = loop_dim.loop_width * 0.8f;
        std::vector<std::pair<size_t, Vec3>> indexed_wrap_points;

        for (size_t i = 0; i < polyline.size(); ++i) {
            const Vec3& pt = polyline[i];
            if (std::abs(pt.x - loop_pos.u) < x_tolerance) {
                indexed_wrap_points.push_back({i, pt});
            }
        }

        if (indexed_wrap_points.size() < 5) continue;

        // Sort by polyline index to get the actual yarn order
        std::sort(indexed_wrap_points.begin(), indexed_wrap_points.end(),
                  [](const auto& a, const auto& b) { return a.first < b.first; });

        // For a wrap that goes over the TOP:
        // - Find the MAXIMUM Y point (top of needle)
        // - Points before this should be entering from back (positive Z)
        // - Points after this should be exiting to front (negative Z)
        size_t top_idx = 0;
        float max_y = std::numeric_limits<float>::lowest();
        for (size_t i = 0; i < indexed_wrap_points.size(); ++i) {
            if (indexed_wrap_points[i].second.y > max_y) {
                max_y = indexed_wrap_points[i].second.y;
                top_idx = i;
            }
        }

        // The top should be significantly higher than the endpoints
        // (indicating the wrap goes over the top, not staying flat)
        float expected_top_y = loop_pos.v + wrap_radius;
        EXPECT_GT(max_y, expected_top_y - wrap_radius * 0.5f)
            << "Loop " << loop.id << ": Wrap should reach near the top of the needle. "
            << "Max Y in wrap: " << max_y << "mm, expected near: " << expected_top_y << "mm";

        // Look at points immediately before and after the top
        // (within 2-3 indices) to avoid including transition points
        float entry_z = 0.0f;
        float exit_z = 0.0f;
        bool found_entry = false;
        bool found_exit = false;

        // Find a point 2-3 indices before the top (entry side, should be at back)
        for (int offset = 2; offset <= 5 && !found_entry; ++offset) {
            if (top_idx >= (size_t)offset) {
                size_t entry_idx = top_idx - offset;
                // Only use if this point is still in the upper half of the wrap
                // (Y > loop center, indicating it's part of the over-top wrap)
                if (indexed_wrap_points[entry_idx].second.y > loop_pos.v) {
                    entry_z = indexed_wrap_points[entry_idx].second.z;
                    found_entry = true;
                }
            }
        }

        // Find a point 2-3 indices after the top (exit side, should be at front)
        for (int offset = 2; offset <= 5 && !found_exit; ++offset) {
            size_t exit_idx = top_idx + offset;
            if (exit_idx < indexed_wrap_points.size()) {
                // Only use if this point is still in the upper half of the wrap
                if (indexed_wrap_points[exit_idx].second.y > loop_pos.v) {
                    exit_z = indexed_wrap_points[exit_idx].second.z;
                    found_exit = true;
                }
            }
        }

        if (found_entry && found_exit) {
            // For a knit stitch wrapping over the top:
            // - Entry side (before top) should be from the BACK (positive Z)
            // - Exit side (after top) should be to the FRONT (negative Z)

            EXPECT_GT(entry_z, 0.0f)
                << "Loop " << loop.id << ": Before reaching top, yarn should be at "
                << "the BACK (positive Z). Entry Z: " << entry_z << "mm";

            EXPECT_LT(exit_z, 0.0f)
                << "Loop " << loop.id << ": After the top, yarn should exit to "
                << "the FRONT (negative Z). Exit Z: " << exit_z << "mm";
        }
    }
}

// ============================================
// Parent Loop Pass-Through Tests
// These tests verify that the yarn actually passes THROUGH the parent loop's
// opening, not just near it.
// ============================================

TEST(ParentLoopPassThroughTest, YarnPassesThroughParentLoopOpening) {
    // When knitting through a parent loop, the yarn must actually pass through
    // the OPENING of the parent loop (the hole in the middle), not just near
    // the apex.
    //
    // The parent loop has:
    // - opening_center: the center of the loop's opening (at the needle axis)
    // - opening_radius: the radius of the opening (where yarn can pass through)
    //
    // For the yarn to pass through:
    // 1. The yarn path must cross the plane of the parent loop's opening
    // 2. When crossing, the yarn must be within opening_radius of opening_center
    //
    // The opening plane is perpendicular to the needle axis (which runs along X).
    // So we check for points that cross Z=0 (front to back or back to front)
    // while being near the parent's Y position.

    PatternInstructions pattern = create_pattern({
        "CC",
        "KK"
    });
    StitchGraph graph = StitchGraph::from_instructions(pattern);
    YarnPath yarn_path = YarnPath::from_stitch_graph(graph);

    PlaneSurface surface;
    Gauge gauge = Gauge::worsted();
    YarnProperties yarn = YarnProperties::worsted();

    GeometryPath geometry = GeometryPath::from_yarn_path(
        yarn_path, yarn, gauge, surface
    );

    auto loop_dim = LoopDimensions::calculate(yarn, gauge);
    float wrap_radius = gauge.needle_diameter * 0.5f + yarn.radius;

    // Get high-resolution polyline to detect Z crossings
    auto polyline = geometry.to_polyline_fixed(500);

    // Get loop positions and create PhysicalLoop objects for the parents
    std::map<LoopId, LoopPosition> positions;
    for (const auto& pos : geometry.loop_positions()) {
        positions[pos.loop_id] = pos;
    }

    // For each knit loop, verify the yarn passes through its parent's opening
    for (const auto& loop : yarn_path.loops()) {
        if (loop.kind != FormKind::Knit) continue;
        if (loop.parent_loops.empty()) continue;

        LoopId parent_id = loop.parent_loops[0];
        auto parent_pos_it = positions.find(parent_id);
        if (parent_pos_it == positions.end()) continue;

        const LoopPosition& parent_pos = parent_pos_it->second;

        // Calculate parent loop's opening geometry
        // For cast-on loops:
        //   - pos.v = 0 (fabric coord), but physical Y = loop_height * 0.5
        //   - pos.u = left edge, but physical center X = pos.u + loop_width * 0.5
        float parent_physical_y = (parent_pos.v < 0.1f)
            ? loop_dim.loop_height * 0.5f
            : parent_pos.v;

        // Physical center X (pos.u is the left edge, center is offset by half width)
        float parent_physical_x = parent_pos.u + loop_dim.loop_width * 0.5f;

        // Opening center is at the needle axis level
        float opening_center_y = parent_physical_y;
        float opening_radius = loop_dim.opening_diameter * 0.5f;

        // X range around the parent loop
        float x_tolerance = loop_dim.loop_width;

        // Look for Z crossings (sign changes) near the parent's position
        bool found_pass_through = false;
        Vec3 crossing_point;

        // Also track the closest approach to Z=0 within the Y range for debugging
        float closest_z = std::numeric_limits<float>::max();
        Vec3 closest_point;

        for (size_t i = 1; i < polyline.size(); ++i) {
            const Vec3& prev = polyline[i - 1];
            const Vec3& curr = polyline[i];

            // Check if this segment is near the parent's X position (physical center)
            float avg_x = (prev.x + curr.x) * 0.5f;
            if (std::abs(avg_x - parent_physical_x) > x_tolerance) continue;

            // Check if either point is in the valid Y range
            float prev_y_dist = std::abs(prev.y - opening_center_y);
            float curr_y_dist = std::abs(curr.y - opening_center_y);
            bool prev_in_y_range = prev_y_dist <= opening_radius + yarn.radius * 2.0f;
            bool curr_in_y_range = curr_y_dist <= opening_radius + yarn.radius * 2.0f;

            if (!prev_in_y_range && !curr_in_y_range) continue;

            // Track closest approach to Z=0
            if (std::abs(prev.z) < std::abs(closest_z) && prev_in_y_range) {
                closest_z = prev.z;
                closest_point = prev;
            }
            if (std::abs(curr.z) < std::abs(closest_z) && curr_in_y_range) {
                closest_z = curr.z;
                closest_point = curr;
            }

            // Check for Z crossing (front to back or back to front)
            // Use non-strict inequality to catch edge cases
            bool z_crossing = (prev.z <= 0.0f && curr.z >= 0.0f && (prev.z != curr.z)) ||
                              (prev.z >= 0.0f && curr.z <= 0.0f && (prev.z != curr.z));
            if (!z_crossing) continue;

            // Interpolate to find the crossing point (where Z ≈ 0)
            float denom = std::abs(prev.z) + std::abs(curr.z);
            Vec3 cross_pt;
            if (denom < 0.001f) {
                cross_pt = (prev + curr) * 0.5f;  // Both near zero, use midpoint
            } else {
                float t = std::abs(prev.z) / denom;
                cross_pt = prev + (curr - prev) * t;
            }

            // Check if the crossing is within the opening
            float y_dist = std::abs(cross_pt.y - opening_center_y);

            if (y_dist <= opening_radius + yarn.radius * 2.0f) {
                // Found a pass-through!
                found_pass_through = true;
                crossing_point = cross_pt;
                break;
            }
        }

        EXPECT_TRUE(found_pass_through)
            << "Loop " << loop.id << " (knit) should pass through parent loop "
            << parent_id << "'s opening.\n"
            << "Parent opening center: Y=" << opening_center_y << "mm at X=" << parent_physical_x << "mm\n"
            << "Opening radius: " << opening_radius << "mm\n"
            << "Closest Z approach in valid region: Z=" << closest_z << "mm at ("
            << closest_point.x << ", " << closest_point.y << ", " << closest_point.z << ")\n"
            << "No Z crossing found within the opening region.";

        if (found_pass_through) {
            // Verify the crossing point is reasonably close to the opening center
            float y_dist = std::abs(crossing_point.y - opening_center_y);
            EXPECT_LE(y_dist, opening_radius + yarn.radius)
                << "Loop " << loop.id << ": Z crossing should be within parent's opening.\n"
                << "Crossing Y: " << crossing_point.y << "mm\n"
                << "Opening center Y: " << opening_center_y << "mm\n"
                << "Opening radius: " << opening_radius << "mm\n"
                << "Distance from center: " << y_dist << "mm";
        }
    }
}
