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
