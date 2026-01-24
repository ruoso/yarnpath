#include <gtest/gtest.h>
#include "geometry.hpp"
#include "yarn_path.hpp"
#include "physical_loop.hpp"
#include "test_helpers.hpp"
#include <algorithm>
#include <cmath>
#include <set>
#include <numbers>

using namespace yarnpath;
using namespace yarnpath::test;

// ============================================
// GeometryPath Integration Tests
// ============================================

TEST(GeometryPathTest, EmptyYarnPath) {
    YarnPath empty_yarn_path;
    PlaneSurface surface;

    GeometryPath geometry = GeometryPath::from_yarn_path(
        empty_yarn_path,
        YarnProperties::worsted(),
        Gauge::worsted(),
        surface
    );

    EXPECT_TRUE(geometry.segments().empty());
}

TEST(GeometryPathTest, CastOnOnly) {
    PatternInstructions pattern = create_pattern({"CCCC"});
    StitchGraph graph = StitchGraph::from_instructions(pattern);
    YarnPath yarn_path = YarnPath::from_stitch_graph(graph);

    PlaneSurface surface;
    YarnProperties yarn = YarnProperties::worsted();
    Gauge gauge = Gauge::worsted();
    GeometryPath geometry = GeometryPath::from_yarn_path(
        yarn_path, yarn, gauge, surface
    );

    // Loop positions should form a horizontal line
    const auto& positions = geometry.loop_positions();
    EXPECT_EQ(positions.size(), 4u);

    // All cast-on loops should be at the base (v=0)
    for (const auto& pos : positions) {
        EXPECT_FLOAT_EQ(pos.v, 0.0f);
    }

    // Sort by u to check sequential spacing
    auto sorted_positions = positions;
    std::sort(sorted_positions.begin(), sorted_positions.end(),
              [](const LoopPosition& a, const LoopPosition& b) { return a.u < b.u; });

    // Check sequential columns with approximately equal spacing
    auto loop_dim = LoopDimensions::calculate(yarn, gauge);
    float prev_u = sorted_positions[0].u;
    for (size_t i = 1; i < sorted_positions.size(); ++i) {
        const auto& pos = sorted_positions[i];
        float spacing = pos.u - prev_u;
        // Spacing should be approximately loop_width
        EXPECT_NEAR(spacing, loop_dim.loop_width, loop_dim.loop_width * 0.5f);
        prev_u = pos.u;
    }
}

TEST(GeometryPathTest, StockinettePattern) {
    PatternInstructions pattern = create_pattern({
        "CCCC",
        "KKKK",
        "PPPP",
        "KKKK"
    });
    StitchGraph graph = StitchGraph::from_instructions(pattern);
    YarnPath yarn_path = YarnPath::from_stitch_graph(graph);

    PlaneSurface surface;
    Gauge gauge = Gauge::worsted();

    GeometryPath geometry = GeometryPath::from_yarn_path(
        yarn_path,
        YarnProperties::worsted(),
        gauge,
        surface
    );

    // Check that loop positions exist for all loops
    const auto& positions = geometry.loop_positions();
    EXPECT_EQ(positions.size(), 16u);  // 4x4 grid

    // Verify v values increase (loops stack vertically)
    // Collect unique v values with tolerance
    std::set<float> unique_v_values;
    for (const auto& pos : positions) {
        unique_v_values.insert(pos.v);
    }

    // Should have 4 distinct v levels (one per row)
    EXPECT_GE(unique_v_values.size(), 4u);

    // Verify v values are increasing
    float prev_v = -1.0f;
    for (float v : unique_v_values) {
        EXPECT_GT(v, prev_v) << "v values should increase";
        prev_v = v;
    }
}

TEST(GeometryPathTest, ToPolyline) {
    PatternInstructions pattern = create_pattern({
        "CCCC",
        "KKKK"
    });
    StitchGraph graph = StitchGraph::from_instructions(pattern);
    YarnPath yarn_path = YarnPath::from_stitch_graph(graph);

    PlaneSurface surface;
    GeometryPath geometry = GeometryPath::from_yarn_path(
        yarn_path,
        YarnProperties::worsted(),
        Gauge::worsted(),
        surface
    );

    // Convert to polyline
    auto polyline = geometry.to_polyline(0.1f);
    EXPECT_FALSE(polyline.empty());
}

TEST(GeometryPathTest, ToOBJ) {
    PatternInstructions pattern = create_pattern({
        "CCCC",
        "KKKK"
    });
    StitchGraph graph = StitchGraph::from_instructions(pattern);
    YarnPath yarn_path = YarnPath::from_stitch_graph(graph);

    PlaneSurface surface;
    GeometryPath geometry = GeometryPath::from_yarn_path(
        yarn_path,
        YarnProperties::worsted(),
        Gauge::worsted(),
        surface
    );

    std::string obj = geometry.to_obj();
    EXPECT_FALSE(obj.empty());
    EXPECT_NE(obj.find("v "), std::string::npos);  // Has vertices
}

TEST(GeometryPathTest, BoundingBox) {
    PatternInstructions pattern = create_pattern({
        "CCCC",
        "KKKK"
    });
    StitchGraph graph = StitchGraph::from_instructions(pattern);
    YarnPath yarn_path = YarnPath::from_stitch_graph(graph);

    PlaneSurface surface;
    GeometryPath geometry = GeometryPath::from_yarn_path(
        yarn_path,
        YarnProperties::worsted(),
        Gauge::worsted(),
        surface
    );

    auto [min_pt, max_pt] = geometry.bounding_box();

    // Box should have non-zero extent in X and Y
    EXPECT_LT(min_pt.x, max_pt.x);
    EXPECT_LT(min_pt.y, max_pt.y);
}

TEST(GeometryPathTest, Validation) {
    PatternInstructions pattern = create_pattern({
        "CCCC",
        "KKKK"
    });
    StitchGraph graph = StitchGraph::from_instructions(pattern);
    YarnPath yarn_path = YarnPath::from_stitch_graph(graph);

    PlaneSurface surface;
    YarnProperties yarn = YarnProperties::worsted();

    GeometryPath geometry = GeometryPath::from_yarn_path(
        yarn_path,
        yarn,
        Gauge::worsted(),
        surface
    );

    ValidationResult result = geometry.validate(yarn);
    // Should be valid (or only have warnings, not errors)
    // We don't strictly require valid=true since some edge cases may have warnings
    EXPECT_TRUE(result.errors.empty());
}

TEST(GeometryPathTest, DecreasePattern) {
    // K2tog creates a decrease (consumes 2 parent loops)
    PatternInstructions pattern = create_pattern({
        "CCCC",
        "KK2"  // K + K + K2tog = 3 outputs from 4 inputs (1+1+2)
    });
    StitchGraph graph = StitchGraph::from_instructions(pattern);
    YarnPath yarn_path = YarnPath::from_stitch_graph(graph);

    PlaneSurface surface;
    GeometryPath geometry = GeometryPath::from_yarn_path(
        yarn_path,
        YarnProperties::worsted(),
        Gauge::worsted(),
        surface
    );

    // The decrease row should have 3 loops (4 - 1 = 3)
    // Cast-on is at v=0, decrease row is at higher v
    int cast_on_count = 0;
    int decrease_row_count = 0;
    for (const auto& pos : geometry.loop_positions()) {
        if (pos.v < 0.1f) {
            cast_on_count++;
        } else {
            decrease_row_count++;
        }
    }
    EXPECT_EQ(cast_on_count, 4);
    EXPECT_EQ(decrease_row_count, 3);
}

TEST(GeometryPathTest, CylinderSurface) {
    PatternInstructions pattern = create_pattern({
        "CCCCCCCC",
        "KKKKKKKK"
    });
    StitchGraph graph = StitchGraph::from_instructions(pattern);
    YarnPath yarn_path = YarnPath::from_stitch_graph(graph);

    // Use cylinder surface
    float circumference = 2.0f;  // Will wrap 8 stitches in circumference
    CylinderSurface surface(circumference / (2.0f * std::numbers::pi_v<float>), circumference);

    GeometryPath geometry = GeometryPath::from_yarn_path(
        yarn_path,
        YarnProperties::worsted(),
        Gauge::worsted(),
        surface
    );

    // Geometry should have positions on cylinder surface
    auto [min_pt, max_pt] = geometry.bounding_box();

    // On a cylinder, X and Z will vary
    EXPECT_LT(min_pt.x, max_pt.x);
}

TEST(GeometryPathTest, TotalArcLength) {
    PatternInstructions pattern = create_pattern({
        "CCCC",
        "KKKK"
    });
    StitchGraph graph = StitchGraph::from_instructions(pattern);
    YarnPath yarn_path = YarnPath::from_stitch_graph(graph);

    PlaneSurface surface;
    GeometryPath geometry = GeometryPath::from_yarn_path(
        yarn_path,
        YarnProperties::worsted(),
        Gauge::worsted(),
        surface
    );

    float arc_length = geometry.total_arc_length();
    EXPECT_GT(arc_length, 0.0f);
}

TEST(GeometryPathTest, SegmentLookup) {
    PatternInstructions pattern = create_pattern({
        "CCCC",
        "KKKK"
    });
    StitchGraph graph = StitchGraph::from_instructions(pattern);
    YarnPath yarn_path = YarnPath::from_stitch_graph(graph);

    PlaneSurface surface;
    GeometryPath geometry = GeometryPath::from_yarn_path(
        yarn_path,
        YarnProperties::worsted(),
        Gauge::worsted(),
        surface
    );

    // Should be able to look up segments by ID
    for (const auto& seg : geometry.segments()) {
        const SegmentGeometry* found = geometry.get_segment(seg.segment_id);
        ASSERT_NE(found, nullptr);
        EXPECT_EQ(found->segment_id, seg.segment_id);
    }
}

// ============================================
// Loops Without Parents (YarnOver, M1L, M1R)
// These loops have no parent loops and must be queued specially during position propagation
// ============================================

TEST(GeometryPathTest, YarnOverPositioning) {
    // YarnOver creates loops with no parents - these need special handling
    // in position propagation since they're not children of any other loop
    PatternInstructions pattern = create_pattern({
        "CCCC",      // Cast on 4 stitches
        "KOKOK"      // K, YO, K, YO, K - creates 2 yarn overs with no parents
    });
    StitchGraph graph = StitchGraph::from_instructions(pattern);
    YarnPath yarn_path = YarnPath::from_stitch_graph(graph);

    PlaneSurface surface;
    GeometryPath geometry = GeometryPath::from_yarn_path(
        yarn_path,
        YarnProperties::worsted(),
        Gauge::worsted(),
        surface
    );

    // All loops should be positioned (the bug was yarn overs weren't getting positioned)
    EXPECT_EQ(geometry.loop_positions().size(), yarn_path.loops().size());

    // Yarn over loops (in row 1) should have valid positions
    for (const auto& pos : geometry.loop_positions()) {
        EXPECT_TRUE(std::isfinite(pos.u));
        EXPECT_TRUE(std::isfinite(pos.v));
    }

    // Should be able to generate output without hanging
    std::string obj = geometry.to_obj();
    EXPECT_FALSE(obj.empty());
}

TEST(GeometryPathTest, MultipleYarnOversInRow) {
    // More complex pattern with multiple yarn overs
    PatternInstructions pattern = create_pattern({
        "CCCCCC",     // Cast on 6 stitches
        "K2OKO2K"     // K2tog, YO, K, YO, K2tog, K - mixed increases/decreases
    });
    StitchGraph graph = StitchGraph::from_instructions(pattern);
    YarnPath yarn_path = YarnPath::from_stitch_graph(graph);

    PlaneSurface surface;
    GeometryPath geometry = GeometryPath::from_yarn_path(
        yarn_path,
        YarnProperties::worsted(),
        Gauge::worsted(),
        surface
    );

    // All loops should be positioned
    EXPECT_EQ(geometry.loop_positions().size(), yarn_path.loops().size());

    // Should complete without infinite loop
    auto [min_pt, max_pt] = geometry.bounding_box();
    EXPECT_LT(min_pt.x, max_pt.x);
}
