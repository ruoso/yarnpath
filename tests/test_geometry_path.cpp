#include <gtest/gtest.h>
#include "geometry.hpp"
#include "yarn_path.hpp"
#include "test_helpers.hpp"
#include <algorithm>
#include <cmath>
#include <numbers>

using namespace yarnpath;
using namespace yarnpath::test;

// ============================================
// GeometryPath Integration Tests
// ============================================

TEST(GeometryPathTest, EmptyYarnPath) {
    YarnPath empty_yarn_path;

    GeometryPath geometry = GeometryPath::from_yarn_path(
        empty_yarn_path,
        YarnProperties::worsted(),
        Gauge::worsted()
    );

    EXPECT_TRUE(geometry.segments().empty());
}

TEST(GeometryPathTest, CastOnOnly) {
    PatternInstructions pattern = create_pattern({"CCCC"});
    StitchGraph graph = StitchGraph::from_instructions(pattern);
    YarnPath yarn_path = YarnPath::from_stitch_graph(graph);

    GeometryPath geometry = GeometryPath::from_yarn_path(
        yarn_path,
        YarnProperties::worsted(),
        Gauge::worsted()
    );

    // Should have geometry segments
    EXPECT_FALSE(geometry.segments().empty());

    // Can convert to polyline
    auto polyline = geometry.to_polyline_fixed(10);
    EXPECT_FALSE(polyline.empty());
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

    GeometryPath geometry = GeometryPath::from_yarn_path(
        yarn_path,
        YarnProperties::worsted(),
        Gauge::worsted()
    );

    // Should have geometry segments
    EXPECT_FALSE(geometry.segments().empty());

    // Polyline should have many points
    auto polyline = geometry.to_polyline_fixed(10);
    EXPECT_GT(polyline.size(), 10u);
}

TEST(GeometryPathTest, PolylineGeneration) {
    PatternInstructions pattern = create_pattern({
        "CC",
        "KK"
    });
    StitchGraph graph = StitchGraph::from_instructions(pattern);
    YarnPath yarn_path = YarnPath::from_stitch_graph(graph);

    GeometryPath geometry = GeometryPath::from_yarn_path(
        yarn_path,
        YarnProperties::worsted(),
        Gauge::worsted()
    );

    // Test fixed samples polyline
    auto polyline_fixed = geometry.to_polyline_fixed(5);
    EXPECT_FALSE(polyline_fixed.empty());

    // Test arc-length based polyline
    auto polyline_arc = geometry.to_polyline(0.5f);
    EXPECT_FALSE(polyline_arc.empty());
}

TEST(GeometryPathTest, BoundingBox) {
    PatternInstructions pattern = create_pattern({
        "CCCC",
        "KKKK",
        "PPPP"
    });
    StitchGraph graph = StitchGraph::from_instructions(pattern);
    YarnPath yarn_path = YarnPath::from_stitch_graph(graph);

    GeometryPath geometry = GeometryPath::from_yarn_path(
        yarn_path,
        YarnProperties::worsted(),
        Gauge::worsted()
    );

    auto [min_pt, max_pt] = geometry.bounding_box();

    // Bounding box should have positive extent
    EXPECT_LT(min_pt.x, max_pt.x);
    EXPECT_LE(min_pt.y, max_pt.y);  // May be equal for flat fabric
}

TEST(GeometryPathTest, TotalArcLength) {
    PatternInstructions pattern = create_pattern({
        "CC",
        "KK"
    });
    StitchGraph graph = StitchGraph::from_instructions(pattern);
    YarnPath yarn_path = YarnPath::from_stitch_graph(graph);

    GeometryPath geometry = GeometryPath::from_yarn_path(
        yarn_path,
        YarnProperties::worsted(),
        Gauge::worsted()
    );

    float total_length = geometry.total_arc_length();
    EXPECT_GT(total_length, 0.0f);
}

TEST(GeometryPathTest, Validation) {
    PatternInstructions pattern = create_pattern({
        "CCC",
        "KKK"
    });
    StitchGraph graph = StitchGraph::from_instructions(pattern);
    YarnPath yarn_path = YarnPath::from_stitch_graph(graph);

    YarnProperties yarn = YarnProperties::worsted();
    GeometryPath geometry = GeometryPath::from_yarn_path(
        yarn_path, yarn, Gauge::worsted()
    );

    ValidationResult result = geometry.validate(yarn);
    // Just check it runs - validation may have warnings
    EXPECT_TRUE(result.valid || !result.warnings.empty() || !result.errors.empty());
}

TEST(GeometryPathTest, ObjExport) {
    PatternInstructions pattern = create_pattern({
        "CC",
        "KK"
    });
    StitchGraph graph = StitchGraph::from_instructions(pattern);
    YarnPath yarn_path = YarnPath::from_stitch_graph(graph);

    GeometryPath geometry = GeometryPath::from_yarn_path(
        yarn_path,
        YarnProperties::worsted(),
        Gauge::worsted()
    );

    std::string obj = geometry.to_obj(5);

    // Should contain OBJ format elements
    EXPECT_NE(obj.find("v "), std::string::npos);  // Vertices
    EXPECT_NE(obj.find("l "), std::string::npos);  // Lines
}

TEST(GeometryPathTest, DifferentYarnProperties) {
    PatternInstructions pattern = create_pattern({
        "CCC",
        "KKK"
    });
    StitchGraph graph = StitchGraph::from_instructions(pattern);
    YarnPath yarn_path = YarnPath::from_stitch_graph(graph);

    // Generate with different yarn types
    auto geom_worsted = GeometryPath::from_yarn_path(
        yarn_path, YarnProperties::worsted(), Gauge::worsted());

    auto geom_fingering = GeometryPath::from_yarn_path(
        yarn_path, YarnProperties::fingering(), Gauge::fingering());

    // Both should generate geometry
    EXPECT_FALSE(geom_worsted.segments().empty());
    EXPECT_FALSE(geom_fingering.segments().empty());

    // Arc lengths should differ (different yarn/gauge)
    EXPECT_NE(geom_worsted.total_arc_length(), geom_fingering.total_arc_length());
}

TEST(GeometryPathTest, SegmentGeometryAccess) {
    PatternInstructions pattern = create_pattern({
        "CC",
        "KK"
    });
    StitchGraph graph = StitchGraph::from_instructions(pattern);
    YarnPath yarn_path = YarnPath::from_stitch_graph(graph);

    GeometryPath geometry = GeometryPath::from_yarn_path(
        yarn_path,
        YarnProperties::worsted(),
        Gauge::worsted()
    );

    // Access individual segments
    for (size_t i = 0; i < geometry.segments().size(); ++i) {
        const SegmentGeometry* seg = geometry.get_segment(static_cast<SegmentId>(i));
        ASSERT_NE(seg, nullptr);
        EXPECT_EQ(seg->segment_id, i);
        EXPECT_GE(seg->arc_length, 0.0f);
    }

    // Out of bounds returns nullptr
    EXPECT_EQ(geometry.get_segment(10000), nullptr);
}
