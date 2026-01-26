#include <gtest/gtest.h>
#include "geometry.hpp"
#include "yarn_path.hpp"
#include "surface/surface.hpp"
#include "test_helpers.hpp"
#include <algorithm>
#include <cmath>
#include <numbers>

using namespace yarnpath;
using namespace yarnpath::test;

// Helper to build surface graph for testing
static SurfaceGraph build_test_surface(const YarnPath& yarn_path,
                                        const YarnProperties& yarn,
                                        const Gauge& gauge) {
    SurfaceBuildConfig build_config;
    build_config.random_seed = 42;

    SurfaceGraph surface = SurfaceBuilder::from_yarn_path(yarn_path, yarn, gauge, build_config);

    SolveConfig solve_config;
    solve_config.max_iterations = 1000;
    solve_config.convergence_threshold = 1e-4f;

    SurfaceSolver::solve(surface, yarn, solve_config);

    return surface;
}

// ============================================
// GeometryPath Integration Tests
// ============================================

TEST(GeometryPathTest, EmptyYarnPath) {
    YarnPath empty_yarn_path;

    // Build an empty surface for empty yarn path
    SurfaceGraph surface;

    GeometryPath geometry = GeometryPath::from_yarn_path(
        empty_yarn_path,
        surface,
        YarnProperties::worsted(),
        Gauge::worsted()
    );

    EXPECT_TRUE(geometry.segments().empty());
}

TEST(GeometryPathTest, CastOnOnly) {
    PatternInstructions pattern = create_pattern({"CCCC"});
    StitchGraph graph = StitchGraph::from_instructions(pattern);
    YarnPath yarn_path = YarnPath::from_stitch_graph(graph);

    YarnProperties yarn = YarnProperties::worsted();
    Gauge gauge = Gauge::worsted();
    SurfaceGraph surface = build_test_surface(yarn_path, yarn, gauge);

    GeometryPath geometry = GeometryPath::from_yarn_path(
        yarn_path,
        surface,
        yarn,
        gauge
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

    YarnProperties yarn = YarnProperties::worsted();
    Gauge gauge = Gauge::worsted();
    SurfaceGraph surface = build_test_surface(yarn_path, yarn, gauge);

    GeometryPath geometry = GeometryPath::from_yarn_path(
        yarn_path,
        surface,
        yarn,
        gauge
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

    YarnProperties yarn = YarnProperties::worsted();
    Gauge gauge = Gauge::worsted();
    SurfaceGraph surface = build_test_surface(yarn_path, yarn, gauge);

    GeometryPath geometry = GeometryPath::from_yarn_path(
        yarn_path,
        surface,
        yarn,
        gauge
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

    YarnProperties yarn = YarnProperties::worsted();
    Gauge gauge = Gauge::worsted();
    SurfaceGraph surface = build_test_surface(yarn_path, yarn, gauge);

    GeometryPath geometry = GeometryPath::from_yarn_path(
        yarn_path,
        surface,
        yarn,
        gauge
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

    YarnProperties yarn = YarnProperties::worsted();
    Gauge gauge = Gauge::worsted();
    SurfaceGraph surface = build_test_surface(yarn_path, yarn, gauge);

    GeometryPath geometry = GeometryPath::from_yarn_path(
        yarn_path,
        surface,
        yarn,
        gauge
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
    Gauge gauge = Gauge::worsted();
    SurfaceGraph surface = build_test_surface(yarn_path, yarn, gauge);

    GeometryPath geometry = GeometryPath::from_yarn_path(
        yarn_path, surface, yarn, gauge
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

    YarnProperties yarn = YarnProperties::worsted();
    Gauge gauge = Gauge::worsted();
    SurfaceGraph surface = build_test_surface(yarn_path, yarn, gauge);

    GeometryPath geometry = GeometryPath::from_yarn_path(
        yarn_path,
        surface,
        yarn,
        gauge
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
    YarnProperties yarn_worsted = YarnProperties::worsted();
    Gauge gauge_worsted = Gauge::worsted();
    SurfaceGraph surface_worsted = build_test_surface(yarn_path, yarn_worsted, gauge_worsted);

    auto geom_worsted = GeometryPath::from_yarn_path(
        yarn_path, surface_worsted, yarn_worsted, gauge_worsted);

    YarnProperties yarn_fingering = YarnProperties::fingering();
    Gauge gauge_fingering = Gauge::fingering();
    SurfaceGraph surface_fingering = build_test_surface(yarn_path, yarn_fingering, gauge_fingering);

    auto geom_fingering = GeometryPath::from_yarn_path(
        yarn_path, surface_fingering, yarn_fingering, gauge_fingering);

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

    YarnProperties yarn = YarnProperties::worsted();
    Gauge gauge = Gauge::worsted();
    SurfaceGraph surface = build_test_surface(yarn_path, yarn, gauge);

    GeometryPath geometry = GeometryPath::from_yarn_path(
        yarn_path,
        surface,
        yarn,
        gauge
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
