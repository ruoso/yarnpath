#include <gtest/gtest.h>
#include "geometry.hpp"
#include "yarn_path.hpp"
#include "surface/surface.hpp"
#include "test_helpers.hpp"

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

    SurfaceSolver::solve(surface, yarn, gauge, solve_config);

    return surface;
}

// Topology-geometry integration tests verify that the topology
// (parent-child relationships) maps correctly to geometry.

TEST(TopologyGeometryTest, ParentChildInTopology) {
    PatternInstructions pattern = create_pattern({
        "CC",
        "KK"
    });
    StitchGraph graph = StitchGraph::from_instructions(pattern);
    YarnPath yarn_path = YarnPath::from_stitch_graph(graph, default_yarn(), default_gauge());

    // Count loop segments with parents
    size_t with_parents = 0;
    size_t without_parents = 0;

    for (const auto& seg : yarn_path.segments()) {
        if (seg.forms_loop) {
            if (seg.through.empty()) {
                without_parents++;
            } else {
                with_parents++;
            }
        }
    }

    EXPECT_EQ(without_parents, 2u);  // 2 cast-on
    EXPECT_EQ(with_parents, 2u);     // 2 knit
}

TEST(TopologyGeometryTest, GeometryFromTopology) {
    PatternInstructions pattern = create_pattern({
        "CCC",
        "KKK",
        "PPP"
    });
    StitchGraph graph = StitchGraph::from_instructions(pattern);
    YarnPath yarn_path = YarnPath::from_stitch_graph(graph, default_yarn(), default_gauge());

    YarnProperties yarn = YarnProperties::worsted();
    Gauge gauge = Gauge::worsted();
    SurfaceGraph surface = build_test_surface(yarn_path, yarn, gauge);

    GeometryPath geometry = GeometryPath::from_yarn_path(
        yarn_path,
        surface,
        yarn,
        gauge
    );

    // Geometry should have same number of segments as topology
    EXPECT_EQ(geometry.segments().size(), yarn_path.segments().size());
}

TEST(TopologyGeometryTest, VerticalStacking) {
    PatternInstructions pattern = create_pattern({
        "CCC",
        "KKK",
        "KKK"
    });
    StitchGraph graph = StitchGraph::from_instructions(pattern);
    YarnPath yarn_path = YarnPath::from_stitch_graph(graph, default_yarn(), default_gauge());

    YarnProperties yarn = YarnProperties::worsted();
    Gauge gauge = Gauge::worsted();
    SurfaceGraph surface = build_test_surface(yarn_path, yarn, gauge);

    GeometryPath geometry = GeometryPath::from_yarn_path(
        yarn_path,
        surface,
        yarn,
        gauge
    );

    // Get bounding box - should have vertical extent
    auto [min_pt, max_pt] = geometry.bounding_box();
    EXPECT_GT(max_pt.y - min_pt.y, 0.0f);  // Non-zero height
}
