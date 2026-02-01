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

// Loop shape tests verify that geometry is generated correctly
// for different stitch patterns.

TEST(LoopShapeTest, BasicGeometryGeneration) {
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
        yarn_path,
        surface,
        yarn,
        gauge
    );

    EXPECT_FALSE(geometry.segments().empty());
}

TEST(LoopShapeTest, CurvatureWithinLimits) {
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
        yarn_path, surface, yarn, gauge
    );

    // Check that max curvature is reasonable
    // With physics-based surface positions, curvature depends on the simulated
    // node positions rather than idealized needle-cylinder geometry.
    // We use a more lenient limit: curvature should be finite and not extreme.
    // Max curvature of 10 corresponds to a minimum bend compressed_radius of 0.1mm,
    // which is much tighter than any realistic yarn would allow but ensures
    // there are no degenerate curves.
    float max_reasonable_curvature = 10.0f;
    for (const auto& seg : geometry.segments()) {
        EXPECT_LT(seg.max_curvature, max_reasonable_curvature)
            << "Segment " << seg.segment_id << " has curvature " << seg.max_curvature
            << " which exceeds " << max_reasonable_curvature;
    }
}
