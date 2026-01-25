#include <gtest/gtest.h>
#include "geometry.hpp"
#include "yarn_path.hpp"
#include "test_helpers.hpp"

using namespace yarnpath;
using namespace yarnpath::test;

// Loop shape tests verify that geometry is generated correctly
// for different stitch patterns.

TEST(LoopShapeTest, BasicGeometryGeneration) {
    PatternInstructions pattern = create_pattern({
        "CCC",
        "KKK"
    });
    StitchGraph graph = StitchGraph::from_instructions(pattern);
    YarnPath yarn_path = YarnPath::from_stitch_graph(graph);

    GeometryPath geometry = GeometryPath::from_yarn_path(
        yarn_path,
        YarnProperties::worsted(),
        Gauge::worsted()
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
    GeometryPath geometry = GeometryPath::from_yarn_path(
        yarn_path, yarn, Gauge::worsted()
    );

    // Check that max curvature is reasonable
    for (const auto& seg : geometry.segments()) {
        // Allow some tolerance above yarn limit (bezier approximation)
        EXPECT_LT(seg.max_curvature, yarn.max_curvature() * 2.0f);
    }
}
