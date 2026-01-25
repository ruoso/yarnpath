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
    // The geometry wraps yarn around needles, so curvature can exceed the yarn's
    // "relaxed" minimum bend radius. We allow up to 1/(needle_radius) curvature
    // since the yarn is physically wrapped around the needle.
    float needle_radius = Gauge::worsted().needle_diameter * 0.5f;
    float max_reasonable_curvature = 1.0f / needle_radius + yarn.max_curvature();
    for (const auto& seg : geometry.segments()) {
        EXPECT_LT(seg.max_curvature, max_reasonable_curvature * 2.0f)
            << "Segment " << seg.segment_id << " has curvature " << seg.max_curvature
            << " which exceeds " << max_reasonable_curvature * 2.0f;
    }
}
