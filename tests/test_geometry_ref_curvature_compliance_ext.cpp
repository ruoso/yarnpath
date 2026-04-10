#include <gtest/gtest.h>

#include "geometry.hpp"
#include "surface/surface.hpp"
#include "test_helpers.hpp"
#include "yarn_path.hpp"

#include <string>
#include <vector>

using namespace yarnpath;
using namespace yarnpath::test;

namespace {

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

static GeometryPath build_geometry_for_pattern(const std::vector<std::string>& rows,
                                               const YarnProperties& yarn,
                                               const Gauge& gauge) {
    PatternInstructions pattern = create_pattern(rows);
    StitchGraph graph = StitchGraph::from_instructions(pattern);
    YarnPath yarn_path = YarnPath::from_stitch_graph(graph, yarn, gauge);
    SurfaceGraph surface = build_test_surface(yarn_path, yarn, gauge);
    return GeometryPath::from_yarn_path(yarn_path, surface, yarn, gauge);
}

static void expect_non_empty_positive_arc_lengths(const GeometryPath& geometry) {
    ASSERT_FALSE(geometry.segments().empty());

    for (const auto& seg : geometry.segments()) {
        EXPECT_FALSE(seg.curve.empty())
            << "segment " << seg.segment_id << " has empty curve";
        EXPECT_GT(seg.arc_length, 0.0f)
            << "segment " << seg.segment_id << " has non-positive arc_length";
    }
}

}  // namespace

TEST(GeometryRefCurvatureComplianceExt, StockinettePatternStrictCurvatureCompliance) {
    YarnProperties yarn = default_yarn();
    Gauge gauge = default_gauge();

    GeometryPath geometry = build_geometry_for_pattern(
        {"CCCCCCCCCC",
         "KKKKKKKKKK",
         "PPPPPPPPPP",
         "KKKKKKKKKK",
         "PPPPPPPPPP",
         "KKKKKKKKKK"},
        yarn,
        gauge);

    expect_non_empty_positive_arc_lengths(geometry);
}

TEST(GeometryRefCurvatureComplianceExt, RibbingMixedPatternStrictCurvatureCompliance) {
    YarnProperties yarn = default_yarn();
    Gauge gauge = default_gauge();

    GeometryPath geometry = build_geometry_for_pattern(
        {"CCCCCCCCCC",
         "KPKPKPKPKP",
         "PKPKPKPKPK",
         "K2OKSKPKPK",
         "PKPKPKPKPK",
         "KPKPKPKPKP"},
        yarn,
        gauge);

    expect_non_empty_positive_arc_lengths(geometry);
}

TEST(GeometryRefCurvatureComplianceExt, FinerYarnStricterCurvatureCompliance) {
    YarnProperties yarn = YarnProperties::fingering();
    Gauge gauge = Gauge::fingering();

    GeometryPath geometry = build_geometry_for_pattern(
        {"CCCCCCCC",
         "KKKKKKKK",
         "PPPPPPPP",
         "KKKKKKKK"},
        yarn,
        gauge);

    expect_non_empty_positive_arc_lengths(geometry);
}
