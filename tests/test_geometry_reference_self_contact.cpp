#include <gtest/gtest.h>
#include "geometry.hpp"
#include "yarn_path.hpp"
#include "surface/surface.hpp"
#include "test_helpers.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

using namespace yarnpath;
using namespace yarnpath::test;

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

static float min_nonlocal_polyline_distance(const std::vector<Vec3>& polyline,
                                            size_t skip_neighbors) {
    if (polyline.size() < 3) return std::numeric_limits<float>::max();

    float min_dist = std::numeric_limits<float>::max();
    for (size_t i = 0; i < polyline.size(); ++i) {
        for (size_t j = i + skip_neighbors; j < polyline.size(); ++j) {
            const float d = (polyline[i] - polyline[j]).length();
            min_dist = std::min(min_dist, d);
        }
    }
    return min_dist;
}

TEST(GeometrySelfContactReference, StockinetteShouldKeepDiameterClearanceGlobally) {
    YarnProperties yarn = default_yarn();
    Gauge gauge = default_gauge();

    GeometryPath geometry = build_geometry_for_pattern(
        {"CCCCCC", "KKKKKK", "PPPPPP", "KKKKKK", "PPPPPP", "KKKKKK"},
        yarn,
        gauge);

    auto polyline = geometry.to_polyline_fixed(16);
    ASSERT_FALSE(polyline.empty());

    // Reference expectation: yarn centerline should avoid self-penetration by diameter.
    // Intentionally strict so this remains a target test during geometry improvements.
    const float min_dist = min_nonlocal_polyline_distance(polyline, 20);
    EXPECT_GT(min_dist, 2.0f * yarn.compressed_radius);
}

TEST(GeometrySelfContactReference, RibbingShouldKeepDiameterClearanceGlobally) {
    YarnProperties yarn = default_yarn();
    Gauge gauge = default_gauge();

    GeometryPath geometry = build_geometry_for_pattern(
        {"CCCCCC", "KPKPKP", "PKPKPK", "KPKPKP"},
        yarn,
        gauge);

    auto polyline = geometry.to_polyline_fixed(16);
    ASSERT_FALSE(polyline.empty());

    const float min_dist = min_nonlocal_polyline_distance(polyline, 20);
    EXPECT_GT(min_dist, 2.0f * yarn.compressed_radius);
}

TEST(GeometrySelfContactReference, YarnOverGeometryShouldNotContainNaN) {
    YarnProperties yarn = default_yarn();
    Gauge gauge = default_gauge();

    GeometryPath geometry = build_geometry_for_pattern({"CCC", "KOK"}, yarn, gauge);

    ASSERT_FALSE(geometry.segments().empty());

    for (const auto& seg : geometry.segments()) {
        EXPECT_TRUE(std::isfinite(seg.arc_length));

        for (const auto& wp : seg.curve.waypoints()) {
            EXPECT_TRUE(std::isfinite(wp.x));
            EXPECT_TRUE(std::isfinite(wp.y));
            EXPECT_TRUE(std::isfinite(wp.z));
        }
    }
}
