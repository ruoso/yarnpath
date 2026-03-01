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

static float mirrored_point_error_mean(const std::vector<Vec3>& points, float center_x) {
    if (points.empty()) return 0.0f;

    float error_sum = 0.0f;
    for (const auto& p : points) {
        float best = std::numeric_limits<float>::max();
        for (const auto& q : points) {
            const float mirror_x_error = std::abs((p.x + q.x) - 2.0f * center_x);
            const float y_error = std::abs(p.y - q.y);
            const float z_error = std::abs(p.z - q.z);
            best = std::min(best, mirror_x_error + y_error + z_error);
        }
        error_sum += best;
    }
    return error_sum / static_cast<float>(points.size());
}

TEST(GeometryScaleReference, DoublingNeedleDiameterShouldDoubleHeightPrecisely) {
    YarnProperties yarn = default_yarn();

    Gauge g1{5.0f};
    Gauge g2{10.0f};

    GeometryPath geom1 = build_geometry_for_pattern(
        {"CCCC", "KKKK", "PPPP", "KKKK"},
        yarn,
        g1);
    GeometryPath geom2 = build_geometry_for_pattern(
        {"CCCC", "KKKK", "PPPP", "KKKK"},
        yarn,
        g2);

    auto [min1, max1] = geom1.bounding_box();
    auto [min2, max2] = geom2.bounding_box();

    const float h1 = max1.y - min1.y;
    const float h2 = max2.y - min2.y;
    ASSERT_GT(h1, 1e-6f);

    const float ratio = h2 / h1;

    // Reference expectation: scale should be almost exactly linear in gauge.
    EXPECT_NEAR(ratio, 2.0f, 0.02f);
}

TEST(GeometryScaleReference, DoublingNeedleDiameterShouldDoubleWidthPrecisely) {
    YarnProperties yarn = default_yarn();

    Gauge g1{5.0f};
    Gauge g2{10.0f};

    GeometryPath geom1 = build_geometry_for_pattern(
        {"CCCC", "KKKK", "PPPP", "KKKK"},
        yarn,
        g1);
    GeometryPath geom2 = build_geometry_for_pattern(
        {"CCCC", "KKKK", "PPPP", "KKKK"},
        yarn,
        g2);

    auto [min1, max1] = geom1.bounding_box();
    auto [min2, max2] = geom2.bounding_box();

    const float w1 = max1.x - min1.x;
    const float w2 = max2.x - min2.x;
    ASSERT_GT(w1, 1e-6f);

    const float ratio = w2 / w1;

    EXPECT_NEAR(ratio, 2.0f, 0.02f);
}

TEST(GeometrySymmetryReference, StockinetteSwatchShouldBeMirrorSymmetricInX) {
    YarnProperties yarn = default_yarn();
    Gauge gauge = default_gauge();

    GeometryPath geometry = build_geometry_for_pattern(
        {"CCCC", "KKKK", "PPPP", "KKKK", "PPPP"},
        yarn,
        gauge);

    auto polyline = geometry.to_polyline_fixed(18);
    ASSERT_FALSE(polyline.empty());

    auto [min_pt, max_pt] = geometry.bounding_box();
    const float cx = (min_pt.x + max_pt.x) * 0.5f;

    const float mean_mirror_error = mirrored_point_error_mean(polyline, cx);

    // Reference expectation for an even-width plain swatch: close mirror symmetry.
    EXPECT_LT(mean_mirror_error, 0.02f * yarn.compressed_radius);
}
