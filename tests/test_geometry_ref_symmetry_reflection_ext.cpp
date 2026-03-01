#include <gtest/gtest.h>

#include "geometry.hpp"
#include "surface/surface.hpp"
#include "test_helpers.hpp"
#include "yarn_path.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
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

// Nearest mirrored-point error in X about center_x.
// For each point p, find q minimizing |x_p + x_q - 2*center_x| + |y_p - y_q| + |z_p - z_q|.
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

}  // namespace

TEST(GeometryRefSymmetryReflectionExt, EvenStockinetteShouldBeApproximatelyMirrorSymmetricInX) {
    const YarnProperties yarn = default_yarn();
    const Gauge gauge = default_gauge();

    // Even-width symmetric stockinette swatch.
    const GeometryPath geometry = build_geometry_for_pattern(
        {"CCCCCCCC", "KKKKKKKK", "PPPPPPPP", "KKKKKKKK", "PPPPPPPP", "KKKKKKKK"},
        yarn,
        gauge);

    const auto polyline = geometry.to_polyline_fixed(24);
    ASSERT_FALSE(polyline.empty());

    const auto [min_pt, max_pt] = geometry.bounding_box();
    const float center_x = 0.5f * (min_pt.x + max_pt.x);

    const float mean_mirror_error = mirrored_point_error_mean(polyline, center_x);

    EXPECT_LT(mean_mirror_error, 0.90f * yarn.compressed_radius)
        << "Stockinette mirror error too high: " << mean_mirror_error;
}

TEST(GeometryRefSymmetryReflectionExt, EvenRibbingShouldBeApproximatelyMirrorSymmetricInX) {
    const YarnProperties yarn = default_yarn();
    const Gauge gauge = default_gauge();

    // Even-width 2x2 rib-like alternation (symmetric under left-right reflection).
    const GeometryPath geometry = build_geometry_for_pattern(
        {"CCCCCCCC", "KKPPKKPP", "PPKKPPKK", "KKPPKKPP", "PPKKPPKK", "KKPPKKPP"},
        yarn,
        gauge);

    const auto polyline = geometry.to_polyline_fixed(24);
    ASSERT_FALSE(polyline.empty());

    const auto [min_pt, max_pt] = geometry.bounding_box();
    const float center_x = 0.5f * (min_pt.x + max_pt.x);

    const float mean_mirror_error = mirrored_point_error_mean(polyline, center_x);

    EXPECT_LT(mean_mirror_error, 1.10f * yarn.compressed_radius)
        << "Ribbing mirror error too high: " << mean_mirror_error;
}

TEST(GeometryRefSymmetryReflectionExt, StrictReflectionToleranceForEvenStockinetteExpectedToFailCurrently) {
    const YarnProperties yarn = default_yarn();
    const Gauge gauge = default_gauge();

    const GeometryPath geometry = build_geometry_for_pattern(
        {"CCCCCCCC", "KKKKKKKK", "PPPPPPPP", "KKKKKKKK", "PPPPPPPP", "KKKKKKKK"},
        yarn,
        gauge);

    const auto polyline = geometry.to_polyline_fixed(28);
    ASSERT_FALSE(polyline.empty());

    const auto [min_pt, max_pt] = geometry.bounding_box();
    const float center_x = 0.5f * (min_pt.x + max_pt.x);

    const float mean_mirror_error = mirrored_point_error_mean(polyline, center_x);

    // Intentionally strict reference threshold (expected red on current implementation).
    EXPECT_LT(mean_mirror_error, 0.02f * yarn.compressed_radius)
        << "Strict stockinette reflection error: " << mean_mirror_error;
}
