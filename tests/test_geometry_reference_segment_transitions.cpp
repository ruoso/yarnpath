#include <gtest/gtest.h>
#include "geometry.hpp"
#include "yarn_path.hpp"
#include "surface/surface.hpp"
#include "test_helpers.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
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

static float angle_between_degrees(const Vec3& a, const Vec3& b) {
    const float a_len = a.length();
    const float b_len = b.length();
    if (a_len < 1e-8f || b_len < 1e-8f) return 0.0f;

    float dot = a.normalized().dot(b.normalized());
    dot = std::max(-1.0f, std::min(1.0f, dot));
    return std::acos(dot) * 180.0f / std::numbers::pi_v<float>;
}

TEST(GeometryTransitionReference, AdjacentSegmentTangentsShouldBeNearC1Continuous) {
    YarnProperties yarn = default_yarn();
    Gauge gauge = default_gauge();

    GeometryPath geometry = build_geometry_for_pattern(
        {"CCCCCC", "KKPPKK", "PPKKPP", "KKPPKK"},
        yarn,
        gauge);

    ASSERT_GT(geometry.segments().size(), 2u);

    for (size_t i = 0; i + 1 < geometry.segments().size(); ++i) {
        const auto& a = geometry.segments()[i];
        const auto& b = geometry.segments()[i + 1];

        if (a.curve.empty() || b.curve.empty()) continue;

        const float ta = static_cast<float>(a.curve.segment_count());
        const Vec3 tan_a = a.curve.tangent(ta);
        const Vec3 tan_b = b.curve.tangent(0.0f);

        const float turn_angle = angle_between_degrees(tan_a, tan_b);

        // Reference expectation: transitions should be visually smooth.
        // Intentionally strict to expose sharp turns at segment boundaries.
        EXPECT_LT(turn_angle, 20.0f)
            << "Sharp boundary between segment " << i << " and " << (i + 1)
            << ", angle=" << turn_angle;
    }
}

TEST(GeometryTransitionReference, PolylineLocalTurnShouldAvoidSharpCorners) {
    YarnProperties yarn = default_yarn();
    Gauge gauge = default_gauge();

    GeometryPath geometry = build_geometry_for_pattern(
        {"CCCC", "KKKK", "PPPP", "KKKK", "PPPP"},
        yarn,
        gauge);

    const auto polyline = geometry.to_polyline_fixed(20);
    ASSERT_GT(polyline.size(), 10u);

    for (size_t i = 1; i + 1 < polyline.size(); ++i) {
        const Vec3 vin = polyline[i] - polyline[i - 1];
        const Vec3 vout = polyline[i + 1] - polyline[i];

        if (vin.length() < 1e-6f || vout.length() < 1e-6f) continue;

        const float turn_angle = angle_between_degrees(vin, vout);

        // Reference expectation: no visibly sharp corners in yarn centerline.
        // Intentionally strict and expected to fail on current implementation.
        EXPECT_LT(turn_angle, 30.0f)
            << "Sharp polyline corner at sample " << i
            << ", angle=" << turn_angle;
    }
}
