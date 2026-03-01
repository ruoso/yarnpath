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

static float angle_between_degrees(const Vec3& a, const Vec3& b) {
    const float a_len = a.length();
    const float b_len = b.length();
    if (a_len < 1e-9f || b_len < 1e-9f) return 0.0f;

    float dot = a.normalized().dot(b.normalized());
    dot = std::max(-1.0f, std::min(1.0f, dot));
    constexpr float kRadToDeg = 57.29577951308232f;
    return std::acos(dot) * kRadToDeg;
}

TEST(GeometryJoinContinuityReferenceExt, C0AdjacentSegmentEndpointsShouldMatchExactly) {
    YarnProperties yarn = default_yarn();
    Gauge gauge = default_gauge();

    GeometryPath geometry = build_geometry_for_pattern(
        {"CCCCCC", "KKPPKK", "PPKKPP", "KKPPKK", "PPKKPP"},
        yarn,
        gauge);

    ASSERT_GT(geometry.segments().size(), 2u);

    for (size_t i = 0; i + 1 < geometry.segments().size(); ++i) {
        const auto& seg_a = geometry.segments()[i];
        const auto& seg_b = geometry.segments()[i + 1];

        ASSERT_FALSE(seg_a.curve.empty());
        ASSERT_FALSE(seg_b.curve.empty());

        const float ta = static_cast<float>(seg_a.curve.segment_count());
        const Vec3 end_a = seg_a.curve.evaluate(ta);
        const Vec3 start_b = seg_b.curve.evaluate(0.0f);

        const float gap = (end_a - start_b).length();

        // Intentionally strict reference condition for step-join continuity.
        EXPECT_LT(gap, 1e-6f)
            << "C0 join gap between segment " << i << " and " << (i + 1)
            << " is " << gap;
    }
}

TEST(GeometryJoinContinuityReferenceExt, C1AdjacentSegmentTangentAngleShouldBeVerySmall) {
    YarnProperties yarn = default_yarn();
    Gauge gauge = default_gauge();

    GeometryPath geometry = build_geometry_for_pattern(
        {"CCCCCC", "KPKPKP", "PKPKPK", "KPKPKP", "PKPKPK"},
        yarn,
        gauge);

    ASSERT_GT(geometry.segments().size(), 2u);

    for (size_t i = 0; i + 1 < geometry.segments().size(); ++i) {
        const auto& seg_a = geometry.segments()[i];
        const auto& seg_b = geometry.segments()[i + 1];

        ASSERT_FALSE(seg_a.curve.empty());
        ASSERT_FALSE(seg_b.curve.empty());

        const float ta = static_cast<float>(seg_a.curve.segment_count());
        const Vec3 tan_a = seg_a.curve.tangent(ta);
        const Vec3 tan_b = seg_b.curve.tangent(0.0f);

        const float angle_deg = angle_between_degrees(tan_a, tan_b);

        // Intentionally strict reference threshold (likely red on current impl).
        EXPECT_LT(angle_deg, 6.0f)
            << "C1 tangent discontinuity between segment " << i << " and " << (i + 1)
            << ": angle=" << angle_deg << " deg";
    }
}

TEST(GeometryJoinContinuityReferenceExt, AdjacentSegmentCurvatureJumpShouldBeMinimal) {
    YarnProperties yarn = default_yarn();
    Gauge gauge = default_gauge();

    GeometryPath geometry = build_geometry_for_pattern(
        {"CCCCCC", "KKKKKK", "PPPPPP", "KKKKKK", "PPPPPP"},
        yarn,
        gauge);

    ASSERT_GT(geometry.segments().size(), 2u);

    for (size_t i = 0; i + 1 < geometry.segments().size(); ++i) {
        const auto& seg_a = geometry.segments()[i];
        const auto& seg_b = geometry.segments()[i + 1];

        ASSERT_FALSE(seg_a.curve.empty());
        ASSERT_FALSE(seg_b.curve.empty());

        const auto& bez_a = seg_a.curve.segments().back();
        const auto& bez_b = seg_b.curve.segments().front();

        const float k_a = bez_a.curvature(1.0f);
        const float k_b = bez_b.curvature(0.0f);

        ASSERT_TRUE(std::isfinite(k_a));
        ASSERT_TRUE(std::isfinite(k_b));

        const float jump = std::fabs(k_a - k_b);

        // Optional higher-order continuity check, intentionally strict.
        EXPECT_LT(jump, 0.08f)
            << "Curvature jump too large at boundary " << i << "->" << (i + 1)
            << ": |k_end-k_start|=" << jump
            << " (k_end=" << k_a << ", k_start=" << k_b << ")";
    }
}
