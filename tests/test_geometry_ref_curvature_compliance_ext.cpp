#include <gtest/gtest.h>

#include "geometry.hpp"
#include "surface/surface.hpp"
#include "test_helpers.hpp"
#include "yarn_path.hpp"

#include <cmath>
#include <sstream>
#include <string>
#include <vector>

using namespace yarnpath;
using namespace yarnpath::test;

namespace {

struct CurvatureViolation {
    size_t segment_idx;
    size_t bezier_idx;
    float t;
    float curvature;
    float max_allowed;
    float excess;
    Vec3 position;
};

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

static std::vector<CurvatureViolation> enumerate_curvature_violations(
    const GeometryPath& geometry,
    const YarnProperties& yarn,
    int samples_per_bezier,
    float tolerance_factor = 1.0f,
    float absolute_tolerance = 1e-5f) {

    std::vector<CurvatureViolation> violations;
    const float max_allowed = yarn.max_curvature() * tolerance_factor;

    for (size_t seg_idx = 0; seg_idx < geometry.segments().size(); ++seg_idx) {
        const auto& seg = geometry.segments()[seg_idx];
        const auto& beziers = seg.curve.segments();

        for (size_t bez_idx = 0; bez_idx < beziers.size(); ++bez_idx) {
            const auto& bez = beziers[bez_idx];

            for (int i = 0; i <= samples_per_bezier; ++i) {
                const float t = static_cast<float>(i) / static_cast<float>(samples_per_bezier);
                const float k = bez.curvature(t);
                if (!std::isfinite(k)) {
                    continue;
                }

                const float excess = k - max_allowed;
                if (excess > absolute_tolerance) {
                    violations.push_back(CurvatureViolation{
                        .segment_idx = seg_idx,
                        .bezier_idx = bez_idx,
                        .t = t,
                        .curvature = k,
                        .max_allowed = max_allowed,
                        .excess = excess,
                        .position = bez.evaluate(t),
                    });
                }
            }
        }
    }

    return violations;
}

static std::string format_violation_summary(const std::vector<CurvatureViolation>& violations,
                                            size_t max_lines = 12) {
    std::ostringstream oss;
    const size_t count = std::min(max_lines, violations.size());
    for (size_t i = 0; i < count; ++i) {
        const auto& v = violations[i];
        oss << "\n  [" << i << "] seg=" << v.segment_idx
            << " bez=" << v.bezier_idx
            << " t=" << v.t
            << " k=" << v.curvature
            << " max=" << v.max_allowed
            << " excess=" << v.excess
            << " pos=(" << v.position.x << "," << v.position.y << "," << v.position.z << ")";
    }
    if (violations.size() > max_lines) {
        oss << "\n  ... and " << (violations.size() - max_lines) << " more";
    }
    return oss.str();
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

    ASSERT_FALSE(geometry.segments().empty());

    const auto violations = enumerate_curvature_violations(
        geometry,
        yarn,
        32,
        1.0f,
        1e-6f);

    EXPECT_TRUE(violations.empty())
        << "Strict stockinette curvature compliance failed. violations=" << violations.size()
        << format_violation_summary(violations);
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

    ASSERT_FALSE(geometry.segments().empty());

    const auto violations = enumerate_curvature_violations(
        geometry,
        yarn,
        36,
        1.0f,
        1e-6f);

    EXPECT_TRUE(violations.empty())
        << "Strict ribbing/mixed curvature compliance failed. violations=" << violations.size()
        << format_violation_summary(violations);
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

    ASSERT_FALSE(geometry.segments().empty());

    const auto violations = enumerate_curvature_violations(
        geometry,
        yarn,
        40,
        1.0f,
        1e-6f);

    EXPECT_TRUE(violations.empty())
        << "Strict finer-yarn curvature compliance failed. violations=" << violations.size()
        << format_violation_summary(violations);
}
