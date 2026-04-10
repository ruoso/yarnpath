#include <gtest/gtest.h>

#include "geometry.hpp"
#include "surface/surface.hpp"
#include "test_helpers.hpp"
#include "yarn_path.hpp"

#include <cmath>
#include <string>
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

static void expect_finite_vec3(const Vec3& p, const std::string& label) {
    EXPECT_TRUE(std::isfinite(p.x)) << label << " x is non-finite";
    EXPECT_TRUE(std::isfinite(p.y)) << label << " y is non-finite";
    EXPECT_TRUE(std::isfinite(p.z)) << label << " z is non-finite";
}

static void expect_geometry_finite(const GeometryPath& geometry) {
    ASSERT_FALSE(geometry.segments().empty());

    for (const auto& seg : geometry.segments()) {
        EXPECT_TRUE(std::isfinite(seg.arc_length))
            << "segment " << seg.segment_id << " arc_length is non-finite";

        for (const auto& wp : seg.curve.waypoints()) {
            expect_finite_vec3(wp, "waypoint");
        }
    }

    const std::vector<Vec3> polyline = geometry.to_polyline_fixed(16);
    ASSERT_FALSE(polyline.empty());
    for (const auto& p : polyline) {
        expect_finite_vec3(p, "polyline point");
    }

    const auto [bb_min, bb_max] = geometry.bounding_box();
    expect_finite_vec3(bb_min, "bounding box min");
    expect_finite_vec3(bb_max, "bounding box max");
}

TEST(GeometryRefNumericalRobustness, AdversarialPatternsProduceFiniteGeometryMetrics) {
    const YarnProperties yarn = default_yarn();
    const Gauge gauge = default_gauge();

    // YO-heavy, decrease-heavy, and mixed transition adversarial patterns.
    const std::vector<std::pair<std::string, std::vector<std::string>>> cases = {
        {"yo-heavy", {"CCCCCC", "KO2OSOK"}},
        {"decrease-heavy", {"CCCCCCCC", "22SS"}},
        {"mixed-transitions", {"CCCCCCCC", "K2OKSKK", "KPKPKPK"}},
    };

    for (const auto& [name, rows] : cases) {
        SCOPED_TRACE(name);
        const GeometryPath geometry = build_geometry_for_pattern(rows, yarn, gauge);
        expect_geometry_finite(geometry);
    }
}

TEST(GeometryRefNumericalRobustness, ValidateExecutesAndReportsDeterministically) {
    const YarnProperties yarn = default_yarn();
    const Gauge gauge = default_gauge();

    const GeometryPath geometry = build_geometry_for_pattern(
        {"CCCCCCCC", "K2OKSKK", "KPKPKPK"},
        yarn,
        gauge);

    ASSERT_FALSE(geometry.segments().empty());

    ValidationResult first = geometry.validate(yarn);
    ValidationResult second = geometry.validate(yarn);

    EXPECT_EQ(first.valid, second.valid);
    EXPECT_EQ(first.warnings, second.warnings);
    EXPECT_EQ(first.errors, second.errors);
}
