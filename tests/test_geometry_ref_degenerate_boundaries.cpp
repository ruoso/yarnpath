#include <gtest/gtest.h>

#include "geometry.hpp"
#include "surface/surface.hpp"
#include "test_helpers.hpp"
#include "yarn_path.hpp"

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

static void expect_finite_vec3(const Vec3& p, const std::string& label) {
    EXPECT_TRUE(std::isfinite(p.x)) << label << " x is non-finite";
    EXPECT_TRUE(std::isfinite(p.y)) << label << " y is non-finite";
    EXPECT_TRUE(std::isfinite(p.z)) << label << " z is non-finite";
}

static void expect_geometry_finite(const GeometryPath& geometry) {
    for (const auto& seg : geometry.segments()) {
        EXPECT_TRUE(std::isfinite(seg.arc_length))
            << "segment " << seg.segment_id << " arc_length is non-finite";

        for (const auto& wp : seg.curve.waypoints()) {
            expect_finite_vec3(wp, "waypoint");
        }
    }

    const std::vector<Vec3> polyline = geometry.to_polyline_fixed(12);
    for (const auto& p : polyline) {
        expect_finite_vec3(p, "polyline point");
    }

    const auto [bb_min, bb_max] = geometry.bounding_box();
    expect_finite_vec3(bb_min, "bounding box min");
    expect_finite_vec3(bb_max, "bounding box max");
}

static size_t count_zero_length_segments(const GeometryPath& geometry, float eps = 1e-6f) {
    size_t degenerate = 0;

    for (const auto& seg : geometry.segments()) {
        if (seg.curve.empty()) continue;
        const float arc = seg.arc_length;
        const float chord = (seg.curve.end() - seg.curve.start()).length();
        if (arc <= eps && chord <= eps) {
            ++degenerate;
        }
    }

    return degenerate;
}

static void expect_boundary_continuity(const GeometryPath& geometry) {
    if (geometry.segments().empty()) {
        return;
    }

    const auto& first = geometry.segments().front();
    const auto& last = geometry.segments().back();
    ASSERT_FALSE(first.curve.empty());
    ASSERT_FALSE(last.curve.empty());

    const Vec3 first_start = first.curve.start();
    const Vec3 last_end = last.curve.end();
    expect_finite_vec3(first_start, "first segment start");
    expect_finite_vec3(last_end, "last segment end");

    if (geometry.segments().size() >= 2) {
        const auto& second = geometry.segments()[1];
        const auto& penultimate = geometry.segments()[geometry.segments().size() - 2];
        ASSERT_FALSE(second.curve.empty());
        ASSERT_FALSE(penultimate.curve.empty());

        const Vec3 first_boundary_end = first.curve.end();
        const Vec3 second_boundary_start = second.curve.start();
        const Vec3 penultimate_boundary_end = penultimate.curve.end();
        const Vec3 last_boundary_start = last.curve.start();

        EXPECT_LT((first_boundary_end - second_boundary_start).length(), 1e-3f)
            << "first boundary is not C0 continuous";
        EXPECT_LT((penultimate_boundary_end - last_boundary_start).length(), 1e-3f)
            << "last boundary is not C0 continuous";
    }
}

}  // namespace

TEST(GeometryRefDegenerateBoundaries, EmptyPatternShouldNotCrashAndRemainFinite) {
    const YarnProperties yarn = default_yarn();
    const Gauge gauge = default_gauge();

    GeometryPath geometry;
    EXPECT_NO_THROW({
        geometry = build_geometry_for_pattern({}, yarn, gauge);
    });

    EXPECT_TRUE(geometry.segments().empty());
    expect_geometry_finite(geometry);
    EXPECT_EQ(count_zero_length_segments(geometry), 0u);
}

TEST(GeometryRefDegenerateBoundaries, CastOnOnlyShouldProduceFiniteNonDegenerateGeometry) {
    const YarnProperties yarn = default_yarn();
    const Gauge gauge = default_gauge();

    GeometryPath geometry;
    EXPECT_NO_THROW({
        geometry = build_geometry_for_pattern({"C"}, yarn, gauge);
    });

    ASSERT_FALSE(geometry.segments().empty());
    expect_geometry_finite(geometry);
    expect_boundary_continuity(geometry);
    EXPECT_EQ(count_zero_length_segments(geometry), 0u);
}

TEST(GeometryRefDegenerateBoundaries, SingleStitchPatternShouldStayFiniteAndContinuousAtBoundaries) {
    const YarnProperties yarn = default_yarn();
    const Gauge gauge = default_gauge();

    GeometryPath geometry;
    EXPECT_NO_THROW({
        geometry = build_geometry_for_pattern({"C", "K"}, yarn, gauge);
    });

    ASSERT_GE(geometry.segments().size(), 2u);
    expect_geometry_finite(geometry);
    expect_boundary_continuity(geometry);
    EXPECT_EQ(count_zero_length_segments(geometry), 0u);
}

TEST(GeometryRefDegenerateBoundaries, SingleRowTransitionsShouldKeepFirstAndLastBoundariesContinuous) {
    const YarnProperties yarn = default_yarn();
    const Gauge gauge = default_gauge();

    GeometryPath geometry;
    EXPECT_NO_THROW({
        geometry = build_geometry_for_pattern({"CCC", "KPK"}, yarn, gauge);
    });

    ASSERT_FALSE(geometry.segments().empty());
    expect_geometry_finite(geometry);
    expect_boundary_continuity(geometry);
    EXPECT_EQ(count_zero_length_segments(geometry), 0u);
}

TEST(GeometryRefDegenerateBoundaries, RepetitiveSingleColumnPatternShouldAvoidZeroLengthBezierCreation) {
    const YarnProperties yarn = default_yarn();
    const Gauge gauge = default_gauge();

    GeometryPath geometry;
    EXPECT_NO_THROW({
        geometry = build_geometry_for_pattern({"C", "K", "K", "K", "K", "K", "K"}, yarn, gauge);
    });

    ASSERT_FALSE(geometry.segments().empty());
    expect_geometry_finite(geometry);
    expect_boundary_continuity(geometry);

    const size_t degenerate_segs = count_zero_length_segments(geometry);
    EXPECT_EQ(degenerate_segs, 0u) << "Expected no zero-length segments";
}
