#include <gtest/gtest.h>
#include "geometry.hpp"
#include "yarn_path.hpp"
#include "surface/surface.hpp"
#include "test_helpers.hpp"

#include <algorithm>
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

// Computes minimum point-to-point distance in a polyline, skipping local
// neighborhoods by index to avoid trivial adjacent-sample distances.
static float min_nonlocal_polyline_distance(const std::vector<Vec3>& polyline,
                                            size_t skip_indices) {
    if (polyline.size() < 2 || skip_indices + 1 >= polyline.size()) {
        return std::numeric_limits<float>::max();
    }

    float min_dist = std::numeric_limits<float>::max();
    for (size_t i = 0; i + skip_indices + 1 < polyline.size(); ++i) {
        for (size_t j = i + skip_indices + 1; j < polyline.size(); ++j) {
            const float d = (polyline[i] - polyline[j]).length();
            min_dist = std::min(min_dist, d);
        }
    }
    return min_dist;
}

static std::vector<Vec3> sample_segment_polyline(const SegmentGeometry& segment,
                                                 int samples_per_bezier = 20) {
    std::vector<Vec3> points;
    for (const auto& bezier : segment.curve.segments()) {
        for (int i = 0; i <= samples_per_bezier; ++i) {
            const float t = static_cast<float>(i) / static_cast<float>(samples_per_bezier);
            points.push_back(bezier.evaluate(t));
        }
    }
    return points;
}

static float min_nonlocal_distance_over_segment_polylines(const GeometryPath& geometry,
                                                          int samples_per_bezier,
                                                          size_t skip_indices) {
    float min_dist = std::numeric_limits<float>::max();
    for (const auto& segment : geometry.segments()) {
        const auto polyline = sample_segment_polyline(segment, samples_per_bezier);
        const float d = min_nonlocal_polyline_distance(polyline, skip_indices);
        min_dist = std::min(min_dist, d);
    }
    return min_dist;
}

TEST(GeometryGlobalClearanceReferenceExt, DenseStockinetteShouldKeepAtLeastDiameterClearanceGlobally) {
    YarnProperties yarn = default_yarn();
    Gauge gauge = default_gauge();

    GeometryPath geometry = build_geometry_for_pattern(
        {"CCCCCCCCCC",
         "KKKKKKKKKK",
         "PPPPPPPPPP",
         "KKKKKKKKKK",
         "PPPPPPPPPP",
         "KKKKKKKKKK",
         "PPPPPPPPPP"},
        yarn,
        gauge);

    const auto global_polyline = geometry.to_polyline_fixed(24);
    ASSERT_FALSE(global_polyline.empty());

    const float min_dist = min_nonlocal_polyline_distance(global_polyline, 24);
    const float yarn_diameter = 2.0f * yarn.compressed_radius;

    // Strict reference target for global self-contact clearance.
    EXPECT_GE(min_dist, yarn_diameter);
}

TEST(GeometryGlobalClearanceReferenceExt, DenseRibbingShouldKeepStricterThanDiameterClearanceGlobally) {
    YarnProperties yarn = default_yarn();
    Gauge gauge = default_gauge();

    GeometryPath geometry = build_geometry_for_pattern(
        {"CCCCCCCCCCCC",
         "KPKPKPKPKPKP",
         "PKPKPKPKPKPK",
         "KPKPKPKPKPKP",
         "PKPKPKPKPKPK",
         "KPKPKPKPKPKP"},
        yarn,
        gauge);

    const auto global_polyline = geometry.to_polyline_fixed(24);
    ASSERT_FALSE(global_polyline.empty());

    const float min_dist = min_nonlocal_polyline_distance(global_polyline, 24);
    const float yarn_diameter = 2.0f * yarn.compressed_radius;

    // Intentionally stricter than diameter to codify future target quality.
    EXPECT_GE(min_dist, 1.10f * yarn_diameter);
}

TEST(GeometryGlobalClearanceReferenceExt, MixedDecreaseIncreaseShouldKeepAtLeastDiameterClearanceGlobally) {
    YarnProperties yarn = default_yarn();
    Gauge gauge = default_gauge();

    // Mixed shaping row: includes K2tog ('2'), SSK ('S') and yarn-over ('O').
    GeometryPath geometry = build_geometry_for_pattern(
        {"CCCCCCCC", "K2OKSKK"},
        yarn,
        gauge);

    const auto global_polyline = geometry.to_polyline_fixed(28);
    ASSERT_FALSE(global_polyline.empty());

    const float min_dist = min_nonlocal_polyline_distance(global_polyline, 20);
    const float yarn_diameter = 2.0f * yarn.compressed_radius;

    EXPECT_GE(min_dist, yarn_diameter);
}

TEST(GeometryGlobalClearanceReferenceExt, MixedShapingShouldPassBothPerSegmentAndGlobalClearanceScans) {
    YarnProperties yarn = default_yarn();
    Gauge gauge = default_gauge();

    GeometryPath geometry = build_geometry_for_pattern(
        {"CCCCCCCC", "K2OKSKK", "KPKPKPK"},
        yarn,
        gauge);

    ASSERT_FALSE(geometry.segments().empty());

    const auto global_polyline = geometry.to_polyline_fixed(28);
    ASSERT_FALSE(global_polyline.empty());

    const float global_min_dist = min_nonlocal_polyline_distance(global_polyline, 20);
    const float per_segment_min_dist =
        min_nonlocal_distance_over_segment_polylines(geometry, 24, 8);

    const float yarn_diameter = 2.0f * yarn.compressed_radius;

    // This test explicitly checks both scan modes (whole-path and per-segment).
    // Thresholds are strict and may fail with current geometry generation.
    EXPECT_GE(global_min_dist, 1.05f * yarn_diameter);
    EXPECT_GE(per_segment_min_dist, yarn_diameter);
}
