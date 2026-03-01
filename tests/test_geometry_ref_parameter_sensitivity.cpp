#include <gtest/gtest.h>

#include "geometry.hpp"
#include "surface/surface.hpp"
#include "test_helpers.hpp"
#include "yarn_path.hpp"

#include <algorithm>
#include <array>
#include <limits>
#include <string>
#include <vector>

using namespace yarnpath;
using namespace yarnpath::test;

namespace {

struct GeometryMetrics {
    float span_x = 0.0f;
    float span_y = 0.0f;
    float span_z = 0.0f;
    float total_arc_length = 0.0f;
    float min_nonlocal_distance = 0.0f;
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

static GeometryMetrics compute_metrics(const GeometryPath& geometry) {
    GeometryMetrics m;

    const auto [bb_min, bb_max] = geometry.bounding_box();
    m.span_x = bb_max.x - bb_min.x;
    m.span_y = bb_max.y - bb_min.y;
    m.span_z = bb_max.z - bb_min.z;
    m.total_arc_length = geometry.total_arc_length();

    const auto polyline = geometry.to_polyline_fixed(24);
    m.min_nonlocal_distance = min_nonlocal_polyline_distance(polyline, 24);

    return m;
}

static const std::vector<std::string>& reference_pattern_rows() {
    static const std::vector<std::string> kRows = {
        "CCCCCCCC",
        "KKKKKKKK",
        "PPPPPPPP",
        "KKKKKKKK"
    };
    return kRows;
}

}  // namespace

TEST(GeometryRefParameterSensitivity, NeedleDiameterSweepShouldMonotonicallyIncreaseBBoxAndArcLength) {
    YarnProperties yarn = default_yarn();
    Gauge gauge = default_gauge();

    const std::array<float, 3> needle_diameters = {3.0f, 5.0f, 7.0f};
    std::vector<GeometryMetrics> metrics;

    for (float d : needle_diameters) {
        gauge.needle_diameter = d;
        const GeometryPath geometry = build_geometry_for_pattern(reference_pattern_rows(), yarn, gauge);
        ASSERT_FALSE(geometry.segments().empty());
        metrics.push_back(compute_metrics(geometry));
    }

    for (size_t i = 1; i < metrics.size(); ++i) {
        SCOPED_TRACE(i);
        EXPECT_GT(metrics[i].span_x, metrics[i - 1].span_x);
        EXPECT_GT(metrics[i].span_y, metrics[i - 1].span_y);
        EXPECT_GT(metrics[i].total_arc_length, metrics[i - 1].total_arc_length);
    }
}

TEST(GeometryRefParameterSensitivity, CompressedRadiusSweepShouldMonotonicallyIncreaseBBox) {
    YarnProperties yarn = default_yarn();
    Gauge gauge = default_gauge();

    const std::array<float, 3> compressed_radii = {0.30f, 0.50f, 0.70f};
    std::vector<GeometryMetrics> metrics;

    for (float r : compressed_radii) {
        yarn.compressed_radius = r;
        yarn.relaxed_radius = std::max(yarn.relaxed_radius, 2.0f * r);
        const GeometryPath geometry = build_geometry_for_pattern(reference_pattern_rows(), yarn, gauge);
        ASSERT_FALSE(geometry.segments().empty());
        metrics.push_back(compute_metrics(geometry));
    }

    for (size_t i = 1; i < metrics.size(); ++i) {
        SCOPED_TRACE(i);
        EXPECT_GT(metrics[i].span_x, metrics[i - 1].span_x);
        EXPECT_GT(metrics[i].span_y, metrics[i - 1].span_y);
        EXPECT_GT(metrics[i].total_arc_length, metrics[i - 1].total_arc_length);
    }
}

TEST(GeometryRefParameterSensitivity, CompressedRadiusSweepShouldMonotonicallyIncreaseGlobalClearanceProxy) {
    YarnProperties yarn = default_yarn();
    Gauge gauge = default_gauge();

    const std::array<float, 3> compressed_radii = {0.30f, 0.50f, 0.70f};
    std::vector<GeometryMetrics> metrics;

    for (float r : compressed_radii) {
        yarn.compressed_radius = r;
        yarn.relaxed_radius = std::max(yarn.relaxed_radius, 2.0f * r);
        const GeometryPath geometry = build_geometry_for_pattern(reference_pattern_rows(), yarn, gauge);
        ASSERT_FALSE(geometry.segments().empty());
        metrics.push_back(compute_metrics(geometry));
    }

    for (size_t i = 1; i < metrics.size(); ++i) {
        SCOPED_TRACE(i);
        EXPECT_GT(metrics[i].min_nonlocal_distance, metrics[i - 1].min_nonlocal_distance);
    }
}

TEST(GeometryRefParameterSensitivity, TensionSweepShouldMonotonicallyTightenGeometryArcLengthAndWidth) {
    YarnProperties yarn = default_yarn();
    Gauge gauge = default_gauge();

    const std::array<float, 3> tensions = {0.20f, 0.50f, 0.80f};
    std::vector<GeometryMetrics> metrics;

    for (float t : tensions) {
        yarn.tension = t;
        const GeometryPath geometry = build_geometry_for_pattern(reference_pattern_rows(), yarn, gauge);
        ASSERT_FALSE(geometry.segments().empty());
        metrics.push_back(compute_metrics(geometry));
    }

    for (size_t i = 1; i < metrics.size(); ++i) {
        SCOPED_TRACE(i);
        EXPECT_LT(metrics[i].total_arc_length, metrics[i - 1].total_arc_length);
        EXPECT_LT(metrics[i].span_x, metrics[i - 1].span_x);
    }
}
