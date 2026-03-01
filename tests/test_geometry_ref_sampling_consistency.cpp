#include <gtest/gtest.h>

#include "geometry.hpp"
#include "surface/surface.hpp"
#include "test_helpers.hpp"
#include "yarn_path.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
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

static float polyline_arc_length(const std::vector<Vec3>& polyline) {
    float total = 0.0f;
    for (size_t i = 1; i < polyline.size(); ++i) {
        total += (polyline[i] - polyline[i - 1]).length();
    }
    return total;
}

static std::pair<Vec3, Vec3> polyline_bbox(const std::vector<Vec3>& polyline) {
    if (polyline.empty()) {
        return {Vec3::zero(), Vec3::zero()};
    }

    Vec3 min_pt(std::numeric_limits<float>::max(),
                std::numeric_limits<float>::max(),
                std::numeric_limits<float>::max());
    Vec3 max_pt(std::numeric_limits<float>::lowest(),
                std::numeric_limits<float>::lowest(),
                std::numeric_limits<float>::lowest());

    for (const auto& p : polyline) {
        min_pt.x = std::min(min_pt.x, p.x);
        min_pt.y = std::min(min_pt.y, p.y);
        min_pt.z = std::min(min_pt.z, p.z);

        max_pt.x = std::max(max_pt.x, p.x);
        max_pt.y = std::max(max_pt.y, p.y);
        max_pt.z = std::max(max_pt.z, p.z);
    }

    return {min_pt, max_pt};
}

static std::vector<Vec3> resample_by_arclength(const std::vector<Vec3>& polyline,
                                               size_t sample_count) {
    if (polyline.empty() || sample_count == 0) {
        return {};
    }

    if (sample_count == 1 || polyline.size() == 1) {
        return {polyline.front()};
    }

    std::vector<float> cumulative(polyline.size(), 0.0f);
    for (size_t i = 1; i < polyline.size(); ++i) {
        cumulative[i] = cumulative[i - 1] + (polyline[i] - polyline[i - 1]).length();
    }

    const float total_len = cumulative.back();
    if (total_len <= 1e-8f) {
        return std::vector<Vec3>(sample_count, polyline.front());
    }

    std::vector<Vec3> out;
    out.reserve(sample_count);

    size_t seg_idx = 0;
    for (size_t k = 0; k < sample_count; ++k) {
        const float target = (total_len * static_cast<float>(k)) /
                             static_cast<float>(sample_count - 1);

        while (seg_idx + 1 < cumulative.size() && cumulative[seg_idx + 1] < target) {
            ++seg_idx;
        }

        if (seg_idx + 1 >= polyline.size()) {
            out.push_back(polyline.back());
            continue;
        }

        const float seg_start = cumulative[seg_idx];
        const float seg_end = cumulative[seg_idx + 1];
        const float seg_len = seg_end - seg_start;

        if (seg_len <= 1e-8f) {
            out.push_back(polyline[seg_idx]);
            continue;
        }

        const float t = (target - seg_start) / seg_len;
        out.push_back(polyline[seg_idx] * (1.0f - t) + polyline[seg_idx + 1] * t);
    }

    return out;
}

static std::vector<float> local_turn_angles(const std::vector<Vec3>& polyline) {
    if (polyline.size() < 3) {
        return {};
    }

    std::vector<float> turns;
    turns.reserve(polyline.size() - 2);

    for (size_t i = 1; i + 1 < polyline.size(); ++i) {
        const Vec3 a = polyline[i] - polyline[i - 1];
        const Vec3 b = polyline[i + 1] - polyline[i];

        const float la = a.length();
        const float lb = b.length();
        if (la <= 1e-8f || lb <= 1e-8f) {
            turns.push_back(0.0f);
            continue;
        }

        const float cos_theta = std::clamp(a.dot(b) / (la * lb), -1.0f, 1.0f);
        turns.push_back(std::acos(cos_theta));
    }

    return turns;
}

}  // namespace

TEST(GeometryRefSamplingConsistency, FixedAndArcLengthSamplingShouldAgreeOnEndpointsAndArcLengthEstimate) {
    const YarnProperties yarn = default_yarn();
    const Gauge gauge = default_gauge();

    const GeometryPath geometry = build_geometry_for_pattern(
        {"CCCCCCCC", "KKKKKKKK", "PPPPPPPP", "KKKKKKKK"}, yarn, gauge);

    const auto poly_fixed = geometry.to_polyline_fixed(20);
    ASSERT_GT(poly_fixed.size(), 8u);

    const float nominal_step = geometry.total_arc_length() /
                               static_cast<float>(std::max<size_t>(1, poly_fixed.size() - 1));
    const auto poly_arc = geometry.to_polyline(nominal_step);
    ASSERT_GT(poly_arc.size(), 8u);

    const float start_error = (poly_fixed.front() - poly_arc.front()).length();
    const float end_error = (poly_fixed.back() - poly_arc.back()).length();
    EXPECT_LT(start_error, 1e-5f);
    EXPECT_LT(end_error, 1e-5f);

    const float len_fixed = polyline_arc_length(poly_fixed);
    const float len_arc = polyline_arc_length(poly_arc);
    const float len_ref = std::max(1e-6f, geometry.total_arc_length());

    EXPECT_LT(std::abs(len_fixed - len_arc) / len_ref, 0.06f);
    EXPECT_LT(std::abs(len_fixed - len_ref) / len_ref, 0.10f);
    EXPECT_LT(std::abs(len_arc - len_ref) / len_ref, 0.10f);
}

TEST(GeometryRefSamplingConsistency, FixedAndArcLengthSamplingShouldHaveConsistentPolylineBoundingBox) {
    const YarnProperties yarn = default_yarn();
    const Gauge gauge = default_gauge();

    const GeometryPath geometry = build_geometry_for_pattern(
        {"CCCCCC", "KO2SKK", "PPPPPP", "KKKKKK"}, yarn, gauge);

    const auto poly_fixed = geometry.to_polyline_fixed(24);
    ASSERT_GT(poly_fixed.size(), 8u);

    const float nominal_step = geometry.total_arc_length() /
                               static_cast<float>(std::max<size_t>(1, poly_fixed.size() - 1));
    const auto poly_arc = geometry.to_polyline(nominal_step);
    ASSERT_GT(poly_arc.size(), 8u);

    const auto [fixed_min, fixed_max] = polyline_bbox(poly_fixed);
    const auto [arc_min, arc_max] = polyline_bbox(poly_arc);

    const float tol = 0.85f * yarn.compressed_radius;
    EXPECT_NEAR(fixed_min.x, arc_min.x, tol);
    EXPECT_NEAR(fixed_min.y, arc_min.y, tol);
    EXPECT_NEAR(fixed_min.z, arc_min.z, tol);
    EXPECT_NEAR(fixed_max.x, arc_max.x, tol);
    EXPECT_NEAR(fixed_max.y, arc_max.y, tol);
    EXPECT_NEAR(fixed_max.z, arc_max.z, tol);

    const auto [geom_min, geom_max] = geometry.bounding_box();
    EXPECT_GE(fixed_min.x, geom_min.x - tol);
    EXPECT_GE(fixed_min.y, geom_min.y - tol);
    EXPECT_GE(fixed_min.z, geom_min.z - tol);
    EXPECT_LE(fixed_max.x, geom_max.x + tol);
    EXPECT_LE(fixed_max.y, geom_max.y + tol);
    EXPECT_LE(fixed_max.z, geom_max.z + tol);
}

TEST(GeometryRefSamplingConsistency, FixedAndArcLengthSamplingShouldPreserveStrictLocalTurnProfile) {
    const YarnProperties yarn = default_yarn();
    const Gauge gauge = default_gauge();

    const GeometryPath geometry = build_geometry_for_pattern(
        {"CCCCCCCC", "KPKPKPKP", "PKPKPKPK", "KKKKKKKK"}, yarn, gauge);

    const auto poly_fixed = geometry.to_polyline_fixed(28);
    ASSERT_GT(poly_fixed.size(), 16u);

    const float nominal_step = geometry.total_arc_length() /
                               static_cast<float>(std::max<size_t>(1, poly_fixed.size() - 1));
    const auto poly_arc = geometry.to_polyline(nominal_step);
    ASSERT_GT(poly_arc.size(), 16u);

    const size_t kResampledCount = 180;
    const auto fixed_rs = resample_by_arclength(poly_fixed, kResampledCount);
    const auto arc_rs = resample_by_arclength(poly_arc, kResampledCount);

    const auto fixed_turns = local_turn_angles(fixed_rs);
    const auto arc_turns = local_turn_angles(arc_rs);

    ASSERT_EQ(fixed_turns.size(), arc_turns.size());
    ASSERT_GT(fixed_turns.size(), 8u);

    float max_turn_diff = 0.0f;
    float mean_turn_diff = 0.0f;

    for (size_t i = 0; i < fixed_turns.size(); ++i) {
        const float d = std::abs(fixed_turns[i] - arc_turns[i]);
        max_turn_diff = std::max(max_turn_diff, d);
        mean_turn_diff += d;
    }
    mean_turn_diff /= static_cast<float>(fixed_turns.size());

    // Strict local-turn consistency target for reference-level comparison.
    EXPECT_LT(max_turn_diff, 0.20f);
    EXPECT_LT(mean_turn_diff, 0.06f);
}
