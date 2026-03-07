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

// -----------------------------------------------------------------------------
// Research basis for expected behavior in this file
// -----------------------------------------------------------------------------
// These tests encode target geometric behavior from knitting references:
// - Knit stitches visually project as "V" columns on the knit side.
// - Purl stitches recede on that side and appear as horizontal bumps.
// - Stockinette knit/purl rows are near-mirror front/back forms.
// - Slip stitches are elongated versus knitted stitches.
// - Yarn-over increases create eyelets/opening and consume extra yarn.
// - Left/right decreases should produce visible opposite lean.
//
// Sources used while planning this suite:
// - https://en.wikipedia.org/wiki/Knitting (courses/wales, knit vs purl behavior)
// - https://en.wikipedia.org/wiki/Basic_knitted_fabrics (stockinette and reverse)
//
// NOTE:
// These are intentionally strict and expected to fail with the current geometry
// builder. They describe desired outcomes for a future implementation.

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

static std::vector<Vec3> sample_segment_points(const SegmentGeometry& seg, int samples_per_bezier = 20) {
    std::vector<Vec3> points;
    for (const auto& bez : seg.curve.segments()) {
        for (int i = 0; i <= samples_per_bezier; ++i) {
            float t = static_cast<float>(i) / static_cast<float>(samples_per_bezier);
            points.push_back(bez.evaluate(t));
        }
    }
    return points;
}

static float segment_mean_z(const SegmentGeometry& seg) {
    auto points = sample_segment_points(seg, 30);
    if (points.empty()) return 0.0f;

    float sum = 0.0f;
    for (const auto& p : points) sum += p.z;
    return sum / static_cast<float>(points.size());
}

static float segment_y_span(const SegmentGeometry& seg) {
    auto points = sample_segment_points(seg, 30);
    if (points.empty()) return 0.0f;

    float min_y = std::numeric_limits<float>::max();
    float max_y = std::numeric_limits<float>::lowest();
    for (const auto& p : points) {
        min_y = std::min(min_y, p.y);
        max_y = std::max(max_y, p.y);
    }
    return max_y - min_y;
}

static float segment_apex_x(const SegmentGeometry& seg) {
    auto points = sample_segment_points(seg, 30);
    if (points.empty()) return 0.0f;

    float max_y = std::numeric_limits<float>::lowest();
    float x_at_max_y = 0.0f;
    for (const auto& p : points) {
        if (p.y > max_y) {
            max_y = p.y;
            x_at_max_y = p.x;
        }
    }
    return x_at_max_y;
}

static float mean_z_for_segment_range(const GeometryPath& geometry, size_t begin, size_t end_inclusive) {
    float sum = 0.0f;
    size_t count = 0;
    for (size_t i = begin; i <= end_inclusive; ++i) {
        const SegmentGeometry* seg = geometry.get_segment(static_cast<SegmentId>(i));
        if (!seg) continue;
        sum += segment_mean_z(*seg);
        ++count;
    }
    return (count > 0) ? sum / static_cast<float>(count) : 0.0f;
}

TEST(GeometryReferenceBehavior, GarterStitchRowsShouldBeNearMirrorInDepth) {
    YarnProperties yarn = default_yarn();
    Gauge gauge = default_gauge();

    // 4 cast-on, then three knit rows (garter stitch in flat knitting)
    // Row 1 (WS): K → Back orientation (negative Z)
    // Row 2 (RS): K → Front orientation (positive Z)
    // Row 3 (WS): K → Back orientation (negative Z)
    // Segment ranges:
    // [0..3] cast-on, [4..7] knit(WS), [8..11] knit(RS), [12..15] knit(WS)
    GeometryPath geometry = build_geometry_for_pattern(
        {"CCCC", "KKKK", "KKKK", "KKKK"},
        yarn,
        gauge);

    const float knit_row_depth = mean_z_for_segment_range(geometry, 4, 7);
    const float purl_row_depth = mean_z_for_segment_range(geometry, 8, 11);

    // Target: garter stitch WS-knit and RS-knit rows have opposite Z depth.
    // Their sum should be near zero (near-mirror).
    EXPECT_LT(std::abs(knit_row_depth + purl_row_depth), 0.05f * yarn.compressed_radius);
}

TEST(GeometryReferenceBehavior, DecreasesShouldShowStrongOppositeLean) {
    YarnProperties yarn = default_yarn();
    Gauge gauge = default_gauge();

    // Cast-on 6, then K, K2tog, SSK, K
    // Segment IDs expected: cast-on [0..5], row1 [6..9]
    // row1: 6=K, 7=K2tog (right-lean), 8=SSK (left-lean), 9=K
    GeometryPath geometry = build_geometry_for_pattern(
        {"CCCCCC", "K2SK"},
        yarn,
        gauge);

    const SegmentGeometry* k2tog = geometry.get_segment(7);
    const SegmentGeometry* ssk = geometry.get_segment(8);
    ASSERT_NE(k2tog, nullptr);
    ASSERT_NE(ssk, nullptr);

    const float k2tog_apex_x = segment_apex_x(*k2tog);
    const float ssk_apex_x = segment_apex_x(*ssk);

    // Target: clear visual asymmetry between left/right decreases.
    // Intentionally strict threshold to codify desired stronger effect.
    EXPECT_GT(k2tog_apex_x - ssk_apex_x, 10.0f * yarn.compressed_radius);
}

TEST(GeometryReferenceBehavior, SlipStitchShouldBeSignificantlyTallerThanWorkedStitches) {
    YarnProperties yarn = default_yarn();
    Gauge gauge = default_gauge();

    // Cast-on 3, then K, Slip, K
    // Segment IDs expected: cast-on [0..2], row1 [3..5]
    GeometryPath geometry = build_geometry_for_pattern(
        {"CCC", "KSK"},
        yarn,
        gauge);

    const SegmentGeometry* knit_left = geometry.get_segment(3);
    const SegmentGeometry* slip = geometry.get_segment(4);
    const SegmentGeometry* knit_right = geometry.get_segment(5);
    ASSERT_NE(knit_left, nullptr);
    ASSERT_NE(slip, nullptr);
    ASSERT_NE(knit_right, nullptr);

    const float knit_mean_span = (segment_y_span(*knit_left) + segment_y_span(*knit_right)) * 0.5f;
    const float slip_span = segment_y_span(*slip);

    // Textile references: slipped stitches are elongated relative to knitted.
    // Intentionally strict and expected to fail currently.
    EXPECT_GT(slip_span, 1.8f * knit_mean_span);
}

TEST(GeometryReferenceBehavior, YarnOverShouldConsumeMoreYarnThanNeighborKnits) {
    YarnProperties yarn = default_yarn();
    Gauge gauge = default_gauge();

    // Cast-on 3, then K, YO, K
    // Segment IDs expected: cast-on [0..2], row1 [3..5], with YO at 4
    GeometryPath geometry = build_geometry_for_pattern(
        {"CCC", "KOK"},
        yarn,
        gauge);

    const SegmentGeometry* knit_left = geometry.get_segment(3);
    const SegmentGeometry* yo = geometry.get_segment(4);
    const SegmentGeometry* knit_right = geometry.get_segment(5);
    ASSERT_NE(knit_left, nullptr);
    ASSERT_NE(yo, nullptr);
    ASSERT_NE(knit_right, nullptr);

    const float knit_mean_arc = (knit_left->arc_length + knit_right->arc_length) * 0.5f;

    // Target: YO creates an eyelet with extra yarn path length.
    // Intentionally strict and expected to fail with current generation.
    EXPECT_GT(yo->arc_length, 1.25f * knit_mean_arc);
}

TEST(GeometryReferenceBehavior, RibbingShouldHaveStrongFrontBackAlternationBetweenWales) {
    YarnProperties yarn = default_yarn();
    Gauge gauge = default_gauge();

    // Cast-on 6, then KPKPKP
    // Segment IDs expected: cast-on [0..5], row1 [6..11]
    GeometryPath geometry = build_geometry_for_pattern(
        {"CCCCCC", "KPKPKP"},
        yarn,
        gauge);

    const std::vector<SegmentId> knit_ids = {6, 8, 10};
    const std::vector<SegmentId> purl_ids = {7, 9, 11};

    float knit_z = 0.0f;
    for (SegmentId id : knit_ids) {
        const SegmentGeometry* seg = geometry.get_segment(id);
        ASSERT_NE(seg, nullptr);
        knit_z += segment_mean_z(*seg);
    }
    knit_z /= static_cast<float>(knit_ids.size());

    float purl_z = 0.0f;
    for (SegmentId id : purl_ids) {
        const SegmentGeometry* seg = geometry.get_segment(id);
        ASSERT_NE(seg, nullptr);
        purl_z += segment_mean_z(*seg);
    }
    purl_z /= static_cast<float>(purl_ids.size());

    // Target: pronounced knit-forward / purl-back relief in 1x1 rib.
    // Intentionally strict and expected to fail now.
    EXPECT_GT(std::abs(knit_z - purl_z), 3.0f * yarn.compressed_radius);
}
