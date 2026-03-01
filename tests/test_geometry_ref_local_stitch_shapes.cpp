#include <gtest/gtest.h>
#include "geometry.hpp"
#include "yarn_path.hpp"
#include "surface/surface.hpp"
#include "test_helpers.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <limits>
#include <stdexcept>
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

static std::vector<StitchInstruction> parse_compact_row(const std::string& row) {
    std::vector<StitchInstruction> stitches;

    for (size_t i = 0; i < row.size();) {
        const char c = row[i];
        if (std::isspace(static_cast<unsigned char>(c)) || c == ',' || c == '_') {
            ++i;
            continue;
        }

        // Multi-character tokens first:
        // SSK = left-lean decrease
        // YO  = yarn-over
        // SL  = slip stitch
        if (i + 3 <= row.size() && row.compare(i, 3, "SSK") == 0) {
            stitches.push_back(instruction::SSK{});
            i += 3;
            continue;
        }
        if (i + 2 <= row.size() && row.compare(i, 2, "YO") == 0) {
            stitches.push_back(instruction::YarnOver{});
            i += 2;
            continue;
        }
        if (i + 2 <= row.size() && row.compare(i, 2, "SL") == 0) {
            stitches.push_back(instruction::Slip{});
            i += 2;
            continue;
        }

        // Single-character compact tokens:
        // C=CastOn, K=Knit, P=Purl, O=YarnOver, 2=K2tog, S=SSK, L=Slip
        switch (c) {
            case 'C': stitches.push_back(instruction::CastOn{1}); break;
            case 'K': stitches.push_back(instruction::Knit{}); break;
            case 'P': stitches.push_back(instruction::Purl{}); break;
            case 'O': stitches.push_back(instruction::YarnOver{}); break;
            case '2': stitches.push_back(instruction::K2tog{}); break;
            case 'S': stitches.push_back(instruction::SSK{}); break;
            case 'L': stitches.push_back(instruction::Slip{}); break;
            default:
                throw std::invalid_argument("Unsupported compact stitch token in row: " + row);
        }
        ++i;
    }

    return stitches;
}

static PatternInstructions create_compact_pattern(const std::vector<std::string>& rows) {
    PatternInstructions pattern;
    pattern.rows.reserve(rows.size());

    for (size_t i = 0; i < rows.size(); ++i) {
        RowInstruction row;
        row.side = (i % 2 == 0) ? RowSide::RS : RowSide::WS;
        row.stitches = parse_compact_row(rows[i]);
        pattern.rows.push_back(std::move(row));
    }

    return pattern;
}

static GeometryPath build_geometry_from_compact_pattern(const std::vector<std::string>& rows,
                                                        const YarnProperties& yarn,
                                                        const Gauge& gauge) {
    PatternInstructions pattern = create_compact_pattern(rows);
    StitchGraph graph = StitchGraph::from_instructions(pattern);
    YarnPath yarn_path = YarnPath::from_stitch_graph(graph, yarn, gauge);
    SurfaceGraph surface = build_test_surface(yarn_path, yarn, gauge);
    return GeometryPath::from_yarn_path(yarn_path, surface, yarn, gauge);
}

static std::vector<Vec3> sample_segment_points(const SegmentGeometry& seg, int samples_per_bezier = 30) {
    std::vector<Vec3> points;
    for (const auto& bez : seg.curve.segments()) {
        for (int i = 0; i <= samples_per_bezier; ++i) {
            const float t = static_cast<float>(i) / static_cast<float>(samples_per_bezier);
            points.push_back(bez.evaluate(t));
        }
    }
    return points;
}

static float segment_mean_z(const SegmentGeometry& seg) {
    const auto points = sample_segment_points(seg, 40);
    if (points.empty()) return 0.0f;

    float sum = 0.0f;
    for (const auto& p : points) sum += p.z;
    return sum / static_cast<float>(points.size());
}

static float segment_y_span(const SegmentGeometry& seg) {
    const auto points = sample_segment_points(seg, 40);
    if (points.empty()) return 0.0f;

    float min_y = std::numeric_limits<float>::max();
    float max_y = std::numeric_limits<float>::lowest();
    for (const auto& p : points) {
        min_y = std::min(min_y, p.y);
        max_y = std::max(max_y, p.y);
    }
    return max_y - min_y;
}

static float segment_x_span(const SegmentGeometry& seg) {
    const auto points = sample_segment_points(seg, 40);
    if (points.empty()) return 0.0f;

    float min_x = std::numeric_limits<float>::max();
    float max_x = std::numeric_limits<float>::lowest();
    for (const auto& p : points) {
        min_x = std::min(min_x, p.x);
        max_x = std::max(max_x, p.x);
    }
    return max_x - min_x;
}

static float segment_apex_x(const SegmentGeometry& seg) {
    const auto points = sample_segment_points(seg, 40);
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

}  // namespace

TEST(GeometryRefLocalStitchShapes, KnitVsPurlShouldHaveOppositeDepthSignAndStrongRelativeDepth) {
    const YarnProperties yarn = default_yarn();
    const Gauge gauge = default_gauge();

    // Segment IDs: cast-on [0..2], row1 [3..5] => K, P, K
    const GeometryPath geometry = build_geometry_from_compact_pattern({"CCC", "KPK"}, yarn, gauge);

    const SegmentGeometry* knit_left = geometry.get_segment(3);
    const SegmentGeometry* purl = geometry.get_segment(4);
    const SegmentGeometry* knit_right = geometry.get_segment(5);
    ASSERT_NE(knit_left, nullptr);
    ASSERT_NE(purl, nullptr);
    ASSERT_NE(knit_right, nullptr);

    const float knit_mean_depth = 0.5f * (segment_mean_z(*knit_left) + segment_mean_z(*knit_right));
    const float purl_depth = segment_mean_z(*purl);

    // Reference-level invariant (strict): knit and purl should project to opposite sides.
    EXPECT_LT(knit_mean_depth * purl_depth, -1.0f * yarn.compressed_radius * yarn.compressed_radius);
    EXPECT_GT(std::abs(knit_mean_depth - purl_depth), 3.0f * yarn.compressed_radius);
}

TEST(GeometryRefLocalStitchShapes, K2togAndSSKShouldLeanInOppositeDirections) {
    const YarnProperties yarn = default_yarn();
    const Gauge gauge = default_gauge();

    // Segment IDs: cast-on [0..5], row1 [6..9] => K, K2tog, SSK, K
    const GeometryPath geometry = build_geometry_from_compact_pattern({"CCCCCC", "K2SK"}, yarn, gauge);

    const SegmentGeometry* k2tog = geometry.get_segment(7);
    const SegmentGeometry* ssk = geometry.get_segment(8);
    ASSERT_NE(k2tog, nullptr);
    ASSERT_NE(ssk, nullptr);

    const float k2tog_apex_x = segment_apex_x(*k2tog);
    const float ssk_apex_x = segment_apex_x(*ssk);

    // Reference-level invariant (strict): right-lean K2tog must shift apex right of SSK.
    EXPECT_GT(k2tog_apex_x - ssk_apex_x, 8.0f * yarn.compressed_radius);
}

TEST(GeometryRefLocalStitchShapes, YarnOverShouldOpenAndConsumeExtraArcLength) {
    const YarnProperties yarn = default_yarn();
    const Gauge gauge = default_gauge();

    // Segment IDs: cast-on [0..2], row1 [3..5] => K, YO, K
    const GeometryPath geometry = build_geometry_from_compact_pattern({"CCC", "KOK"}, yarn, gauge);

    const SegmentGeometry* knit_left = geometry.get_segment(3);
    const SegmentGeometry* yo = geometry.get_segment(4);
    const SegmentGeometry* knit_right = geometry.get_segment(5);
    ASSERT_NE(knit_left, nullptr);
    ASSERT_NE(yo, nullptr);
    ASSERT_NE(knit_right, nullptr);

    const float knit_mean_arc = 0.5f * (knit_left->arc_length + knit_right->arc_length);
    const float knit_mean_x_span = 0.5f * (segment_x_span(*knit_left) + segment_x_span(*knit_right));

    // Reference-level invariants (strict): YO should both open wider and consume more yarn.
    EXPECT_GT(yo->arc_length, 1.35f * knit_mean_arc);
    EXPECT_GT(segment_x_span(*yo), 1.30f * knit_mean_x_span);
}

TEST(GeometryRefLocalStitchShapes, SlipStitchShouldBeElongatedVersusNeighborKnits) {
    const YarnProperties yarn = default_yarn();
    const Gauge gauge = default_gauge();

    // Segment IDs: cast-on [0..2], row1 [3..5] => K, SL, K
    // Row2 stabilizes downstream pull and should amplify elongation if modeled.
    const GeometryPath geometry = build_geometry_from_compact_pattern({"CCC", "KLK", "KKK"}, yarn, gauge);

    const SegmentGeometry* knit_left = geometry.get_segment(3);
    const SegmentGeometry* slip = geometry.get_segment(4);
    const SegmentGeometry* knit_right = geometry.get_segment(5);
    ASSERT_NE(knit_left, nullptr);
    ASSERT_NE(slip, nullptr);
    ASSERT_NE(knit_right, nullptr);

    const float knit_mean_span = 0.5f * (segment_y_span(*knit_left) + segment_y_span(*knit_right));
    const float knit_mean_arc = 0.5f * (knit_left->arc_length + knit_right->arc_length);

    // Reference-level invariants (strict): slipped stitch should be visibly elongated.
    EXPECT_GT(segment_y_span(*slip), 1.60f * knit_mean_span);
    EXPECT_GT(slip->arc_length, 1.25f * knit_mean_arc);
}
