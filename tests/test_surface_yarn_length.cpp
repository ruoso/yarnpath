#include <gtest/gtest.h>
#include "yarn_path.hpp"
#include "stitch_instruction.hpp"
#include "row_instruction.hpp"
#include "surface/surface_builder.hpp"
#include "test_helpers.hpp"
#include <cmath>

using namespace yarnpath;
using namespace yarnpath::test;

// ============================================
// YarnPath Tests - Verify yarn length calculation
// ============================================

TEST(YarnPath, YarnLengthCalculatedForKnit) {
    // Pattern: Cast on 3, Row 1: K3
    PatternInstructions pattern;

    RowInstruction row0;
    row0.side = RowSide::RS;
    row0.stitches = {instruction::CastOn{3}};
    pattern.rows.push_back(row0);

    RowInstruction row1;
    row1.side = RowSide::RS;
    row1.stitches = {instruction::Knit{}, instruction::Knit{}, instruction::Knit{}};
    pattern.rows.push_back(row1);

    auto yarn = default_yarn();
    auto gauge = default_gauge();
    auto graph = StitchGraph::from_instructions(pattern);
    auto yarn_path = YarnPath::from_stitch_graph(graph, yarn, gauge);

    const auto& segments = yarn_path.segments();

    // Two-pass cast-on: 3 foundation + 3 return = 6 segments before knit row
    // Knit stitches (segments 6-8) should have yarn length = π * loop_height + stitch_width
    float loop_height = gauge.loop_height(yarn.compressed_radius);
    float stitch_width = gauge.stitch_width(yarn.compressed_radius);
    float expected_knit_length = M_PI * loop_height + stitch_width;

    for (size_t i = 6; i < 9; ++i) {
        EXPECT_NEAR(segments[i].target_yarn_length, expected_knit_length, expected_knit_length * 0.01f)
            << "Knit segment " << i << " should have base yarn length";
    }
}

TEST(YarnPath, YarnLengthCalculatedForPurl) {
    // Pattern: Cast on 3, Row 1 (RS): P3
    // RS Purls create back-facing loops which use 12% less yarn
    PatternInstructions pattern;

    RowInstruction row0;
    row0.side = RowSide::RS;
    row0.stitches = {instruction::CastOn{3}};
    pattern.rows.push_back(row0);

    RowInstruction row1;
    row1.side = RowSide::RS;
    row1.stitches = {instruction::Purl{}, instruction::Purl{}, instruction::Purl{}};
    pattern.rows.push_back(row1);

    auto yarn = default_yarn();
    auto gauge = default_gauge();
    auto graph = StitchGraph::from_instructions(pattern);
    auto yarn_path = YarnPath::from_stitch_graph(graph, yarn, gauge);

    const auto& segments = yarn_path.segments();

    // Two-pass cast-on: 3 foundation + 3 return = 6 segments before purl row
    // Purl stitches (segments 6-8) should have yarn length = 0.88 * (π * loop_height + stitch_width)
    float loop_height = gauge.loop_height(yarn.compressed_radius);
    float stitch_width = gauge.stitch_width(yarn.compressed_radius);
    float base_length = M_PI * loop_height + stitch_width;
    float expected_purl_length = 0.88f * base_length;

    for (size_t i = 6; i < 9; ++i) {
        EXPECT_NEAR(segments[i].target_yarn_length, expected_purl_length, expected_purl_length * 0.01f)
            << "Purl segment " << i << " should have 88% of base yarn length";
    }
}

TEST(YarnPath, YarnLengthCalculatedForSlip) {
    // Pattern: Cast on 3, Row 1: K1, Slip, K1
    PatternInstructions pattern;

    RowInstruction row0;
    row0.side = RowSide::RS;
    row0.stitches = {instruction::CastOn{3}};
    pattern.rows.push_back(row0);

    RowInstruction row1;
    row1.side = RowSide::RS;
    row1.stitches = {instruction::Knit{}, instruction::Slip{}, instruction::Knit{}};
    pattern.rows.push_back(row1);

    auto yarn = default_yarn();
    auto gauge = default_gauge();
    auto graph = StitchGraph::from_instructions(pattern);
    auto yarn_path = YarnPath::from_stitch_graph(graph, yarn, gauge);

    const auto& segments = yarn_path.segments();

    // Two-pass cast-on: 3 foundation + 3 return = 6 segments before user row
    // Slip stitch (segment 7) should have yarn length = 0.5 * stitch_width
    float stitch_width = gauge.stitch_width(yarn.compressed_radius);
    float expected_slip_length = 0.5f * stitch_width;

    EXPECT_NEAR(segments[7].target_yarn_length, expected_slip_length, expected_slip_length * 0.01f)
        << "Slip stitch should have minimal yarn length (0.5 * stitch_width)";
}

TEST(YarnPath, YarnLengthCalculatedForDecreases) {
    // Pattern: Cast on 4, Row 1: K2tog, SSK
    PatternInstructions pattern;

    RowInstruction row0;
    row0.side = RowSide::RS;
    row0.stitches = {instruction::CastOn{4}};
    pattern.rows.push_back(row0);

    RowInstruction row1;
    row1.side = RowSide::RS;
    row1.stitches = {instruction::K2tog{}, instruction::SSK{}};
    pattern.rows.push_back(row1);

    auto yarn = default_yarn();
    auto gauge = default_gauge();
    auto graph = StitchGraph::from_instructions(pattern);
    auto yarn_path = YarnPath::from_stitch_graph(graph, yarn, gauge);

    const auto& segments = yarn_path.segments();

    // Two-pass cast-on: 4 foundation + 4 return = 8 segments before user row
    // K2tog (segment 8) should have 0.82 * base length
    // SSK (segment 9) should have 0.86 * base length
    float loop_height = gauge.loop_height(yarn.compressed_radius);
    float stitch_width = gauge.stitch_width(yarn.compressed_radius);
    float base_length = M_PI * loop_height + stitch_width;

    float expected_k2tog = 0.82f * base_length;
    float expected_ssk = 0.86f * base_length;

    EXPECT_NEAR(segments[8].target_yarn_length, expected_k2tog, expected_k2tog * 0.01f)
        << "K2tog should use 82% of base yarn length";

    EXPECT_NEAR(segments[9].target_yarn_length, expected_ssk, expected_ssk * 0.01f)
        << "SSK should use 86% of base yarn length";

    // SSK should use slightly more yarn than K2tog
    EXPECT_GT(segments[9].target_yarn_length, segments[8].target_yarn_length)
        << "SSK should use more yarn than K2tog (looser decrease)";
}

// ============================================
// Surface Tests - Verify usage of yarn length
// ============================================

TEST(SurfaceYarnLength, KnitVsPurlMass) {
    // Pattern: Cast on 2, Row 1: K2, Row 2: P2
    PatternInstructions pattern;

    RowInstruction row0;
    row0.side = RowSide::RS;
    row0.stitches = {instruction::CastOn{2}};
    pattern.rows.push_back(row0);

    RowInstruction row1;
    row1.side = RowSide::RS;
    row1.stitches = {instruction::Knit{}, instruction::Knit{}};
    pattern.rows.push_back(row1);

    RowInstruction row2;
    row2.side = RowSide::RS;
    row2.stitches = {instruction::Purl{}, instruction::Purl{}};
    pattern.rows.push_back(row2);

    auto yarn = default_yarn();
    auto gauge = default_gauge();
    auto graph = StitchGraph::from_instructions(pattern);
    auto yarn_path = YarnPath::from_stitch_graph(graph, yarn, gauge);

    SurfaceGraph surface = SurfaceBuilder::from_yarn_path(yarn_path, yarn, gauge);

    // Two-pass cast-on: 2 foundation + 2 return = 4 segments
    // Knit segments: 4-5, Purl segments: 6-7
    float knit_mass_avg = (surface.node(4).mass + surface.node(5).mass) / 2.0f;
    float purl_mass_avg = (surface.node(6).mass + surface.node(7).mass) / 2.0f;

    // RS Purl → Back orientation → 0.88 factor, should have ~88% of knit mass
    float mass_ratio = purl_mass_avg / knit_mass_avg;
    EXPECT_NEAR(mass_ratio, 0.88f, 0.02f)
        << "Purl nodes should have ~88% mass of knit nodes";
}

TEST(SurfaceYarnLength, SlipStitchMinimalMass) {
    // Pattern: Cast on 3, Row 1: K1, Slip, K1
    PatternInstructions pattern;

    RowInstruction row0;
    row0.side = RowSide::RS;
    row0.stitches = {instruction::CastOn{3}};
    pattern.rows.push_back(row0);

    RowInstruction row1;
    row1.side = RowSide::RS;
    row1.stitches = {instruction::Knit{}, instruction::Slip{}, instruction::Knit{}};
    pattern.rows.push_back(row1);

    auto yarn = default_yarn();
    auto gauge = default_gauge();
    auto graph = StitchGraph::from_instructions(pattern);
    auto yarn_path = YarnPath::from_stitch_graph(graph, yarn, gauge);

    SurfaceGraph surface = SurfaceBuilder::from_yarn_path(yarn_path, yarn, gauge);

    // Two-pass cast-on: 3 foundation + 3 return = 6 segments
    // Knit: segment 6, Slip: segment 7, Knit: segment 8
    float knit_mass = surface.node(6).mass;
    float slip_mass = surface.node(7).mass;

    // Slip should have much less mass (0.5 * stitch_width vs full loop)
    EXPECT_LT(slip_mass, knit_mass * 0.2f)
        << "Slip stitch should have minimal mass compared to knit";
}

TEST(SurfaceYarnLength, K2togVsSSKMass) {
    // Pattern: Cast on 4, Row 1: K2tog, SSK
    PatternInstructions pattern;

    RowInstruction row0;
    row0.side = RowSide::RS;
    row0.stitches = {instruction::CastOn{4}};
    pattern.rows.push_back(row0);

    RowInstruction row1;
    row1.side = RowSide::RS;
    row1.stitches = {instruction::K2tog{}, instruction::SSK{}};
    pattern.rows.push_back(row1);

    auto yarn = default_yarn();
    auto gauge = default_gauge();
    auto graph = StitchGraph::from_instructions(pattern);
    auto yarn_path = YarnPath::from_stitch_graph(graph, yarn, gauge);

    SurfaceGraph surface = SurfaceBuilder::from_yarn_path(yarn_path, yarn, gauge);

    // Two-pass cast-on: 4 foundation + 4 return = 8 segments
    // K2tog: segment 8, SSK: segment 9
    float k2tog_mass = surface.node(8).mass;
    float ssk_mass = surface.node(9).mass;

    // SSK should have slightly more mass than K2tog (86% vs 82%)
    EXPECT_GT(ssk_mass, k2tog_mass)
        << "SSK should have more mass than K2tog";

    float mass_ratio = ssk_mass / k2tog_mass;
    EXPECT_NEAR(mass_ratio, 0.86f / 0.82f, 0.05f)
        << "SSK/K2tog mass ratio should match yarn length ratio";
}

TEST(SurfaceYarnLength, YarnOverHasLoopOnlyMass) {
    // Pattern: Cast on 2, Row 1: K1, YO, K1
    PatternInstructions pattern;

    RowInstruction row0;
    row0.side = RowSide::RS;
    row0.stitches = {instruction::CastOn{2}};
    pattern.rows.push_back(row0);

    RowInstruction row1;
    row1.side = RowSide::RS;
    row1.stitches = {instruction::Knit{}, instruction::YarnOver{}, instruction::Knit{}};
    pattern.rows.push_back(row1);

    auto yarn = default_yarn();
    auto gauge = default_gauge();
    auto graph = StitchGraph::from_instructions(pattern);
    auto yarn_path = YarnPath::from_stitch_graph(graph, yarn, gauge);

    SurfaceGraph surface = SurfaceBuilder::from_yarn_path(yarn_path, yarn, gauge);

    // Two-pass cast-on: 2 foundation + 2 return = 4 segments
    // Knit: segment 4, YO: segment 5, Knit: segment 6
    float knit_mass = surface.node(4).mass;
    float yo_mass = surface.node(5).mass;

    // YO should have less mass than knit (just loop, no passthrough)
    EXPECT_LT(yo_mass, knit_mass)
        << "YarnOver should have less mass than knit (no passthrough component)";

    // Should be roughly π * loop_height (no stitch_width component)
    float loop_height = gauge.loop_height(yarn.compressed_radius);
    float expected_yo_mass = M_PI * loop_height * yarn.linear_density;
    EXPECT_NEAR(yo_mass, expected_yo_mass, expected_yo_mass * 0.05f)
        << "YarnOver mass should match loop-only yarn length";
}

TEST(SurfaceYarnLength, StiffnessProportionalToYarnDensity) {
    // Pattern: Cast on 2, Row 1: K2, Row 2: P2
    PatternInstructions pattern;

    RowInstruction row0;
    row0.side = RowSide::RS;
    row0.stitches = {instruction::CastOn{2}};
    pattern.rows.push_back(row0);

    RowInstruction row1;
    row1.side = RowSide::RS;
    row1.stitches = {instruction::Knit{}, instruction::Knit{}};
    pattern.rows.push_back(row1);

    RowInstruction row2;
    row2.side = RowSide::RS;
    row2.stitches = {instruction::Purl{}, instruction::Purl{}};
    pattern.rows.push_back(row2);

    auto yarn = default_yarn();
    auto gauge = default_gauge();
    auto graph = StitchGraph::from_instructions(pattern);
    auto yarn_path = YarnPath::from_stitch_graph(graph, yarn, gauge);

    SurfaceGraph surface = SurfaceBuilder::from_yarn_path(yarn_path, yarn, gauge);

    // Two-pass cast-on: 2 foundation + 2 return = 4 segments
    // Find continuity edges in knit row (4->5) and purl row (6->7)
    float knit_continuity_stiffness = 0.0f;
    float purl_continuity_stiffness = 0.0f;

    for (const auto& edge : surface.edges()) {
        if (edge.type == EdgeType::YarnContinuity) {
            if (edge.node_a == 4 && edge.node_b == 5) {
                knit_continuity_stiffness = edge.stiffness;
            } else if (edge.node_a == 6 && edge.node_b == 7) {
                purl_continuity_stiffness = edge.stiffness;
            }
        }
    }

    // Purl should have less stiffness than knit (less yarn density)
    EXPECT_GT(knit_continuity_stiffness, 0.0f);
    EXPECT_GT(purl_continuity_stiffness, 0.0f);
    EXPECT_LT(purl_continuity_stiffness, knit_continuity_stiffness)
        << "Purl continuity should be less stiff than knit (less yarn density)";
}

TEST(SurfaceYarnLength, OrientationAffectsZPosition) {
    // Pattern: Cast on 2, Row 1: K2, Row 2: P2
    PatternInstructions pattern;

    RowInstruction row0;
    row0.side = RowSide::RS;
    row0.stitches = {instruction::CastOn{2}};
    pattern.rows.push_back(row0);

    RowInstruction row1;
    row1.side = RowSide::RS;
    row1.stitches = {instruction::Knit{}, instruction::Knit{}};
    pattern.rows.push_back(row1);

    RowInstruction row2;
    row2.side = RowSide::RS;
    row2.stitches = {instruction::Purl{}, instruction::Purl{}};
    pattern.rows.push_back(row2);

    auto yarn = default_yarn();
    auto gauge = default_gauge();
    auto graph = StitchGraph::from_instructions(pattern);
    auto yarn_path = YarnPath::from_stitch_graph(graph, yarn, gauge);

    SurfaceGraph surface = SurfaceBuilder::from_yarn_path(yarn_path, yarn, gauge);

    // Two-pass cast-on: 2 foundation + 2 return = 4 segments
    // RS Knit segments: 4-5 → Front orientation → positive Z
    // RS Purl segments: 6-7 → Back orientation → negative Z
    float knit_z_avg = (surface.node(4).position.z + surface.node(5).position.z) / 2.0f;
    float purl_z_avg = (surface.node(6).position.z + surface.node(7).position.z) / 2.0f;

    EXPECT_GT(knit_z_avg, 0.0f)
        << "Knit stitches should have positive Z (curl forward)";
    EXPECT_LT(purl_z_avg, 0.0f)
        << "Purl stitches should have negative Z (curl backward)";
}

TEST(SurfaceYarnLength, TransferredStitchLooserPassthrough) {
    // Pattern: Cast on 3, Row 1: K1, Slip, K1
    PatternInstructions pattern;

    RowInstruction row0;
    row0.side = RowSide::RS;
    row0.stitches = {instruction::CastOn{3}};
    pattern.rows.push_back(row0);

    RowInstruction row1;
    row1.side = RowSide::RS;
    row1.stitches = {instruction::Knit{}, instruction::Slip{}, instruction::Knit{}};
    pattern.rows.push_back(row1);

    auto yarn = default_yarn();
    auto gauge = default_gauge();
    auto graph = StitchGraph::from_instructions(pattern);
    auto yarn_path = YarnPath::from_stitch_graph(graph, yarn, gauge);

    SurfaceGraph surface = SurfaceBuilder::from_yarn_path(yarn_path, yarn, gauge);

    // Two-pass cast-on: 3 foundation + 3 return = 6 segments
    // Find passthrough edges for knit (node 6) and slip (node 7)
    float knit_passthrough_stiffness = 0.0f;
    float slip_passthrough_stiffness = 0.0f;

    for (const auto& edge : surface.edges()) {
        if (edge.type == EdgeType::PassThrough) {
            if (edge.node_a == 6) {
                knit_passthrough_stiffness = edge.stiffness;
            } else if (edge.node_a == 7) {
                slip_passthrough_stiffness = edge.stiffness;
            }
        }
    }

    // Slip passthrough should be much less stiff than knit passthrough
    EXPECT_GT(knit_passthrough_stiffness, 0.0f);
    EXPECT_GT(slip_passthrough_stiffness, 0.0f);
    EXPECT_LT(slip_passthrough_stiffness, knit_passthrough_stiffness * 0.5f)
        << "Slip stitch passthrough should be much looser than worked stitch";
}
