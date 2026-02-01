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

    // Knit stitches (segments 3-5) should have yarn length = π * loop_height + stitch_width
    float loop_height = gauge.loop_height(yarn.compressed_radius);
    float stitch_width = gauge.stitch_width(yarn.compressed_radius);
    float expected_knit_length = M_PI * loop_height + stitch_width;

    for (size_t i = 3; i < 6; ++i) {
        EXPECT_NEAR(segments[i].target_yarn_length, expected_knit_length, expected_knit_length * 0.01f)
            << "Knit segment " << i << " should have base yarn length";
    }
}

TEST(YarnPath, YarnLengthCalculatedForPurl) {
    // Pattern: Cast on 3, Row 1 (WS): P3
    PatternInstructions pattern;

    RowInstruction row0;
    row0.side = RowSide::RS;
    row0.stitches = {instruction::CastOn{3}};
    pattern.rows.push_back(row0);

    RowInstruction row1;
    row1.side = RowSide::WS;
    row1.stitches = {instruction::Purl{}, instruction::Purl{}, instruction::Purl{}};
    pattern.rows.push_back(row1);

    auto yarn = default_yarn();
    auto gauge = default_gauge();
    auto graph = StitchGraph::from_instructions(pattern);
    auto yarn_path = YarnPath::from_stitch_graph(graph, yarn, gauge);

    const auto& segments = yarn_path.segments();

    // Purl stitches (segments 3-5) should have yarn length = 0.88 * (π * loop_height + stitch_width)
    float loop_height = gauge.loop_height(yarn.compressed_radius);
    float stitch_width = gauge.stitch_width(yarn.compressed_radius);
    float base_length = M_PI * loop_height + stitch_width;
    float expected_purl_length = 0.88f * base_length;

    for (size_t i = 3; i < 6; ++i) {
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

    // Slip stitch (segment 4) should have yarn length = 0.5 * stitch_width
    float stitch_width = gauge.stitch_width(yarn.compressed_radius);
    float expected_slip_length = 0.5f * stitch_width;

    EXPECT_NEAR(segments[4].target_yarn_length, expected_slip_length, expected_slip_length * 0.01f)
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

    // K2tog (segment 4) should have 0.82 * base length
    // SSK (segment 5) should have 0.86 * base length
    float loop_height = gauge.loop_height(yarn.compressed_radius);
    float stitch_width = gauge.stitch_width(yarn.compressed_radius);
    float base_length = M_PI * loop_height + stitch_width;

    float expected_k2tog = 0.82f * base_length;
    float expected_ssk = 0.86f * base_length;

    EXPECT_NEAR(segments[4].target_yarn_length, expected_k2tog, expected_k2tog * 0.01f)
        << "K2tog should use 82% of base yarn length";

    EXPECT_NEAR(segments[5].target_yarn_length, expected_ssk, expected_ssk * 0.01f)
        << "SSK should use 86% of base yarn length";

    // SSK should use slightly more yarn than K2tog
    EXPECT_GT(segments[5].target_yarn_length, segments[4].target_yarn_length)
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
    row2.side = RowSide::WS;
    row2.stitches = {instruction::Purl{}, instruction::Purl{}};
    pattern.rows.push_back(row2);

    auto yarn = default_yarn();
    auto gauge = default_gauge();
    auto graph = StitchGraph::from_instructions(pattern);
    auto yarn_path = YarnPath::from_stitch_graph(graph, yarn, gauge);

    SurfaceGraph surface = SurfaceBuilder::from_yarn_path(yarn_path, yarn, gauge);

    // Knit segments: 2-3
    // Purl segments: 4-5
    float knit_mass_avg = (surface.node(2).mass + surface.node(3).mass) / 2.0f;
    float purl_mass_avg = (surface.node(4).mass + surface.node(5).mass) / 2.0f;

    // Purl should have ~88% of knit mass
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

    // Knit: segment 3, Slip: segment 4, Knit: segment 5
    float knit_mass = surface.node(3).mass;
    float slip_mass = surface.node(4).mass;

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

    // K2tog: segment 4, SSK: segment 5
    float k2tog_mass = surface.node(4).mass;
    float ssk_mass = surface.node(5).mass;

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

    // Knit: segment 2, YO: segment 3, Knit: segment 4
    float knit_mass = surface.node(2).mass;
    float yo_mass = surface.node(3).mass;

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
    row2.side = RowSide::WS;
    row2.stitches = {instruction::Purl{}, instruction::Purl{}};
    pattern.rows.push_back(row2);

    auto yarn = default_yarn();
    auto gauge = default_gauge();
    auto graph = StitchGraph::from_instructions(pattern);
    auto yarn_path = YarnPath::from_stitch_graph(graph, yarn, gauge);

    SurfaceGraph surface = SurfaceBuilder::from_yarn_path(yarn_path, yarn, gauge);

    // Find continuity edges in knit row (2->3) and purl row (4->5)
    // Segments: 0-1=CastOn, 2-3=Knit, 4-5=Purl
    float knit_continuity_stiffness = 0.0f;
    float purl_continuity_stiffness = 0.0f;

    for (const auto& edge : surface.edges()) {
        if (edge.type == EdgeType::YarnContinuity) {
            if (edge.node_a == 2 && edge.node_b == 3) {
                knit_continuity_stiffness = edge.stiffness;
            } else if (edge.node_a == 4 && edge.node_b == 5) {
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
    row2.side = RowSide::WS;
    row2.stitches = {instruction::Purl{}, instruction::Purl{}};
    pattern.rows.push_back(row2);

    auto yarn = default_yarn();
    auto gauge = default_gauge();
    auto graph = StitchGraph::from_instructions(pattern);
    auto yarn_path = YarnPath::from_stitch_graph(graph, yarn, gauge);

    SurfaceGraph surface = SurfaceBuilder::from_yarn_path(yarn_path, yarn, gauge);

    // Knit segments: 2-3 should have positive Z
    // Purl segments: 4-5 should have negative Z
    float knit_z_avg = (surface.node(2).position.z + surface.node(3).position.z) / 2.0f;
    float purl_z_avg = (surface.node(4).position.z + surface.node(5).position.z) / 2.0f;

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

    // Find passthrough edges for knit (node 3) and slip (node 4)
    float knit_passthrough_stiffness = 0.0f;
    float slip_passthrough_stiffness = 0.0f;

    for (const auto& edge : surface.edges()) {
        if (edge.type == EdgeType::PassThrough) {
            if (edge.node_a == 3) {
                knit_passthrough_stiffness = edge.stiffness;
            } else if (edge.node_a == 4) {
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
