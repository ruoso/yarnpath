#include "yarn_path.hpp"
#include "stitch_instruction.hpp"
#include "row_instruction.hpp"
#include "test_helpers.hpp"
#include <gtest/gtest.h>

using namespace yarnpath;
using namespace yarnpath::test;

TEST(YarnTopology, KnitVsPurlOrientation) {
    // Test that Knit and Purl have different orientations
    // Pattern: Cast on 3, Row 1 (RS): K3, Row 2 (RS): P3
    // RS Knit = Front-facing, RS Purl = Back-facing
    // (WS Purl would flip to Front, same as stockinette, so we use RS Purl)
    PatternInstructions pattern;

    // Cast on
    RowInstruction row0;
    row0.side = RowSide::RS;
    row0.stitches = {instruction::CastOn{3}};
    pattern.rows.push_back(row0);

    // Row 1: Knit 3 (RS)
    RowInstruction row1;
    row1.side = RowSide::RS;
    row1.stitches = {instruction::Knit{}, instruction::Knit{}, instruction::Knit{}};
    pattern.rows.push_back(row1);

    // Row 2: Purl 3 (RS) - creates back-facing loops
    RowInstruction row2;
    row2.side = RowSide::RS;
    row2.stitches = {instruction::Purl{}, instruction::Purl{}, instruction::Purl{}};
    pattern.rows.push_back(row2);

    auto graph = StitchGraph::from_instructions(pattern);
    auto yarn_path = YarnPath::from_stitch_graph(graph, default_yarn(), default_gauge());

    const auto& segments = yarn_path.segments();

    // Two-pass cast-on: 3 foundation segments (0-2, Created) + 3 return segments (3-5, Worked)
    for (size_t i = 0; i < 3; ++i) {
        EXPECT_EQ(segments[i].orientation, YarnSegment::LoopOrientation::Neutral);
        EXPECT_EQ(segments[i].work_type, YarnSegment::WorkType::Created);
    }
    for (size_t i = 3; i < 6; ++i) {
        EXPECT_EQ(segments[i].orientation, YarnSegment::LoopOrientation::Neutral);
        EXPECT_EQ(segments[i].work_type, YarnSegment::WorkType::Worked);
    }

    // Knit segments (6-8) should be Front-facing
    for (size_t i = 6; i < 9; ++i) {
        EXPECT_EQ(segments[i].orientation, YarnSegment::LoopOrientation::Front);
        EXPECT_EQ(segments[i].work_type, YarnSegment::WorkType::Worked);
    }

    // Purl segments (9-11) should be Back-facing
    for (size_t i = 9; i < 12; ++i) {
        EXPECT_EQ(segments[i].orientation, YarnSegment::LoopOrientation::Back);
        EXPECT_EQ(segments[i].work_type, YarnSegment::WorkType::Worked);
    }
}

TEST(YarnTopology, K2togVsSSKWrapDirection) {
    // Test that K2tog and SSK have different wrap directions
    // Pattern: Cast on 6, Row 1: K1, K2tog, SSK, K1
    PatternInstructions pattern;

    // Cast on
    RowInstruction row0;
    row0.side = RowSide::RS;
    row0.stitches = {instruction::CastOn{6}};
    pattern.rows.push_back(row0);

    // Row 1: K1, K2tog, SSK, K1
    RowInstruction row1;
    row1.side = RowSide::RS;
    row1.stitches = {instruction::Knit{}, instruction::K2tog{}, instruction::SSK{}, instruction::Knit{}};
    pattern.rows.push_back(row1);

    auto graph = StitchGraph::from_instructions(pattern);
    auto yarn_path = YarnPath::from_stitch_graph(graph, default_yarn(), default_gauge());

    const auto& segments = yarn_path.segments();

    // Two-pass cast-on: segments 0-11 (6 foundation + 6 return)
    // Row 1: K1 (12), K2tog (13), SSK (14), K1 (15)

    // First knit (segment 12) should have Front orientation, no wrap direction
    EXPECT_EQ(segments[12].orientation, YarnSegment::LoopOrientation::Front);
    EXPECT_EQ(segments[12].wrap_direction, YarnSegment::WrapDirection::None);
    EXPECT_EQ(segments[12].through.size(), 1u);

    // K2tog (segment 13) should have Clockwise wrap
    EXPECT_EQ(segments[13].orientation, YarnSegment::LoopOrientation::Front);
    EXPECT_EQ(segments[13].wrap_direction, YarnSegment::WrapDirection::Clockwise);
    EXPECT_EQ(segments[13].work_type, YarnSegment::WorkType::Worked);
    EXPECT_EQ(segments[13].through.size(), 2u);

    // SSK (segment 14) should have CounterClockwise wrap
    EXPECT_EQ(segments[14].orientation, YarnSegment::LoopOrientation::Front);
    EXPECT_EQ(segments[14].wrap_direction, YarnSegment::WrapDirection::CounterClockwise);
    EXPECT_EQ(segments[14].work_type, YarnSegment::WorkType::Worked);
    EXPECT_EQ(segments[14].through.size(), 2u);

    // Last knit (segment 15)
    EXPECT_EQ(segments[15].orientation, YarnSegment::LoopOrientation::Front);
    EXPECT_EQ(segments[15].wrap_direction, YarnSegment::WrapDirection::None);
    EXPECT_EQ(segments[15].through.size(), 1u);
}

TEST(YarnTopology, M1LVsM1RTwist) {
    // Test that M1L and M1R have different twist directions
    // Pattern: Cast on 3, Row 1: K1, M1L, M1R, K1
    PatternInstructions pattern;

    // Cast on
    RowInstruction row0;
    row0.side = RowSide::RS;
    row0.stitches = {instruction::CastOn{3}};
    pattern.rows.push_back(row0);

    // Row 1: K1, M1L, M1R, K1
    RowInstruction row1;
    row1.side = RowSide::RS;
    row1.stitches = {instruction::Knit{}, instruction::M1L{}, instruction::M1R{}, instruction::Knit{}};
    pattern.rows.push_back(row1);

    auto graph = StitchGraph::from_instructions(pattern);
    auto yarn_path = YarnPath::from_stitch_graph(graph, default_yarn(), default_gauge());

    const auto& segments = yarn_path.segments();

    // Two-pass cast-on: segments 0-5 (3 foundation + 3 return)
    // Row 1: K1 (6), M1L (7), M1R (8), K1 (9)

    // M1L (segment 7) should have CounterClockwise twist
    EXPECT_EQ(segments[7].orientation, YarnSegment::LoopOrientation::Neutral);
    EXPECT_EQ(segments[7].wrap_direction, YarnSegment::WrapDirection::CounterClockwise);
    EXPECT_EQ(segments[7].work_type, YarnSegment::WorkType::Created);
    EXPECT_TRUE(segments[7].through.empty());

    // M1R (segment 8) should have Clockwise twist
    EXPECT_EQ(segments[8].orientation, YarnSegment::LoopOrientation::Neutral);
    EXPECT_EQ(segments[8].wrap_direction, YarnSegment::WrapDirection::Clockwise);
    EXPECT_EQ(segments[8].work_type, YarnSegment::WorkType::Created);
    EXPECT_TRUE(segments[8].through.empty());
}

TEST(YarnTopology, SlipVsKnitWorkSemantics) {
    // Test that Slip has different work semantics than Knit
    // Pattern: Cast on 3, Row 1: K1, Slip, K1
    PatternInstructions pattern;

    // Cast on
    RowInstruction row0;
    row0.side = RowSide::RS;
    row0.stitches = {instruction::CastOn{3}};
    pattern.rows.push_back(row0);

    // Row 1: K1, Slip, K1
    RowInstruction row1;
    row1.side = RowSide::RS;
    row1.stitches = {instruction::Knit{}, instruction::Slip{}, instruction::Knit{}};
    pattern.rows.push_back(row1);

    auto graph = StitchGraph::from_instructions(pattern);
    auto yarn_path = YarnPath::from_stitch_graph(graph, default_yarn(), default_gauge());

    const auto& segments = yarn_path.segments();

    // Two-pass cast-on: segments 0-5 (3 foundation + 3 return)
    // Row 1: K1 (6), Slip (7), K1 (8)

    // First knit (segment 6) should be Worked
    EXPECT_EQ(segments[6].work_type, YarnSegment::WorkType::Worked);
    EXPECT_EQ(segments[6].orientation, YarnSegment::LoopOrientation::Front);
    EXPECT_EQ(segments[6].through.size(), 1u);

    // Slip (segment 7) should be Transferred, not Worked
    EXPECT_EQ(segments[7].work_type, YarnSegment::WorkType::Transferred);
    EXPECT_EQ(segments[7].orientation, YarnSegment::LoopOrientation::Neutral);
    EXPECT_EQ(segments[7].through.size(), 1u);

    // Last knit (segment 8) should be Worked
    EXPECT_EQ(segments[8].work_type, YarnSegment::WorkType::Worked);
    EXPECT_EQ(segments[8].orientation, YarnSegment::LoopOrientation::Front);
    EXPECT_EQ(segments[8].through.size(), 1u);
}

TEST(YarnTopology, YarnOverCreation) {
    // Test that YarnOver has Created work type
    // Pattern: Cast on 2, Row 1: K1, YO, K1
    PatternInstructions pattern;

    // Cast on
    RowInstruction row0;
    row0.side = RowSide::RS;
    row0.stitches = {instruction::CastOn{2}};
    pattern.rows.push_back(row0);

    // Row 1: K1, YO, K1
    RowInstruction row1;
    row1.side = RowSide::RS;
    row1.stitches = {instruction::Knit{}, instruction::YarnOver{}, instruction::Knit{}};
    pattern.rows.push_back(row1);

    auto graph = StitchGraph::from_instructions(pattern);
    auto yarn_path = YarnPath::from_stitch_graph(graph, default_yarn(), default_gauge());

    const auto& segments = yarn_path.segments();

    // Two-pass cast-on: segments 0-3 (2 foundation + 2 return)
    // Row 1: K1 (4), YO (5), K1 (6)

    // YarnOver (segment 5) should be Created with Neutral orientation
    EXPECT_EQ(segments[5].work_type, YarnSegment::WorkType::Created);
    EXPECT_EQ(segments[5].orientation, YarnSegment::LoopOrientation::Neutral);
    EXPECT_EQ(segments[5].wrap_direction, YarnSegment::WrapDirection::None);
    EXPECT_TRUE(segments[5].through.empty());
}

TEST(YarnTopology, S2KPTripleDecrease) {
    // Test that S2KP has correct wrap direction
    // Pattern: Cast on 5, Row 1: K1, S2KP, K1
    PatternInstructions pattern;

    // Cast on
    RowInstruction row0;
    row0.side = RowSide::RS;
    row0.stitches = {instruction::CastOn{5}};
    pattern.rows.push_back(row0);

    // Row 1: K1, S2KP, K1
    RowInstruction row1;
    row1.side = RowSide::RS;
    row1.stitches = {instruction::Knit{}, instruction::S2KP{}, instruction::Knit{}};
    pattern.rows.push_back(row1);

    auto graph = StitchGraph::from_instructions(pattern);
    auto yarn_path = YarnPath::from_stitch_graph(graph, default_yarn(), default_gauge());

    const auto& segments = yarn_path.segments();

    // Two-pass cast-on: segments 0-9 (5 foundation + 5 return)
    // Row 1: K1 (10), S2KP (11), K1 (12)

    // S2KP (segment 11) should have 3 parents and CounterClockwise wrap
    EXPECT_EQ(segments[11].through.size(), 3u);
    EXPECT_EQ(segments[11].wrap_direction, YarnSegment::WrapDirection::CounterClockwise);
    EXPECT_EQ(segments[11].orientation, YarnSegment::LoopOrientation::Front);
    EXPECT_EQ(segments[11].work_type, YarnSegment::WorkType::Worked);
}

TEST(YarnTopology, CablePreservesOrientation) {
    // Test that cable stitches maintain Front orientation
    // Pattern: Cast on 4, Row 1: CableLeft{2, 2}
    PatternInstructions pattern;

    // Cast on
    RowInstruction row0;
    row0.side = RowSide::RS;
    row0.stitches = {instruction::CastOn{4}};
    pattern.rows.push_back(row0);

    // Row 1: Cable Left (2 held, 2 crossed)
    RowInstruction row1;
    row1.side = RowSide::RS;
    instruction::CableLeft cable;
    cable.hold = 2;
    cable.cross = 2;
    row1.stitches = {cable};
    pattern.rows.push_back(row1);

    auto graph = StitchGraph::from_instructions(pattern);
    auto yarn_path = YarnPath::from_stitch_graph(graph, default_yarn(), default_gauge());

    const auto& segments = yarn_path.segments();

    // Two-pass cast-on: segments 0-7 (4 foundation + 4 return)
    // Row 1: Cable produces 4 stitches (segments 8-11)

    // All cable stitches should have Front orientation and Worked type
    for (size_t i = 8; i < 12; ++i) {
        EXPECT_EQ(segments[i].orientation, YarnSegment::LoopOrientation::Front);
        EXPECT_EQ(segments[i].work_type, YarnSegment::WorkType::Worked);
        EXPECT_EQ(segments[i].through.size(), 1u);
    }
}

TEST(YarnTopology, KFBPreservesOrientation) {
    // Test that KFB (Knit Front and Back) has correct topology
    // Pattern: Cast on 3, Row 1: K1, KFB, K1
    PatternInstructions pattern;

    // Cast on
    RowInstruction row0;
    row0.side = RowSide::RS;
    row0.stitches = {instruction::CastOn{3}};
    pattern.rows.push_back(row0);

    // Row 1: K1, KFB, K1
    RowInstruction row1;
    row1.side = RowSide::RS;
    row1.stitches = {instruction::Knit{}, instruction::KFB{}, instruction::Knit{}};
    pattern.rows.push_back(row1);

    auto graph = StitchGraph::from_instructions(pattern);
    auto yarn_path = YarnPath::from_stitch_graph(graph, default_yarn(), default_gauge());

    const auto& segments = yarn_path.segments();

    // Two-pass cast-on: segments 0-5 (3 foundation + 3 return)
    // Row 1: K1 (6), KFB produces 2 stitches (7, 8), K1 (9)

    // Both KFB stitches should have Front orientation and same parent
    EXPECT_EQ(segments[7].orientation, YarnSegment::LoopOrientation::Front);
    EXPECT_EQ(segments[7].work_type, YarnSegment::WorkType::Worked);
    EXPECT_EQ(segments[7].through.size(), 1u);

    EXPECT_EQ(segments[8].orientation, YarnSegment::LoopOrientation::Front);
    EXPECT_EQ(segments[8].work_type, YarnSegment::WorkType::Worked);
    EXPECT_EQ(segments[8].through.size(), 1u);

    // Both should have the same parent
    EXPECT_EQ(segments[7].through[0], segments[8].through[0]);
}

TEST(YarnTopology, BindOffHasWorkedType) {
    // Test that BindOff has Worked type
    // Pattern: Cast on 3, Row 1: K3, Row 2: BindOff 3
    PatternInstructions pattern;

    // Cast on
    RowInstruction row0;
    row0.side = RowSide::RS;
    row0.stitches = {instruction::CastOn{3}};
    pattern.rows.push_back(row0);

    // Row 1: K3
    RowInstruction row1;
    row1.side = RowSide::RS;
    row1.stitches = {instruction::Knit{}, instruction::Knit{}, instruction::Knit{}};
    pattern.rows.push_back(row1);

    // Row 2: BindOff 3
    RowInstruction row2;
    row2.side = RowSide::RS;
    row2.stitches = {instruction::BindOff{3}};
    pattern.rows.push_back(row2);

    auto graph = StitchGraph::from_instructions(pattern);
    auto yarn_path = YarnPath::from_stitch_graph(graph, default_yarn(), default_gauge());

    const auto& segments = yarn_path.segments();

    // Two-pass cast-on: segments 0-5 (3 foundation + 3 return)
    // Row 1: K3 (segments 6-8)
    // Row 2: BindOff 3 (segments 9-11)

    // BindOff stitches should have Worked type
    for (size_t i = 9; i < 12; ++i) {
        EXPECT_EQ(segments[i].work_type, YarnSegment::WorkType::Worked);
        EXPECT_EQ(segments[i].orientation, YarnSegment::LoopOrientation::Neutral);
        EXPECT_EQ(segments[i].through.size(), 1u);
    }
}
