#include "yarn_path.hpp"
#include "stitch_instruction.hpp"
#include "row_instruction.hpp"
#include <gtest/gtest.h>

using namespace yarnpath;

TEST(YarnTopology, KnitVsPurlOrientation) {
    // Test that Knit and Purl have different orientations
    // Pattern: Cast on 3, Row 1 (RS): K3, Row 2 (WS): P3
    PatternInstructions pattern;

    // Cast on
    RowInstruction row0;
    row0.side = RowSide::RS;
    row0.stitches = {instruction::CastOn{3}};
    pattern.rows.push_back(row0);

    // Row 1: Knit 3
    RowInstruction row1;
    row1.side = RowSide::RS;
    row1.stitches = {instruction::Knit{}, instruction::Knit{}, instruction::Knit{}};
    pattern.rows.push_back(row1);

    // Row 2: Purl 3
    RowInstruction row2;
    row2.side = RowSide::WS;
    row2.stitches = {instruction::Purl{}, instruction::Purl{}, instruction::Purl{}};
    pattern.rows.push_back(row2);

    auto graph = StitchGraph::from_instructions(pattern);
    auto yarn_path = YarnPath::from_stitch_graph(graph);

    const auto& segments = yarn_path.segments();

    // Cast on segments (0-2) should be Neutral
    for (size_t i = 0; i < 3; ++i) {
        EXPECT_EQ(segments[i].orientation, YarnSegment::LoopOrientation::Neutral);
        EXPECT_EQ(segments[i].work_type, YarnSegment::WorkType::Created);
    }

    // Knit segments (3-5) should be Front-facing
    for (size_t i = 3; i < 6; ++i) {
        EXPECT_EQ(segments[i].orientation, YarnSegment::LoopOrientation::Front);
        EXPECT_EQ(segments[i].work_type, YarnSegment::WorkType::Worked);
    }

    // Purl segments (6-8) should be Back-facing
    for (size_t i = 6; i < 9; ++i) {
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
    auto yarn_path = YarnPath::from_stitch_graph(graph);

    const auto& segments = yarn_path.segments();

    // Cast on: segments 0-5
    // Row 1: K1 (6), K2tog (7), SSK (8), K1 (9)

    // First knit (segment 6) should have Front orientation, no wrap direction
    EXPECT_EQ(segments[6].orientation, YarnSegment::LoopOrientation::Front);
    EXPECT_EQ(segments[6].wrap_direction, YarnSegment::WrapDirection::None);
    EXPECT_EQ(segments[6].through.size(), 1u);

    // K2tog (segment 7) should have Clockwise wrap
    EXPECT_EQ(segments[7].orientation, YarnSegment::LoopOrientation::Front);
    EXPECT_EQ(segments[7].wrap_direction, YarnSegment::WrapDirection::Clockwise);
    EXPECT_EQ(segments[7].work_type, YarnSegment::WorkType::Worked);
    EXPECT_EQ(segments[7].through.size(), 2u);

    // SSK (segment 8) should have CounterClockwise wrap
    EXPECT_EQ(segments[8].orientation, YarnSegment::LoopOrientation::Front);
    EXPECT_EQ(segments[8].wrap_direction, YarnSegment::WrapDirection::CounterClockwise);
    EXPECT_EQ(segments[8].work_type, YarnSegment::WorkType::Worked);
    EXPECT_EQ(segments[8].through.size(), 2u);

    // Last knit (segment 9)
    EXPECT_EQ(segments[9].orientation, YarnSegment::LoopOrientation::Front);
    EXPECT_EQ(segments[9].wrap_direction, YarnSegment::WrapDirection::None);
    EXPECT_EQ(segments[9].through.size(), 1u);
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
    auto yarn_path = YarnPath::from_stitch_graph(graph);

    const auto& segments = yarn_path.segments();

    // Cast on: segments 0-2
    // Row 1: K1 (3), M1L (4), M1R (5), K1 (6)

    // M1L (segment 4) should have CounterClockwise twist
    EXPECT_EQ(segments[4].orientation, YarnSegment::LoopOrientation::Neutral);
    EXPECT_EQ(segments[4].wrap_direction, YarnSegment::WrapDirection::CounterClockwise);
    EXPECT_EQ(segments[4].work_type, YarnSegment::WorkType::Created);
    EXPECT_TRUE(segments[4].through.empty());

    // M1R (segment 5) should have Clockwise twist
    EXPECT_EQ(segments[5].orientation, YarnSegment::LoopOrientation::Neutral);
    EXPECT_EQ(segments[5].wrap_direction, YarnSegment::WrapDirection::Clockwise);
    EXPECT_EQ(segments[5].work_type, YarnSegment::WorkType::Created);
    EXPECT_TRUE(segments[5].through.empty());
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
    auto yarn_path = YarnPath::from_stitch_graph(graph);

    const auto& segments = yarn_path.segments();

    // Cast on: segments 0-2
    // Row 1: K1 (3), Slip (4), K1 (5)

    // First knit (segment 3) should be Worked
    EXPECT_EQ(segments[3].work_type, YarnSegment::WorkType::Worked);
    EXPECT_EQ(segments[3].orientation, YarnSegment::LoopOrientation::Front);
    EXPECT_EQ(segments[3].through.size(), 1u);

    // Slip (segment 4) should be Transferred, not Worked
    EXPECT_EQ(segments[4].work_type, YarnSegment::WorkType::Transferred);
    EXPECT_EQ(segments[4].orientation, YarnSegment::LoopOrientation::Neutral);
    EXPECT_EQ(segments[4].through.size(), 1u);

    // Last knit (segment 5) should be Worked
    EXPECT_EQ(segments[5].work_type, YarnSegment::WorkType::Worked);
    EXPECT_EQ(segments[5].orientation, YarnSegment::LoopOrientation::Front);
    EXPECT_EQ(segments[5].through.size(), 1u);
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
    auto yarn_path = YarnPath::from_stitch_graph(graph);

    const auto& segments = yarn_path.segments();

    // Cast on: segments 0-1
    // Row 1: K1 (2), YO (3), K1 (4)

    // YarnOver (segment 3) should be Created with Neutral orientation
    EXPECT_EQ(segments[3].work_type, YarnSegment::WorkType::Created);
    EXPECT_EQ(segments[3].orientation, YarnSegment::LoopOrientation::Neutral);
    EXPECT_EQ(segments[3].wrap_direction, YarnSegment::WrapDirection::None);
    EXPECT_TRUE(segments[3].through.empty());
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
    auto yarn_path = YarnPath::from_stitch_graph(graph);

    const auto& segments = yarn_path.segments();

    // Cast on: segments 0-4
    // Row 1: K1 (5), S2KP (6), K1 (7)

    // S2KP (segment 6) should have 3 parents and CounterClockwise wrap
    EXPECT_EQ(segments[6].through.size(), 3u);
    EXPECT_EQ(segments[6].wrap_direction, YarnSegment::WrapDirection::CounterClockwise);
    EXPECT_EQ(segments[6].orientation, YarnSegment::LoopOrientation::Front);
    EXPECT_EQ(segments[6].work_type, YarnSegment::WorkType::Worked);
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
    auto yarn_path = YarnPath::from_stitch_graph(graph);

    const auto& segments = yarn_path.segments();

    // Cast on: segments 0-3
    // Row 1: Cable produces 4 stitches (segments 4-7)

    // All cable stitches should have Front orientation and Worked type
    for (size_t i = 4; i < 8; ++i) {
        EXPECT_EQ(segments[i].orientation, YarnSegment::LoopOrientation::Front);
        EXPECT_EQ(segments[i].work_type, YarnSegment::WorkType::Worked);
        EXPECT_EQ(segments[i].through.size(), 1u);
    }
}

TEST(YarnTopology, KFBPreservesOrientation) {
    // Test that KFB (Knit Front and Back) has correct topology
    // Pattern: Cast on 2, Row 1: K1, KFB, K1
    PatternInstructions pattern;

    // Cast on
    RowInstruction row0;
    row0.side = RowSide::RS;
    row0.stitches = {instruction::CastOn{2}};
    pattern.rows.push_back(row0);

    // Row 1: K1, KFB, K1
    RowInstruction row1;
    row1.side = RowSide::RS;
    row1.stitches = {instruction::Knit{}, instruction::KFB{}, instruction::Knit{}};
    pattern.rows.push_back(row1);

    auto graph = StitchGraph::from_instructions(pattern);
    auto yarn_path = YarnPath::from_stitch_graph(graph);

    const auto& segments = yarn_path.segments();

    // Cast on: segments 0-1
    // Row 1: K1 (2), KFB produces 2 stitches (3, 4), K1 (5)

    // Both KFB stitches should have Front orientation and same parent
    EXPECT_EQ(segments[3].orientation, YarnSegment::LoopOrientation::Front);
    EXPECT_EQ(segments[3].work_type, YarnSegment::WorkType::Worked);
    EXPECT_EQ(segments[3].through.size(), 1u);

    EXPECT_EQ(segments[4].orientation, YarnSegment::LoopOrientation::Front);
    EXPECT_EQ(segments[4].work_type, YarnSegment::WorkType::Worked);
    EXPECT_EQ(segments[4].through.size(), 1u);

    // Both should have the same parent
    EXPECT_EQ(segments[3].through[0], segments[4].through[0]);
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
    auto yarn_path = YarnPath::from_stitch_graph(graph);

    const auto& segments = yarn_path.segments();

    // Cast on: segments 0-2
    // Row 1: K3 (segments 3-5)
    // Row 2: BindOff 3 (segments 6-8)

    // BindOff stitches should have Worked type
    for (size_t i = 6; i < 9; ++i) {
        EXPECT_EQ(segments[i].work_type, YarnSegment::WorkType::Worked);
        EXPECT_EQ(segments[i].orientation, YarnSegment::LoopOrientation::Neutral);
        EXPECT_EQ(segments[i].through.size(), 1u);
    }
}
