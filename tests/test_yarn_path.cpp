#include "yarn_path.hpp"
#include "stitch_instruction.hpp"
#include "row_instruction.hpp"
#include "test_helpers.hpp"
#include <gtest/gtest.h>
#include <algorithm>

using namespace yarnpath;
using namespace yarnpath::test;

TEST(YarnPath, CastOnOnly) {
    // Cast on 3 stitches
    PatternInstructions pattern;

    RowInstruction row0;
    row0.side = RowSide::RS;
    row0.stitches = {instruction::CastOn{3}};
    pattern.rows.push_back(row0);

    auto graph = StitchGraph::from_instructions(pattern);
    auto yarn_path = YarnPath::from_stitch_graph(graph, default_yarn(), default_gauge());

    // Should have 3 segments that form loops (one per cast-on)
    size_t loop_count = 0;
    for (const auto& seg : yarn_path.segments()) {
        if (seg.forms_loop) {
            loop_count++;
            // Cast-on loops have no parents
            EXPECT_TRUE(seg.through.empty());
        }
    }
    EXPECT_EQ(loop_count, 3u);
}

TEST(YarnPath, SingleRowKnit) {
    // Cast on 3, then knit 3
    PatternInstructions pattern;

    RowInstruction row0;
    row0.side = RowSide::RS;
    row0.stitches = {instruction::CastOn{3}};
    pattern.rows.push_back(row0);

    RowInstruction row1;
    row1.side = RowSide::RS;
    instruction::Repeat rep;
    rep.instructions = {instruction::Knit{}};
    rep.times = 3;
    row1.stitches = {rep};
    pattern.rows.push_back(row1);

    auto graph = StitchGraph::from_instructions(pattern);
    auto yarn_path = YarnPath::from_stitch_graph(graph, default_yarn(), default_gauge());

    // Count loops and verify parent relationships
    size_t loop_count = 0;
    size_t cast_on_count = 0;
    size_t worked_count = 0;

    for (const auto& seg : yarn_path.segments()) {
        if (seg.forms_loop) {
            loop_count++;
            if (seg.through.empty()) {
                cast_on_count++;
            } else {
                worked_count++;
                // Knit loops should have exactly one parent
                EXPECT_EQ(seg.through.size(), 1u);
            }
        }
    }

    // Should have 6 loops: 3 cast-on + 3 knit
    EXPECT_EQ(loop_count, 6u);
    EXPECT_EQ(cast_on_count, 3u);
    EXPECT_EQ(worked_count, 3u);
}

TEST(YarnPath, PurlRow) {
    // Cast on 3, then purl 3
    PatternInstructions pattern;

    RowInstruction row0;
    row0.side = RowSide::RS;
    row0.stitches = {instruction::CastOn{3}};
    pattern.rows.push_back(row0);

    RowInstruction row1;
    row1.side = RowSide::WS;
    instruction::Repeat rep;
    rep.instructions = {instruction::Purl{}};
    rep.times = 3;
    row1.stitches = {rep};
    pattern.rows.push_back(row1);

    auto graph = StitchGraph::from_instructions(pattern);
    auto yarn_path = YarnPath::from_stitch_graph(graph, default_yarn(), default_gauge());

    // Count loops
    size_t loop_count = 0;
    for (const auto& seg : yarn_path.segments()) {
        if (seg.forms_loop) {
            loop_count++;
        }
    }

    // Should have 6 loops: 3 cast-on + 3 purl
    EXPECT_EQ(loop_count, 6u);
}

TEST(YarnPath, StockinetteMultipleRows) {
    // Cast on 3, K3, P3, K3
    PatternInstructions pattern;

    RowInstruction row0;
    row0.side = RowSide::RS;
    row0.stitches = {instruction::CastOn{3}};
    pattern.rows.push_back(row0);

    RowInstruction row1;
    row1.side = RowSide::RS;
    instruction::Repeat rep1;
    rep1.instructions = {instruction::Knit{}};
    rep1.times = 3;
    row1.stitches = {rep1};
    pattern.rows.push_back(row1);

    RowInstruction row2;
    row2.side = RowSide::WS;
    instruction::Repeat rep2;
    rep2.instructions = {instruction::Purl{}};
    rep2.times = 3;
    row2.stitches = {rep2};
    pattern.rows.push_back(row2);

    RowInstruction row3;
    row3.side = RowSide::RS;
    instruction::Repeat rep3;
    rep3.instructions = {instruction::Knit{}};
    rep3.times = 3;
    row3.stitches = {rep3};
    pattern.rows.push_back(row3);

    auto graph = StitchGraph::from_instructions(pattern);
    auto yarn_path = YarnPath::from_stitch_graph(graph, default_yarn(), default_gauge());

    // Count loops
    size_t loop_count = 0;
    for (const auto& seg : yarn_path.segments()) {
        if (seg.forms_loop) {
            loop_count++;
        }
    }

    // Should have 12 loops: 3 cast-on + 3 knit + 3 purl + 3 knit
    EXPECT_EQ(loop_count, 12u);
}

TEST(YarnPath, ParentChildRelationships) {
    // Verify parent-child relationships through the through vector
    PatternInstructions pattern;

    RowInstruction row0;
    row0.side = RowSide::RS;
    row0.stitches = {instruction::CastOn{2}};
    pattern.rows.push_back(row0);

    RowInstruction row1;
    row1.side = RowSide::RS;
    instruction::Repeat rep;
    rep.instructions = {instruction::Knit{}};
    rep.times = 2;
    row1.stitches = {rep};
    pattern.rows.push_back(row1);

    auto graph = StitchGraph::from_instructions(pattern);
    auto yarn_path = YarnPath::from_stitch_graph(graph, default_yarn(), default_gauge());

    // Find loop segments
    std::vector<SegmentId> loop_ids;
    for (size_t i = 0; i < yarn_path.segments().size(); ++i) {
        if (yarn_path.segments()[i].forms_loop) {
            loop_ids.push_back(static_cast<SegmentId>(i));
        }
    }

    ASSERT_EQ(loop_ids.size(), 4u);  // 2 cast-on + 2 knit

    // First two are cast-ons (no parents)
    EXPECT_TRUE(yarn_path.segments()[loop_ids[0]].through.empty());
    EXPECT_TRUE(yarn_path.segments()[loop_ids[1]].through.empty());

    // Last two are knits (each has one parent)
    const auto& knit1 = yarn_path.segments()[loop_ids[2]];
    const auto& knit2 = yarn_path.segments()[loop_ids[3]];

    EXPECT_EQ(knit1.through.size(), 1u);
    EXPECT_EQ(knit2.through.size(), 1u);

    // Parents should be cast-on loops
    EXPECT_TRUE(knit1.through[0] == loop_ids[0] || knit1.through[0] == loop_ids[1]);
    EXPECT_TRUE(knit2.through[0] == loop_ids[0] || knit2.through[0] == loop_ids[1]);
}

TEST(YarnPath, BindOff) {
    // Cast on 3, K3, Bind off 3
    PatternInstructions pattern;

    RowInstruction row0;
    row0.side = RowSide::RS;
    row0.stitches = {instruction::CastOn{3}};
    pattern.rows.push_back(row0);

    RowInstruction row1;
    row1.side = RowSide::RS;
    instruction::Repeat rep1;
    rep1.instructions = {instruction::Knit{}};
    rep1.times = 3;
    row1.stitches = {rep1};
    pattern.rows.push_back(row1);

    RowInstruction row2;
    row2.side = RowSide::RS;
    row2.stitches = {instruction::BindOff{3}};
    pattern.rows.push_back(row2);

    auto graph = StitchGraph::from_instructions(pattern);
    auto yarn_path = YarnPath::from_stitch_graph(graph, default_yarn(), default_gauge());

    // Count loops
    size_t loop_count = 0;
    for (const auto& seg : yarn_path.segments()) {
        if (seg.forms_loop) {
            loop_count++;
        }
    }

    // Should have 9 loops: 3 cast-on + 3 knit + 3 bind-off
    EXPECT_EQ(loop_count, 9u);
}

TEST(YarnPath, IsLoopHelper) {
    PatternInstructions pattern;

    RowInstruction row0;
    row0.side = RowSide::RS;
    row0.stitches = {instruction::CastOn{2}};
    pattern.rows.push_back(row0);

    auto graph = StitchGraph::from_instructions(pattern);
    auto yarn_path = YarnPath::from_stitch_graph(graph, default_yarn(), default_gauge());

    // Find which segments are loops
    for (size_t i = 0; i < yarn_path.segments().size(); ++i) {
        SegmentId id = static_cast<SegmentId>(i);
        EXPECT_EQ(yarn_path.is_loop(id), yarn_path.segments()[i].forms_loop);
    }
}

TEST(YarnPath, GetThroughHelper) {
    PatternInstructions pattern;

    RowInstruction row0;
    row0.side = RowSide::RS;
    row0.stitches = {instruction::CastOn{2}};
    pattern.rows.push_back(row0);

    RowInstruction row1;
    row1.side = RowSide::RS;
    instruction::Repeat rep;
    rep.instructions = {instruction::Knit{}};
    rep.times = 2;
    row1.stitches = {rep};
    pattern.rows.push_back(row1);

    auto graph = StitchGraph::from_instructions(pattern);
    auto yarn_path = YarnPath::from_stitch_graph(graph, default_yarn(), default_gauge());

    // Get through for each segment
    for (size_t i = 0; i < yarn_path.segments().size(); ++i) {
        SegmentId id = static_cast<SegmentId>(i);
        const auto* through = yarn_path.get_through(id);
        ASSERT_NE(through, nullptr);
        EXPECT_EQ(*through, yarn_path.segments()[i].through);
    }

    // Out of bounds returns nullptr
    EXPECT_EQ(yarn_path.get_through(1000), nullptr);
}

TEST(YarnPath, ToDot) {
    PatternInstructions pattern;

    RowInstruction row0;
    row0.side = RowSide::RS;
    row0.stitches = {instruction::CastOn{2}};
    pattern.rows.push_back(row0);

    RowInstruction row1;
    row1.side = RowSide::RS;
    instruction::Repeat rep;
    rep.instructions = {instruction::Knit{}};
    rep.times = 2;
    row1.stitches = {rep};
    pattern.rows.push_back(row1);

    auto graph = StitchGraph::from_instructions(pattern);
    auto yarn_path = YarnPath::from_stitch_graph(graph, default_yarn(), default_gauge());

    std::string dot = yarn_path.to_dot();

    // Should contain basic DOT structure
    EXPECT_NE(dot.find("digraph"), std::string::npos);
    EXPECT_NE(dot.find("YarnPath"), std::string::npos);

    // Should have segment nodes
    EXPECT_NE(dot.find("seg"), std::string::npos);
}
