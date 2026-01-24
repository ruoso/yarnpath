#include "yarn_path.hpp"
#include "stitch_instruction.hpp"
#include "row_instruction.hpp"
#include <gtest/gtest.h>
#include <algorithm>

using namespace yarnpath;

TEST(YarnPath, CastOnOnly) {
    // Cast on 3 stitches
    PatternInstructions pattern;

    RowInstruction row0;
    row0.side = RowSide::RS;
    row0.stitches = {instruction::CastOn{3}};
    pattern.rows.push_back(row0);

    auto graph = StitchGraph::from_instructions(pattern);
    auto yarn_path = YarnPath::from_stitch_graph(graph);

    // Should have 3 loops (one per cast-on)
    EXPECT_EQ(yarn_path.loops().size(), 3u);

    // Verify all loops are cast-on
    for (const auto& loop : yarn_path.loops()) {
        EXPECT_EQ(loop.kind, FormKind::CastOn);
        EXPECT_TRUE(loop.parent_loops.empty());
    }

    // Verify prev/next chain
    const Loop* loop0 = yarn_path.get_loop(0);
    const Loop* loop1 = yarn_path.get_loop(1);
    const Loop* loop2 = yarn_path.get_loop(2);

    ASSERT_NE(loop0, nullptr);
    ASSERT_NE(loop1, nullptr);
    ASSERT_NE(loop2, nullptr);

    EXPECT_FALSE(loop0->prev_in_yarn.has_value());
    EXPECT_EQ(loop0->next_in_yarn, 1u);

    EXPECT_EQ(loop1->prev_in_yarn, 0u);
    EXPECT_EQ(loop1->next_in_yarn, 2u);

    EXPECT_EQ(loop2->prev_in_yarn, 1u);
    EXPECT_FALSE(loop2->next_in_yarn.has_value());

    // Verify first and last
    EXPECT_EQ(yarn_path.first_loop(), 0u);
    EXPECT_EQ(yarn_path.last_loop(), 2u);

    // Verify anchors exist for each loop
    for (const auto& loop : yarn_path.loops()) {
        auto anchors = yarn_path.anchors_for_stitch(loop.stitch_id);
        EXPECT_GE(anchors.size(), 2u);  // At least CastOnBase and LoopApex
    }

    // Verify segments exist
    EXPECT_GT(yarn_path.segment_count(), 0u);
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
    auto yarn_path = YarnPath::from_stitch_graph(graph);

    // Should have 6 loops: 3 cast-on + 3 knit
    EXPECT_EQ(yarn_path.loops().size(), 6u);

    // Verify cast-on loops
    for (LoopId i = 0; i < 3; ++i) {
        const Loop* loop = yarn_path.get_loop(i);
        ASSERT_NE(loop, nullptr);
        EXPECT_EQ(loop->kind, FormKind::CastOn);
        EXPECT_TRUE(loop->parent_loops.empty());
    }

    // Verify knit loops
    for (LoopId i = 3; i < 6; ++i) {
        const Loop* loop = yarn_path.get_loop(i);
        ASSERT_NE(loop, nullptr);
        EXPECT_EQ(loop->kind, FormKind::Knit);
        EXPECT_EQ(loop->parent_loops.size(), 1u);
    }

    // Verify parent-child relationships
    // Knit loop 3 should have cast-on loop 0 as parent
    const Loop* knit0 = yarn_path.get_loop(3);
    ASSERT_NE(knit0, nullptr);
    EXPECT_EQ(knit0->parent_loops[0], 0u);

    // Cast-on loop 0 should have knit loop 3 as child
    const Loop* cast0 = yarn_path.get_loop(0);
    ASSERT_NE(cast0, nullptr);
    EXPECT_EQ(cast0->child_loops.size(), 1u);
    EXPECT_EQ(cast0->child_loops[0], 3u);

    // Verify prev/next chain is continuous
    LoopId current = yarn_path.first_loop();
    size_t count = 0;
    while (count < yarn_path.loops().size()) {
        const Loop* loop = yarn_path.get_loop(current);
        ASSERT_NE(loop, nullptr);
        count++;
        if (!loop->next_in_yarn.has_value()) break;
        current = loop->next_in_yarn.value();
    }
    EXPECT_EQ(count, yarn_path.loops().size());
}

TEST(YarnPath, StockinetteMultiRow) {
    // Cast on 3, then 2 rows (knit RS, purl WS)
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

    auto graph = StitchGraph::from_instructions(pattern);
    auto yarn_path = YarnPath::from_stitch_graph(graph);

    // 3 + 3 + 3 = 9 loops
    EXPECT_EQ(yarn_path.loops().size(), 9u);

    // Verify purl loops have correct kind
    for (LoopId i = 6; i < 9; ++i) {
        const Loop* loop = yarn_path.get_loop(i);
        ASSERT_NE(loop, nullptr);
        EXPECT_EQ(loop->kind, FormKind::Purl);
    }

    // Verify full yarn chain
    LoopId current = yarn_path.first_loop();
    size_t count = 0;
    while (count < 100) {  // Safety limit
        const Loop* loop = yarn_path.get_loop(current);
        ASSERT_NE(loop, nullptr);
        count++;
        if (!loop->next_in_yarn.has_value()) break;
        current = loop->next_in_yarn.value();
    }
    EXPECT_EQ(count, 9u);
}

TEST(YarnPath, YarnOverIncrease) {
    // Cast on 2, row 1: K YO K
    PatternInstructions pattern;

    RowInstruction row0;
    row0.side = RowSide::RS;
    row0.stitches = {instruction::CastOn{2}};
    pattern.rows.push_back(row0);

    RowInstruction row1;
    row1.side = RowSide::RS;
    row1.stitches = {instruction::Knit{}, instruction::YarnOver{}, instruction::Knit{}};
    pattern.rows.push_back(row1);

    auto graph = StitchGraph::from_instructions(pattern);
    auto yarn_path = YarnPath::from_stitch_graph(graph);

    // 2 + 3 = 5 loops
    EXPECT_EQ(yarn_path.loops().size(), 5u);

    // Find the YO loop
    const Loop* yo_loop = nullptr;
    for (const auto& loop : yarn_path.loops()) {
        if (loop.kind == FormKind::YarnOver) {
            yo_loop = &loop;
            break;
        }
    }
    ASSERT_NE(yo_loop, nullptr);

    // YO has no parents
    EXPECT_TRUE(yo_loop->parent_loops.empty());

    // YO should have a YarnOverApex anchor
    bool found_yo_apex = false;
    for (const auto& anchor_node : yarn_path.anchors()) {
        if (std::holds_alternative<anchor::YarnOverApex>(anchor_node.anchor)) {
            found_yo_apex = true;
            break;
        }
    }
    EXPECT_TRUE(found_yo_apex);

    // Check for Wrap segment
    bool found_wrap = false;
    for (const auto& seg : yarn_path.segments()) {
        if (std::holds_alternative<segment::Wrap>(seg.segment_type)) {
            found_wrap = true;
            break;
        }
    }
    EXPECT_TRUE(found_wrap);
}

TEST(YarnPath, KfbIncrease) {
    // Cast on 2, row 1: KFB K
    // In StitchGraph, KFB is expanded into 2 stitch nodes (each with same parent)
    PatternInstructions pattern;

    RowInstruction row0;
    row0.side = RowSide::RS;
    row0.stitches = {instruction::CastOn{2}};
    pattern.rows.push_back(row0);

    RowInstruction row1;
    row1.side = RowSide::RS;
    row1.stitches = {instruction::KFB{}, instruction::Knit{}};
    pattern.rows.push_back(row1);

    auto graph = StitchGraph::from_instructions(pattern);
    auto yarn_path = YarnPath::from_stitch_graph(graph);

    // StitchGraph creates 2 cast-on + 3 row-1 stitches = 5 stitches
    // Each stitch creates 1 loop = 5 loops
    EXPECT_EQ(yarn_path.loops().size(), 5u);

    // Find KFB loops (2 stitch nodes, each creates 1 loop)
    std::vector<const Loop*> kfb_loops;
    for (const auto& loop : yarn_path.loops()) {
        if (loop.kind == FormKind::KFB) {
            kfb_loops.push_back(&loop);
        }
    }
    EXPECT_EQ(kfb_loops.size(), 2u);

    // Both KFB loops should have the same parent (cast-on 0)
    ASSERT_EQ(kfb_loops[0]->parent_loops.size(), 1u);
    ASSERT_EQ(kfb_loops[1]->parent_loops.size(), 1u);
    EXPECT_EQ(kfb_loops[0]->parent_loops[0], kfb_loops[1]->parent_loops[0]);

    // The parent should have 2 children
    const Loop* parent = yarn_path.get_loop(kfb_loops[0]->parent_loops[0]);
    ASSERT_NE(parent, nullptr);
    EXPECT_EQ(parent->child_loops.size(), 2u);
}

TEST(YarnPath, M1LIncrease) {
    // Cast on 2, row 1: K M1L K
    PatternInstructions pattern;

    RowInstruction row0;
    row0.side = RowSide::RS;
    row0.stitches = {instruction::CastOn{2}};
    pattern.rows.push_back(row0);

    RowInstruction row1;
    row1.side = RowSide::RS;
    row1.stitches = {instruction::Knit{}, instruction::M1L{}, instruction::Knit{}};
    pattern.rows.push_back(row1);

    auto graph = StitchGraph::from_instructions(pattern);
    auto yarn_path = YarnPath::from_stitch_graph(graph);

    // 2 + 3 = 5 loops
    EXPECT_EQ(yarn_path.loops().size(), 5u);

    // Find the M1L loop
    const Loop* m1l_loop = nullptr;
    for (const auto& loop : yarn_path.loops()) {
        if (loop.kind == FormKind::M1L) {
            m1l_loop = &loop;
            break;
        }
    }
    ASSERT_NE(m1l_loop, nullptr);

    // M1L has no parents and is a lifted bar
    EXPECT_TRUE(m1l_loop->parent_loops.empty());
    EXPECT_TRUE(m1l_loop->is_lifted_bar);
}

TEST(YarnPath, M1RIncrease) {
    // Cast on 2, row 1: K M1R K
    PatternInstructions pattern;

    RowInstruction row0;
    row0.side = RowSide::RS;
    row0.stitches = {instruction::CastOn{2}};
    pattern.rows.push_back(row0);

    RowInstruction row1;
    row1.side = RowSide::RS;
    row1.stitches = {instruction::Knit{}, instruction::M1R{}, instruction::Knit{}};
    pattern.rows.push_back(row1);

    auto graph = StitchGraph::from_instructions(pattern);
    auto yarn_path = YarnPath::from_stitch_graph(graph);

    // Find the M1R loop
    const Loop* m1r_loop = nullptr;
    for (const auto& loop : yarn_path.loops()) {
        if (loop.kind == FormKind::M1R) {
            m1r_loop = &loop;
            break;
        }
    }
    ASSERT_NE(m1r_loop, nullptr);
    EXPECT_TRUE(m1r_loop->is_lifted_bar);
}

TEST(YarnPath, K2togDecrease) {
    // Cast on 4, row 1: K K2tog K
    PatternInstructions pattern;

    RowInstruction row0;
    row0.side = RowSide::RS;
    row0.stitches = {instruction::CastOn{4}};
    pattern.rows.push_back(row0);

    RowInstruction row1;
    row1.side = RowSide::RS;
    row1.stitches = {instruction::Knit{}, instruction::K2tog{}, instruction::Knit{}};
    pattern.rows.push_back(row1);

    auto graph = StitchGraph::from_instructions(pattern);
    auto yarn_path = YarnPath::from_stitch_graph(graph);

    // 4 + 3 = 7 loops
    EXPECT_EQ(yarn_path.loops().size(), 7u);

    // Find the K2tog loop
    const Loop* k2tog_loop = nullptr;
    for (const auto& loop : yarn_path.loops()) {
        if (loop.kind == FormKind::K2tog) {
            k2tog_loop = &loop;
            break;
        }
    }
    ASSERT_NE(k2tog_loop, nullptr);

    // K2tog has 2 parents
    EXPECT_EQ(k2tog_loop->parent_loops.size(), 2u);

    // Both parents should have K2tog as child
    for (LoopId parent_id : k2tog_loop->parent_loops) {
        const Loop* parent = yarn_path.get_loop(parent_id);
        ASSERT_NE(parent, nullptr);
        EXPECT_TRUE(std::find(parent->child_loops.begin(), parent->child_loops.end(),
                              k2tog_loop->id) != parent->child_loops.end());
    }
}

TEST(YarnPath, SskDecrease) {
    // Cast on 4, row 1: K SSK K
    PatternInstructions pattern;

    RowInstruction row0;
    row0.side = RowSide::RS;
    row0.stitches = {instruction::CastOn{4}};
    pattern.rows.push_back(row0);

    RowInstruction row1;
    row1.side = RowSide::RS;
    row1.stitches = {instruction::Knit{}, instruction::SSK{}, instruction::Knit{}};
    pattern.rows.push_back(row1);

    auto graph = StitchGraph::from_instructions(pattern);
    auto yarn_path = YarnPath::from_stitch_graph(graph);

    // Find the SSK loop
    const Loop* ssk_loop = nullptr;
    for (const auto& loop : yarn_path.loops()) {
        if (loop.kind == FormKind::SSK) {
            ssk_loop = &loop;
            break;
        }
    }
    ASSERT_NE(ssk_loop, nullptr);
    EXPECT_EQ(ssk_loop->parent_loops.size(), 2u);
}

TEST(YarnPath, S2kpDecrease) {
    // Cast on 5, row 1: K S2KP K
    PatternInstructions pattern;

    RowInstruction row0;
    row0.side = RowSide::RS;
    row0.stitches = {instruction::CastOn{5}};
    pattern.rows.push_back(row0);

    RowInstruction row1;
    row1.side = RowSide::RS;
    row1.stitches = {instruction::Knit{}, instruction::S2KP{}, instruction::Knit{}};
    pattern.rows.push_back(row1);

    auto graph = StitchGraph::from_instructions(pattern);
    auto yarn_path = YarnPath::from_stitch_graph(graph);

    // Find the S2KP loop
    const Loop* s2kp_loop = nullptr;
    for (const auto& loop : yarn_path.loops()) {
        if (loop.kind == FormKind::S2KP) {
            s2kp_loop = &loop;
            break;
        }
    }
    ASSERT_NE(s2kp_loop, nullptr);

    // S2KP has 3 parents
    EXPECT_EQ(s2kp_loop->parent_loops.size(), 3u);

    // All 3 parents should have this loop as child
    for (LoopId parent_id : s2kp_loop->parent_loops) {
        const Loop* parent = yarn_path.get_loop(parent_id);
        ASSERT_NE(parent, nullptr);
        EXPECT_TRUE(std::find(parent->child_loops.begin(), parent->child_loops.end(),
                              s2kp_loop->id) != parent->child_loops.end());
    }
}

TEST(YarnPath, SlipStitch) {
    // Cast on 3, row 1: K Slip K
    PatternInstructions pattern;

    RowInstruction row0;
    row0.side = RowSide::RS;
    row0.stitches = {instruction::CastOn{3}};
    pattern.rows.push_back(row0);

    RowInstruction row1;
    row1.side = RowSide::RS;
    row1.stitches = {instruction::Knit{}, instruction::Slip{}, instruction::Knit{}};
    pattern.rows.push_back(row1);

    auto graph = StitchGraph::from_instructions(pattern);
    auto yarn_path = YarnPath::from_stitch_graph(graph);

    // 3 + 3 = 6 loops
    EXPECT_EQ(yarn_path.loops().size(), 6u);

    // Find the Slip loop
    const Loop* slip_loop = nullptr;
    for (const auto& loop : yarn_path.loops()) {
        if (loop.kind == FormKind::Slip) {
            slip_loop = &loop;
            break;
        }
    }
    ASSERT_NE(slip_loop, nullptr);

    // Slip has 1 parent
    EXPECT_EQ(slip_loop->parent_loops.size(), 1u);
}

TEST(YarnPath, BindOff) {
    // Cast on 3, row 1: K K K, row 2: BindOff{3}
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

    RowInstruction row2;
    row2.side = RowSide::WS;
    row2.stitches = {instruction::BindOff{3}};
    pattern.rows.push_back(row2);

    auto graph = StitchGraph::from_instructions(pattern);
    auto yarn_path = YarnPath::from_stitch_graph(graph);

    // 3 + 3 + 3 = 9 loops
    EXPECT_EQ(yarn_path.loops().size(), 9u);

    // Find bind-off loops
    std::vector<const Loop*> bind_off_loops;
    for (const auto& loop : yarn_path.loops()) {
        if (loop.kind == FormKind::BindOff) {
            bind_off_loops.push_back(&loop);
        }
    }
    EXPECT_EQ(bind_off_loops.size(), 3u);

    // Bind-off loops should be marked
    for (const auto* loop : bind_off_loops) {
        EXPECT_TRUE(loop->is_bound_off);
        EXPECT_EQ(loop->parent_loops.size(), 1u);
    }

    // Check for BindOffEnd anchor
    bool found_bind_off_end = false;
    for (const auto& anchor_node : yarn_path.anchors()) {
        if (std::holds_alternative<anchor::BindOffEnd>(anchor_node.anchor)) {
            found_bind_off_end = true;
            break;
        }
    }
    EXPECT_TRUE(found_bind_off_end);
}

TEST(YarnPath, StitchToLoopMapping) {
    // Cast on 2, then KFB K (3 loops from 2 stitches)
    PatternInstructions pattern;

    RowInstruction row0;
    row0.side = RowSide::RS;
    row0.stitches = {instruction::CastOn{2}};
    pattern.rows.push_back(row0);

    RowInstruction row1;
    row1.side = RowSide::RS;
    row1.stitches = {instruction::KFB{}, instruction::Knit{}};
    pattern.rows.push_back(row1);

    auto graph = StitchGraph::from_instructions(pattern);
    auto yarn_path = YarnPath::from_stitch_graph(graph);

    // KFB stitch (stitch 2 in row 1) creates 2 loops
    // Actually in graph: stitch IDs 0,1 are cast-on, 2,3,4 are row 1
    // Stitch 2 and 3 are KFB (same parent), stitch 4 is K

    // Verify stitch_to_loops
    for (const auto& loop : yarn_path.loops()) {
        const auto& loops_for_stitch = yarn_path.stitch_to_loops(loop.stitch_id);
        EXPECT_TRUE(std::find(loops_for_stitch.begin(), loops_for_stitch.end(),
                              loop.id) != loops_for_stitch.end());
    }

    // Verify loop_to_stitch
    for (const auto& loop : yarn_path.loops()) {
        EXPECT_EQ(yarn_path.loop_to_stitch(loop.id), loop.stitch_id);
    }
}

TEST(YarnPath, SegmentConnectivity) {
    // Cast on 2, K K
    PatternInstructions pattern;

    RowInstruction row0;
    row0.side = RowSide::RS;
    row0.stitches = {instruction::CastOn{2}};
    pattern.rows.push_back(row0);

    RowInstruction row1;
    row1.side = RowSide::RS;
    row1.stitches = {instruction::Knit{}, instruction::Knit{}};
    pattern.rows.push_back(row1);

    auto graph = StitchGraph::from_instructions(pattern);
    auto yarn_path = YarnPath::from_stitch_graph(graph);

    // Verify segments reference valid anchors
    for (const auto& seg : yarn_path.segments()) {
        const auto* from = yarn_path.get_anchor(seg.from_anchor);
        const auto* to = yarn_path.get_anchor(seg.to_anchor);
        ASSERT_NE(from, nullptr);
        ASSERT_NE(to, nullptr);
    }

    // Verify ThroughLoop segments exist for knit stitches
    bool found_through_loop = false;
    for (const auto& seg : yarn_path.segments()) {
        if (std::holds_alternative<segment::ThroughLoop>(seg.segment_type)) {
            found_through_loop = true;
            auto through = std::get<segment::ThroughLoop>(seg.segment_type);
            EXPECT_TRUE(through.mode == PassMode::KnitWise ||
                        through.mode == PassMode::PurlWise);
            break;
        }
    }
    EXPECT_TRUE(found_through_loop);
}

TEST(YarnPath, LoopFormAnchors) {
    // Cast on 2, K K
    PatternInstructions pattern;

    RowInstruction row0;
    row0.side = RowSide::RS;
    row0.stitches = {instruction::CastOn{2}};
    pattern.rows.push_back(row0);

    RowInstruction row1;
    row1.side = RowSide::RS;
    row1.stitches = {instruction::Knit{}, instruction::Knit{}};
    pattern.rows.push_back(row1);

    auto graph = StitchGraph::from_instructions(pattern);
    auto yarn_path = YarnPath::from_stitch_graph(graph);

    // Every loop should have a LoopForm anchor
    for (const auto& loop : yarn_path.loops()) {
        const auto* form_anchor = yarn_path.get_anchor(loop.form_anchor);
        ASSERT_NE(form_anchor, nullptr);

        // Verify it's a LoopForm
        EXPECT_TRUE(std::holds_alternative<anchor::LoopForm>(form_anchor->anchor));

        if (std::holds_alternative<anchor::LoopForm>(form_anchor->anchor)) {
            auto form = std::get<anchor::LoopForm>(form_anchor->anchor);
            EXPECT_EQ(form.loop_id, loop.id);
            EXPECT_EQ(form.kind, loop.kind);
            EXPECT_EQ(form.parents.size(), loop.parent_loops.size());
        }
    }
}

TEST(YarnPath, EmptyGraph) {
    PatternInstructions pattern;
    auto graph = StitchGraph::from_instructions(pattern);
    auto yarn_path = YarnPath::from_stitch_graph(graph);

    EXPECT_EQ(yarn_path.loops().size(), 0u);
    EXPECT_EQ(yarn_path.anchors().size(), 0u);
    EXPECT_EQ(yarn_path.segment_count(), 0u);
}

TEST(YarnPath, GetInvalidLoop) {
    PatternInstructions pattern;
    RowInstruction row0;
    row0.side = RowSide::RS;
    row0.stitches = {instruction::CastOn{2}};
    pattern.rows.push_back(row0);

    auto graph = StitchGraph::from_instructions(pattern);
    auto yarn_path = YarnPath::from_stitch_graph(graph);

    const auto* invalid = yarn_path.get_loop(100);
    EXPECT_EQ(invalid, nullptr);
}

TEST(YarnPath, GetInvalidAnchor) {
    PatternInstructions pattern;
    RowInstruction row0;
    row0.side = RowSide::RS;
    row0.stitches = {instruction::CastOn{2}};
    pattern.rows.push_back(row0);

    auto graph = StitchGraph::from_instructions(pattern);
    auto yarn_path = YarnPath::from_stitch_graph(graph);

    const auto* invalid = yarn_path.get_anchor(1000);
    EXPECT_EQ(invalid, nullptr);
}

TEST(YarnPath, GetInvalidSegment) {
    PatternInstructions pattern;
    RowInstruction row0;
    row0.side = RowSide::RS;
    row0.stitches = {instruction::CastOn{2}};
    pattern.rows.push_back(row0);

    auto graph = StitchGraph::from_instructions(pattern);
    auto yarn_path = YarnPath::from_stitch_graph(graph);

    const auto* invalid = yarn_path.get_segment(1000);
    EXPECT_EQ(invalid, nullptr);
}

TEST(YarnPath, CableLeft) {
    // Cast on 6, row 1: K CableLeft{2,2} K
    PatternInstructions pattern;

    RowInstruction row0;
    row0.side = RowSide::RS;
    row0.stitches = {instruction::CastOn{6}};
    pattern.rows.push_back(row0);

    RowInstruction row1;
    row1.side = RowSide::RS;
    row1.stitches = {instruction::Knit{}, instruction::CableLeft{2, 2}, instruction::Knit{}};
    pattern.rows.push_back(row1);

    auto graph = StitchGraph::from_instructions(pattern);
    auto yarn_path = YarnPath::from_stitch_graph(graph);

    // 6 + 6 = 12 loops
    EXPECT_EQ(yarn_path.loops().size(), 12u);

    // All loops in row 1 should be knit (cables are processed as knit currently)
    for (LoopId i = 6; i < 12; ++i) {
        const Loop* loop = yarn_path.get_loop(i);
        ASSERT_NE(loop, nullptr);
        EXPECT_EQ(loop->kind, FormKind::Knit);
    }
}

TEST(YarnPath, CableRight) {
    // Cast on 6, row 1: K CableRight{2,2} K
    PatternInstructions pattern;

    RowInstruction row0;
    row0.side = RowSide::RS;
    row0.stitches = {instruction::CastOn{6}};
    pattern.rows.push_back(row0);

    RowInstruction row1;
    row1.side = RowSide::RS;
    row1.stitches = {instruction::Knit{}, instruction::CableRight{2, 2}, instruction::Knit{}};
    pattern.rows.push_back(row1);

    auto graph = StitchGraph::from_instructions(pattern);
    auto yarn_path = YarnPath::from_stitch_graph(graph);

    // 6 + 6 = 12 loops
    EXPECT_EQ(yarn_path.loops().size(), 12u);

    // Verify parent relationships match the cable crossing
    // Cable stitches should have correct parent-child relationships
    for (LoopId i = 6; i < 12; ++i) {
        const Loop* loop = yarn_path.get_loop(i);
        ASSERT_NE(loop, nullptr);
        EXPECT_EQ(loop->parent_loops.size(), 1u);
    }
}

TEST(YarnPath, ComplexPattern) {
    // A more complex pattern: cast on 6, K YO K2tog K K K
    PatternInstructions pattern;

    RowInstruction row0;
    row0.side = RowSide::RS;
    row0.stitches = {instruction::CastOn{6}};
    pattern.rows.push_back(row0);

    RowInstruction row1;
    row1.side = RowSide::RS;
    row1.stitches = {
        instruction::Knit{},
        instruction::YarnOver{},
        instruction::K2tog{},
        instruction::Knit{},
        instruction::Knit{},
        instruction::Knit{}
    };
    pattern.rows.push_back(row1);

    auto graph = StitchGraph::from_instructions(pattern);
    auto yarn_path = YarnPath::from_stitch_graph(graph);

    // 6 + 6 = 12 loops (YO adds 1, K2tog removes 1, net 0)
    EXPECT_EQ(yarn_path.loops().size(), 12u);

    // Count each kind
    size_t cast_on_count = 0;
    size_t knit_count = 0;
    size_t yo_count = 0;
    size_t k2tog_count = 0;

    for (const auto& loop : yarn_path.loops()) {
        switch (loop.kind) {
            case FormKind::CastOn: cast_on_count++; break;
            case FormKind::Knit: knit_count++; break;
            case FormKind::YarnOver: yo_count++; break;
            case FormKind::K2tog: k2tog_count++; break;
            default: break;
        }
    }

    EXPECT_EQ(cast_on_count, 6u);
    EXPECT_EQ(knit_count, 4u);  // K + K + K + K
    EXPECT_EQ(yo_count, 1u);
    EXPECT_EQ(k2tog_count, 1u);

    // Verify yarn chain is complete
    LoopId current = yarn_path.first_loop();
    size_t count = 0;
    while (count < 100) {
        const Loop* loop = yarn_path.get_loop(current);
        ASSERT_NE(loop, nullptr);
        count++;
        if (!loop->next_in_yarn.has_value()) break;
        current = loop->next_in_yarn.value();
    }
    EXPECT_EQ(count, 12u);
}

TEST(YarnPath, AnchorStitchReference) {
    // Verify that all anchors reference valid stitches
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
    auto yarn_path = YarnPath::from_stitch_graph(graph);

    for (const auto& anchor_node : yarn_path.anchors()) {
        EXPECT_LT(anchor_node.stitch_id, static_cast<StitchId>(graph.size()));
    }
}

TEST(YarnPath, PurlPassMode) {
    // Cast on 2, Purl 2
    PatternInstructions pattern;

    RowInstruction row0;
    row0.side = RowSide::RS;
    row0.stitches = {instruction::CastOn{2}};
    pattern.rows.push_back(row0);

    RowInstruction row1;
    row1.side = RowSide::RS;
    row1.stitches = {instruction::Purl{}, instruction::Purl{}};
    pattern.rows.push_back(row1);

    auto graph = StitchGraph::from_instructions(pattern);
    auto yarn_path = YarnPath::from_stitch_graph(graph);

    // Find ThroughLoop segments with PurlWise mode
    bool found_purl_wise = false;
    for (const auto& seg : yarn_path.segments()) {
        if (std::holds_alternative<segment::ThroughLoop>(seg.segment_type)) {
            auto through = std::get<segment::ThroughLoop>(seg.segment_type);
            if (through.mode == PassMode::PurlWise) {
                found_purl_wise = true;
                break;
            }
        }
    }
    EXPECT_TRUE(found_purl_wise);
}
