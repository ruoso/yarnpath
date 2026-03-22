#include "yarn_path.hpp"
#include "stitch_instruction.hpp"
#include "row_instruction.hpp"
#include "test_helpers.hpp"
#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

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

    // Two-pass cast-on: 3 foundation + 3 return = 6 segments
    size_t foundation_count = 0;
    size_t return_count = 0;
    for (const auto& seg : yarn_path.segments()) {
        if (seg.forms_loop) {
            if (seg.through.empty()) {
                foundation_count++;
            } else {
                return_count++;
                // Return segments each pass through one foundation
                EXPECT_EQ(seg.through.size(), 1u);
            }
        }
    }
    EXPECT_EQ(foundation_count, 3u);
    EXPECT_EQ(return_count, 3u);
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
    size_t foundation_count = 0;
    size_t worked_count = 0;

    for (const auto& seg : yarn_path.segments()) {
        if (seg.forms_loop) {
            loop_count++;
            if (seg.through.empty()) {
                foundation_count++;
            } else {
                worked_count++;
                // All worked loops (return + knit) have exactly one parent
                EXPECT_EQ(seg.through.size(), 1u);
            }
        }
    }

    // 3 foundation + 3 return + 3 knit = 9 loops
    EXPECT_EQ(loop_count, 9u);
    EXPECT_EQ(foundation_count, 3u);
    EXPECT_EQ(worked_count, 6u);  // 3 return + 3 knit
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

    // 3 foundation + 3 return + 3 purl = 9 loops
    EXPECT_EQ(loop_count, 9u);
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

    // 3 foundation + 3 return + 3 knit + 3 purl + 3 knit = 15 loops
    EXPECT_EQ(loop_count, 15u);
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

    ASSERT_EQ(loop_ids.size(), 6u);  // 2 foundation + 2 return + 2 knit

    // First two are foundations (no parents)
    EXPECT_TRUE(yarn_path.segments()[loop_ids[0]].through.empty());
    EXPECT_TRUE(yarn_path.segments()[loop_ids[1]].through.empty());

    // Next two are return segments (each through one foundation)
    const auto& ret1 = yarn_path.segments()[loop_ids[2]];
    const auto& ret2 = yarn_path.segments()[loop_ids[3]];
    EXPECT_EQ(ret1.through.size(), 1u);
    EXPECT_EQ(ret2.through.size(), 1u);

    // Last two are knits (each through one return segment)
    const auto& knit1 = yarn_path.segments()[loop_ids[4]];
    const auto& knit2 = yarn_path.segments()[loop_ids[5]];
    EXPECT_EQ(knit1.through.size(), 1u);
    EXPECT_EQ(knit2.through.size(), 1u);

    // Knit parents should be return segments, not foundations
    EXPECT_TRUE(knit1.through[0] == loop_ids[2] || knit1.through[0] == loop_ids[3]);
    EXPECT_TRUE(knit2.through[0] == loop_ids[2] || knit2.through[0] == loop_ids[3]);
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

    // 3 foundation + 3 return + 3 knit + 3 bind-off = 12 loops
    EXPECT_EQ(loop_count, 12u);
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

    // Should have segment node identifiers (s0, s1, etc.)
    EXPECT_NE(dot.find("s0"), std::string::npos);
    EXPECT_NE(dot.find("s1"), std::string::npos);
}

// === Decrease tests ===

TEST(YarnPath, K2togDecrease) {
    // Cast on 4, RS: K K2tog K
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
    auto yarn_path = YarnPath::from_stitch_graph(graph, default_yarn(), default_gauge());

    // Find worked loop segments (non-foundation) — includes return + user-row
    std::vector<const YarnSegment*> worked;
    for (const auto& seg : yarn_path.segments()) {
        if (seg.forms_loop && !seg.through.empty()) {
            worked.push_back(&seg);
        }
    }

    ASSERT_EQ(worked.size(), 7u);  // 4 return + K, K2tog, K

    // K2tog should have 2 parents (after the 4 return segments)
    EXPECT_EQ(worked[4]->through.size(), 1u);  // K
    EXPECT_EQ(worked[5]->through.size(), 2u);  // K2tog
    EXPECT_EQ(worked[6]->through.size(), 1u);  // K
}

TEST(YarnPath, SSKDecrease) {
    // Cast on 4, RS: K SSK K
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
    auto yarn_path = YarnPath::from_stitch_graph(graph, default_yarn(), default_gauge());

    std::vector<const YarnSegment*> worked;
    for (const auto& seg : yarn_path.segments()) {
        if (seg.forms_loop && !seg.through.empty()) {
            worked.push_back(&seg);
        }
    }

    ASSERT_EQ(worked.size(), 7u);  // 4 return + K, SSK, K
    EXPECT_EQ(worked[5]->through.size(), 2u);  // SSK has 2 parents
}

TEST(YarnPath, S2KPDecrease) {
    // Cast on 5, RS: K S2KP K
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
    auto yarn_path = YarnPath::from_stitch_graph(graph, default_yarn(), default_gauge());

    std::vector<const YarnSegment*> worked;
    for (const auto& seg : yarn_path.segments()) {
        if (seg.forms_loop && !seg.through.empty()) {
            worked.push_back(&seg);
        }
    }

    ASSERT_EQ(worked.size(), 8u);  // 5 return + K, S2KP, K
    EXPECT_EQ(worked[6]->through.size(), 3u);  // S2KP has 3 parents
}

// === Increase tests ===

TEST(YarnPath, YarnOverIncrease) {
    // Cast on 2, RS: K YO K
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
    auto yarn_path = YarnPath::from_stitch_graph(graph, default_yarn(), default_gauge());

    // Find the YO segment: forms_loop, no parents, Created work_type
    bool found_yo = false;
    for (const auto& seg : yarn_path.segments()) {
        if (seg.forms_loop && seg.through.empty() &&
            seg.work_type == WorkType::Created) {
            // Could be cast-on or YO — distinguish by checking orientation or position
            // YO and CastOn both have Neutral orientation and Created work_type,
            // but we can count: 2 cast-on + 1 YO = 3 created segments
            found_yo = true;
        }
    }
    EXPECT_TRUE(found_yo);

    // Count total loops: 2 foundation + 2 return + 2 knit + 1 YO = 7
    size_t loop_count = 0;
    for (const auto& seg : yarn_path.segments()) {
        if (seg.forms_loop) loop_count++;
    }
    EXPECT_EQ(loop_count, 7u);
}

TEST(YarnPath, KFBIncrease) {
    // Cast on 1, RS: KFB
    PatternInstructions pattern;

    RowInstruction row0;
    row0.side = RowSide::RS;
    row0.stitches = {instruction::CastOn{1}};
    pattern.rows.push_back(row0);

    RowInstruction row1;
    row1.side = RowSide::RS;
    row1.stitches = {instruction::KFB{}};
    pattern.rows.push_back(row1);

    auto graph = StitchGraph::from_instructions(pattern);
    auto yarn_path = YarnPath::from_stitch_graph(graph, default_yarn(), default_gauge());

    // KFB consumes 1, produces 2 — worked segments include 1 return + 2 KFB
    std::vector<const YarnSegment*> worked;
    for (const auto& seg : yarn_path.segments()) {
        if (seg.forms_loop && seg.work_type != WorkType::Created) {
            worked.push_back(&seg);
        }
    }

    ASSERT_EQ(worked.size(), 3u);  // 1 return + 2 KFB
    // KFB segments both reference the same parent (the return segment)
    EXPECT_EQ(worked[1]->through.size(), 1u);
    EXPECT_EQ(worked[2]->through.size(), 1u);
    EXPECT_EQ(worked[1]->through[0], worked[2]->through[0]);
}

TEST(YarnPath, M1LIncrease) {
    // Cast on 2, RS: K M1L K
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
    auto yarn_path = YarnPath::from_stitch_graph(graph, default_yarn(), default_gauge());

    // Find M1L segment: Created work_type, no parents, CounterClockwise wrap
    bool found_m1l = false;
    for (const auto& seg : yarn_path.segments()) {
        if (seg.work_type == WorkType::Created &&
            seg.wrap_direction == WrapDirection::CounterClockwise) {
            found_m1l = true;
            EXPECT_TRUE(seg.through.empty());
        }
    }
    EXPECT_TRUE(found_m1l);

    // Total loops: 2 foundation + 2 return + 2 knit + 1 M1L = 7
    size_t loop_count = 0;
    for (const auto& seg : yarn_path.segments()) {
        if (seg.forms_loop) loop_count++;
    }
    EXPECT_EQ(loop_count, 7u);
}

TEST(YarnPath, M1RIncrease) {
    // Cast on 2, RS: K M1R K
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
    auto yarn_path = YarnPath::from_stitch_graph(graph, default_yarn(), default_gauge());

    // Find M1R segment: Created work_type, Clockwise wrap
    bool found_m1r = false;
    for (const auto& seg : yarn_path.segments()) {
        if (seg.work_type == WorkType::Created &&
            seg.wrap_direction == WrapDirection::Clockwise) {
            found_m1r = true;
            EXPECT_TRUE(seg.through.empty());
        }
    }
    EXPECT_TRUE(found_m1r);
}

// === Cable tests ===

TEST(YarnPath, CableLeftCrossing) {
    // Cast on 6, RS: K CableLeft{2,2} K
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
    auto yarn_path = YarnPath::from_stitch_graph(graph, default_yarn(), default_gauge());

    // 6 return + 6 user-row worked = 12 segments with parents
    std::vector<const YarnSegment*> worked;
    for (const auto& seg : yarn_path.segments()) {
        if (seg.forms_loop && !seg.through.empty()) {
            worked.push_back(&seg);
        }
    }

    ASSERT_EQ(worked.size(), 12u);  // 6 return + 1 K + 4 cable + 1 K

    // User-row starts at index 6 (after return segments)
    // Each cable segment should have exactly 1 parent
    for (size_t i = 7; i <= 10; ++i) {
        EXPECT_EQ(worked[i]->through.size(), 1u);
    }

    // Cable segments should have reordered parents (all different return segments)
    std::vector<SegmentId> cable_parents;
    for (size_t i = 7; i <= 10; ++i) {
        cable_parents.push_back(worked[i]->through[0]);
    }
    // All parents should be unique
    std::sort(cable_parents.begin(), cable_parents.end());
    EXPECT_EQ(std::unique(cable_parents.begin(), cable_parents.end()), cable_parents.end());
}

// === Orientation metadata tests ===

TEST(YarnPath, RSKnitOrientation) {
    // Knit on RS should have Front orientation
    PatternInstructions pattern;

    RowInstruction row0;
    row0.side = RowSide::RS;
    row0.stitches = {instruction::CastOn{1}};
    pattern.rows.push_back(row0);

    RowInstruction row1;
    row1.side = RowSide::RS;
    row1.stitches = {instruction::Knit{}};
    pattern.rows.push_back(row1);

    auto graph = StitchGraph::from_instructions(pattern);
    auto yarn_path = YarnPath::from_stitch_graph(graph, default_yarn(), default_gauge());

    // Find the knit segment (skip return segments whose parent has empty through)
    for (const auto& seg : yarn_path.segments()) {
        if (seg.forms_loop && !seg.through.empty() &&
            !yarn_path.segments()[seg.through[0]].through.empty()) {
            EXPECT_EQ(seg.orientation, LoopOrientation::Front);
        }
    }
}

TEST(YarnPath, WSPurlOrientation) {
    // Purl on WS: instruction orientation is Back, WS flips it to Front
    PatternInstructions pattern;

    RowInstruction row0;
    row0.side = RowSide::RS;
    row0.stitches = {instruction::CastOn{1}};
    pattern.rows.push_back(row0);

    RowInstruction row1;
    row1.side = RowSide::WS;
    row1.stitches = {instruction::Purl{}};
    pattern.rows.push_back(row1);

    auto graph = StitchGraph::from_instructions(pattern);
    auto yarn_path = YarnPath::from_stitch_graph(graph, default_yarn(), default_gauge());

    for (const auto& seg : yarn_path.segments()) {
        if (seg.forms_loop && !seg.through.empty() &&
            !yarn_path.segments()[seg.through[0]].through.empty()) {
            // Purl instruction = Back, WS flips Back→Front
            EXPECT_EQ(seg.orientation, LoopOrientation::Front);
        }
    }
}

TEST(YarnPath, WSKnitOrientation) {
    // Knit on WS: instruction orientation is Front, WS flips it to Back
    PatternInstructions pattern;

    RowInstruction row0;
    row0.side = RowSide::RS;
    row0.stitches = {instruction::CastOn{1}};
    pattern.rows.push_back(row0);

    RowInstruction row1;
    row1.side = RowSide::WS;
    row1.stitches = {instruction::Knit{}};
    pattern.rows.push_back(row1);

    auto graph = StitchGraph::from_instructions(pattern);
    auto yarn_path = YarnPath::from_stitch_graph(graph, default_yarn(), default_gauge());

    for (const auto& seg : yarn_path.segments()) {
        if (seg.forms_loop && !seg.through.empty() &&
            !yarn_path.segments()[seg.through[0]].through.empty()) {
            // Knit instruction = Front, WS flips Front→Back
            EXPECT_EQ(seg.orientation, LoopOrientation::Back);
        }
    }
}

TEST(YarnPath, DecreaseWrapDirection) {
    // K2tog → Clockwise, SSK → CounterClockwise
    PatternInstructions pattern;

    RowInstruction row0;
    row0.side = RowSide::RS;
    row0.stitches = {instruction::CastOn{4}};
    pattern.rows.push_back(row0);

    RowInstruction row1;
    row1.side = RowSide::RS;
    row1.stitches = {instruction::K2tog{}, instruction::SSK{}};
    pattern.rows.push_back(row1);

    auto graph = StitchGraph::from_instructions(pattern);
    auto yarn_path = YarnPath::from_stitch_graph(graph, default_yarn(), default_gauge());

    std::vector<const YarnSegment*> worked;
    for (const auto& seg : yarn_path.segments()) {
        if (seg.forms_loop && !seg.through.empty()) {
            worked.push_back(&seg);
        }
    }

    ASSERT_EQ(worked.size(), 6u);  // 4 return + K2tog + SSK
    EXPECT_EQ(worked[4]->wrap_direction, WrapDirection::Clockwise);       // K2tog
    EXPECT_EQ(worked[5]->wrap_direction, WrapDirection::CounterClockwise); // SSK
}

TEST(YarnPath, SlipWorkType) {
    // Slip should have Transferred work_type
    PatternInstructions pattern;

    RowInstruction row0;
    row0.side = RowSide::RS;
    row0.stitches = {instruction::CastOn{1}};
    pattern.rows.push_back(row0);

    RowInstruction row1;
    row1.side = RowSide::RS;
    row1.stitches = {instruction::Slip{}};
    pattern.rows.push_back(row1);

    auto graph = StitchGraph::from_instructions(pattern);
    auto yarn_path = YarnPath::from_stitch_graph(graph, default_yarn(), default_gauge());

    // Check only user-row segments (skip return segments whose parent has empty through)
    for (const auto& seg : yarn_path.segments()) {
        if (seg.forms_loop && !seg.through.empty() &&
            !yarn_path.segments()[seg.through[0]].through.empty()) {
            EXPECT_EQ(seg.work_type, WorkType::Transferred);
            EXPECT_EQ(seg.orientation, LoopOrientation::Neutral);
        }
    }
}

// === Yarn length tests ===

TEST(YarnPath, KnitVsPurlYarnLength) {
    // Purl should use ~0.88x the yarn of a knit
    PatternInstructions pattern;

    RowInstruction row0;
    row0.side = RowSide::RS;
    row0.stitches = {instruction::CastOn{2}};
    pattern.rows.push_back(row0);

    RowInstruction row1;
    row1.side = RowSide::RS;
    row1.stitches = {instruction::Knit{}, instruction::Purl{}};
    pattern.rows.push_back(row1);

    auto graph = StitchGraph::from_instructions(pattern);
    auto yarn_path = YarnPath::from_stitch_graph(graph, default_yarn(), default_gauge());

    // Skip return segments (parent has empty through)
    std::vector<const YarnSegment*> user_worked;
    for (const auto& seg : yarn_path.segments()) {
        if (seg.forms_loop && !seg.through.empty() &&
            !yarn_path.segments()[seg.through[0]].through.empty()) {
            user_worked.push_back(&seg);
        }
    }

    ASSERT_EQ(user_worked.size(), 2u);
    float knit_len = user_worked[0]->target_yarn_length;
    float purl_len = user_worked[1]->target_yarn_length;

    EXPECT_GT(knit_len, 0.0f);
    EXPECT_GT(purl_len, 0.0f);
    // Purl should be ~0.88x knit
    float ratio = purl_len / knit_len;
    EXPECT_NEAR(ratio, 0.88f, 0.01f);
}

TEST(YarnPath, SlipYarnLength) {
    // Slip should be much shorter: 0.5 * stitch_width
    PatternInstructions pattern;

    RowInstruction row0;
    row0.side = RowSide::RS;
    row0.stitches = {instruction::CastOn{1}};
    pattern.rows.push_back(row0);

    RowInstruction row1;
    row1.side = RowSide::RS;
    row1.stitches = {instruction::Slip{}};
    pattern.rows.push_back(row1);

    auto yarn = default_yarn();
    auto gauge = default_gauge();
    auto graph = StitchGraph::from_instructions(pattern);
    auto yarn_path = YarnPath::from_stitch_graph(graph, yarn, gauge);

    float expected = gauge.stitch_width(yarn.compressed_radius) * 0.5f;

    // Check only user-row segments (skip return segments)
    for (const auto& seg : yarn_path.segments()) {
        if (seg.forms_loop && !seg.through.empty() &&
            !yarn_path.segments()[seg.through[0]].through.empty()) {
            EXPECT_NEAR(seg.target_yarn_length, expected, 0.01f);
        }
    }
}

TEST(YarnPath, K2togYarnLength) {
    // K2tog should be ~0.82x of base knit length
    PatternInstructions pattern;

    RowInstruction row0;
    row0.side = RowSide::RS;
    row0.stitches = {instruction::CastOn{3}};
    pattern.rows.push_back(row0);

    RowInstruction row1;
    row1.side = RowSide::RS;
    row1.stitches = {instruction::Knit{}, instruction::K2tog{}};
    pattern.rows.push_back(row1);

    auto graph = StitchGraph::from_instructions(pattern);
    auto yarn_path = YarnPath::from_stitch_graph(graph, default_yarn(), default_gauge());

    // Skip return segments (parent has empty through)
    std::vector<const YarnSegment*> user_worked;
    for (const auto& seg : yarn_path.segments()) {
        if (seg.forms_loop && !seg.through.empty() &&
            !yarn_path.segments()[seg.through[0]].through.empty()) {
            user_worked.push_back(&seg);
        }
    }

    ASSERT_EQ(user_worked.size(), 2u);
    float knit_len = user_worked[0]->target_yarn_length;
    float k2tog_len = user_worked[1]->target_yarn_length;

    EXPECT_GT(k2tog_len, 0.0f);
    float ratio = k2tog_len / knit_len;
    EXPECT_NEAR(ratio, 0.82f, 0.01f);
}

TEST(YarnPath, YarnOverYarnLength) {
    // YarnOver should be pi * loop_height (pure loop, no passthrough)
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

    float expected = static_cast<float>(M_PI) * gauge.loop_height(yarn.compressed_radius);

    // Find YO segment — skip foundation segments (which are in the first 2*N positions
    // and have a different yarn length formula)
    size_t co_segments = 4;  // 2 foundation + 2 return for cast-on 2
    for (size_t i = co_segments; i < yarn_path.segments().size(); ++i) {
        const auto& seg = yarn_path.segments()[i];
        if (seg.work_type == WorkType::Created && seg.forms_loop &&
            seg.wrap_direction == WrapDirection::None) {
            // This is the YO segment
            EXPECT_NEAR(seg.target_yarn_length, expected, 0.01f);
        }
    }
}
