#include "stitch_node.hpp"
#include "stitch_instruction.hpp"
#include "row_instruction.hpp"
#include <iostream>
#include <stdexcept>
#include <cassert>

using namespace yarnpath;

#define ASSERT_EQ(a, b) do { \
    if ((a) != (b)) { \
        throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__) + \
            " assertion failed: " #a " != " #b); \
    } \
} while(0)

#define ASSERT_TRUE(x) do { \
    if (!(x)) { \
        throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__) + \
            " assertion failed: " #x); \
    } \
} while(0)

namespace {

void test_stockinette() {
    std::cout << "  test_stockinette (3x3)...\n";

    // Cast on 3, then two rows of knit
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

    // Total: 3 cast-on + 3 row-1 + 3 row-2 = 9
    ASSERT_EQ(graph.size(), 9u);
    ASSERT_EQ(graph.row_count(), 3u);

    // Check row 0 (cast-on)
    auto r0 = graph.row(0);
    ASSERT_EQ(r0.size(), 3u);
    for (const auto& node : r0) {
        ASSERT_TRUE(std::holds_alternative<stitch::CastOn>(node.operation));
        ASSERT_TRUE(node.worked_through.empty());
    }

    // Check row 1
    auto r1 = graph.row(1);
    ASSERT_EQ(r1.size(), 3u);
    for (const auto& node : r1) {
        ASSERT_TRUE(std::holds_alternative<stitch::Knit>(node.operation));
        ASSERT_EQ(node.worked_through.size(), 1u);
    }

    // Check row 2
    auto r2 = graph.row(2);
    ASSERT_EQ(r2.size(), 3u);
    for (const auto& node : r2) {
        ASSERT_TRUE(std::holds_alternative<stitch::Purl>(node.operation));
        ASSERT_EQ(node.worked_through.size(), 1u);
    }

    // Each row-2 stitch has exactly 1 parent in row-1
    for (const auto& node : r2) {
        StitchId parent = node.worked_through[0];
        const auto* parent_node = graph.get(parent);
        ASSERT_TRUE(parent_node != nullptr);
        ASSERT_EQ(parent_node->row, 1u);
    }
}

void test_k2tog_decrease() {
    std::cout << "  test_k2tog_decrease...\n";

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

    // 4 cast-on + 3 row-1 = 7
    ASSERT_EQ(graph.size(), 7u);

    // Row 1 has 3 stitches
    auto r1 = graph.row(1);
    ASSERT_EQ(r1.size(), 3u);

    // Find the K2tog
    const StitchNode* k2tog_node = nullptr;
    for (const auto& node : r1) {
        if (std::holds_alternative<stitch::K2tog>(node.operation)) {
            k2tog_node = &node;
            break;
        }
    }
    ASSERT_TRUE(k2tog_node != nullptr);

    // K2tog has 2 parents
    ASSERT_EQ(k2tog_node->worked_through.size(), 2u);

    // Parents are from cast-on (row 0, stitches 1 and 2)
    for (StitchId parent : k2tog_node->worked_through) {
        const auto* p = graph.get(parent);
        ASSERT_TRUE(p != nullptr);
        ASSERT_EQ(p->row, 0u);
    }
}

void test_yarn_over_increase() {
    std::cout << "  test_yarn_over_increase...\n";

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

    // 2 cast-on + 3 row-1 = 5
    ASSERT_EQ(graph.size(), 5u);

    // Row 1 has 3 stitches
    auto r1 = graph.row(1);
    ASSERT_EQ(r1.size(), 3u);

    // Find the YO
    const StitchNode* yo_node = nullptr;
    for (const auto& node : r1) {
        if (std::holds_alternative<stitch::YarnOver>(node.operation)) {
            yo_node = &node;
            break;
        }
    }
    ASSERT_TRUE(yo_node != nullptr);

    // YO has 0 parents
    ASSERT_EQ(yo_node->worked_through.size(), 0u);
}

void test_kfb_increase() {
    std::cout << "  test_kfb_increase...\n";

    // Cast on 2, row 1: KFB K
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

    // 2 cast-on + 3 row-1 = 5
    ASSERT_EQ(graph.size(), 5u);

    // Row 1 has 3 stitches
    auto r1 = graph.row(1);
    ASSERT_EQ(r1.size(), 3u);

    // First two stitches are KFB, both with same parent
    ASSERT_TRUE(std::holds_alternative<stitch::KFB>(r1[0].operation));
    ASSERT_TRUE(std::holds_alternative<stitch::KFB>(r1[1].operation));
    ASSERT_EQ(r1[0].worked_through.size(), 1u);
    ASSERT_EQ(r1[1].worked_through.size(), 1u);
    ASSERT_EQ(r1[0].worked_through[0], r1[1].worked_through[0]);

    // Parent is stitch 0 from cast-on
    ASSERT_EQ(r1[0].worked_through[0], 0u);
}

void test_cable_left() {
    std::cout << "  test_cable_left (C4L)...\n";

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

    // 6 cast-on + 6 row-1 = 12
    ASSERT_EQ(graph.size(), 12u);

    // Row 1 has 6 stitches
    auto r1 = graph.row(1);
    ASSERT_EQ(r1.size(), 6u);

    // Stitches 1-4 are CableLeft
    for (size_t i = 1; i <= 4; ++i) {
        ASSERT_TRUE(std::holds_alternative<stitch::CableLeft>(r1[i].operation));
    }

    // For CableLeft{2,2}:
    // - Hold = first 2 cast-on stitches after the first K (IDs 1, 2)
    // - Cross = next 2 cast-on stitches (IDs 3, 4)
    // - Output order: crossed first, then held
    // So: r1[1] works through cast-on 3, r1[2] through cast-on 4
    //     r1[3] works through cast-on 1, r1[4] through cast-on 2

    // First K works through cast-on 0
    ASSERT_EQ(r1[0].worked_through[0], 0u);

    // Cable stitches - crossed are worked first
    ASSERT_EQ(r1[1].worked_through[0], 3u);  // first crossed
    ASSERT_EQ(r1[2].worked_through[0], 4u);  // second crossed
    ASSERT_EQ(r1[3].worked_through[0], 1u);  // first held
    ASSERT_EQ(r1[4].worked_through[0], 2u);  // second held

    // Last K works through cast-on 5
    ASSERT_EQ(r1[5].worked_through[0], 5u);
}

void test_slip_stitch() {
    std::cout << "  test_slip_stitch...\n";

    // Cast on 3, row 1: K Slip K, row 2: K K K
    PatternInstructions pattern;

    RowInstruction row0;
    row0.side = RowSide::RS;
    row0.stitches = {instruction::CastOn{3}};
    pattern.rows.push_back(row0);

    RowInstruction row1;
    row1.side = RowSide::RS;
    row1.stitches = {instruction::Knit{}, instruction::Slip{}, instruction::Knit{}};
    pattern.rows.push_back(row1);

    RowInstruction row2;
    row2.side = RowSide::WS;
    instruction::Repeat rep;
    rep.instructions = {instruction::Knit{}};
    rep.times = 3;
    row2.stitches = {rep};
    pattern.rows.push_back(row2);

    auto graph = StitchGraph::from_instructions(pattern);

    // 3 + 3 + 3 = 9
    ASSERT_EQ(graph.size(), 9u);

    // Row 1 has slip in middle
    auto r1 = graph.row(1);
    ASSERT_EQ(r1.size(), 3u);
    ASSERT_TRUE(std::holds_alternative<stitch::Slip>(r1[1].operation));

    // The slip stitch passes through to row 2 - it has a parent from row 0
    // and becomes a live stitch that row 2 works through
    auto r2 = graph.row(2);
    ASSERT_EQ(r2.size(), 3u);

    // Each row 2 stitch has parent in row 1
    for (const auto& node : r2) {
        StitchId parent = node.worked_through[0];
        const auto* p = graph.get(parent);
        ASSERT_TRUE(p != nullptr);
        ASSERT_EQ(p->row, 1u);
    }
}

void test_empty_pattern() {
    std::cout << "  test_empty_pattern...\n";

    PatternInstructions pattern;
    auto graph = StitchGraph::from_instructions(pattern);

    ASSERT_EQ(graph.size(), 0u);
    ASSERT_EQ(graph.row_count(), 0u);
}

void test_cast_on_only() {
    std::cout << "  test_cast_on_only...\n";

    PatternInstructions pattern;
    RowInstruction row0;
    row0.side = RowSide::RS;
    row0.stitches = {instruction::CastOn{5}};
    pattern.rows.push_back(row0);

    auto graph = StitchGraph::from_instructions(pattern);

    ASSERT_EQ(graph.size(), 5u);
    ASSERT_EQ(graph.row_count(), 1u);

    auto r0 = graph.row(0);
    ASSERT_EQ(r0.size(), 5u);
    for (const auto& node : r0) {
        ASSERT_TRUE(std::holds_alternative<stitch::CastOn>(node.operation));
    }
}

void test_single_stitch() {
    std::cout << "  test_single_stitch...\n";

    // Cast on 1, then K for 3 rows
    PatternInstructions pattern;

    RowInstruction row0;
    row0.side = RowSide::RS;
    row0.stitches = {instruction::CastOn{1}};
    pattern.rows.push_back(row0);

    for (int i = 0; i < 3; ++i) {
        RowInstruction row;
        row.side = (i % 2 == 0) ? RowSide::RS : RowSide::WS;
        row.stitches = {instruction::Knit{}};
        pattern.rows.push_back(row);
    }

    auto graph = StitchGraph::from_instructions(pattern);

    // 1 + 1 + 1 + 1 = 4
    ASSERT_EQ(graph.size(), 4u);
    ASSERT_EQ(graph.row_count(), 4u);

    // Each stitch (except cast-on) has parent in previous row
    for (uint32_t r = 1; r < graph.row_count(); ++r) {
        auto row = graph.row(r);
        ASSERT_EQ(row.size(), 1u);
        ASSERT_EQ(row[0].worked_through.size(), 1u);

        StitchId parent = row[0].worked_through[0];
        const auto* p = graph.get(parent);
        ASSERT_EQ(p->row, r - 1);
    }
}

void test_children_of() {
    std::cout << "  test_children_of...\n";

    // Cast on 2, row 1: K K
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

    // Cast-on stitch 0 has one child (row 1 stitch 0)
    auto children0 = graph.children_of(0);
    ASSERT_EQ(children0.size(), 1u);
    ASSERT_EQ(children0[0], 2u);

    // Cast-on stitch 1 has one child (row 1 stitch 1)
    auto children1 = graph.children_of(1);
    ASSERT_EQ(children1.size(), 1u);
    ASSERT_EQ(children1[0], 3u);
}

void test_panel_id() {
    std::cout << "  test_panel_id...\n";

    PatternInstructions pattern;
    pattern.panel_id = 99;

    RowInstruction row0;
    row0.side = RowSide::RS;
    row0.stitches = {instruction::CastOn{2}};
    pattern.rows.push_back(row0);

    auto graph = StitchGraph::from_instructions(pattern);

    for (const auto& node : graph.nodes()) {
        ASSERT_TRUE(node.panel_id.has_value());
        ASSERT_EQ(*node.panel_id, 99u);
    }
}

void test_ssk_decrease() {
    std::cout << "  test_ssk_decrease...\n";

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

    // 4 cast-on + 3 row-1 = 7
    ASSERT_EQ(graph.size(), 7u);

    auto r1 = graph.row(1);
    ASSERT_EQ(r1.size(), 3u);

    // Verify stitch types
    ASSERT_TRUE(std::holds_alternative<stitch::Knit>(r1[0].operation));
    ASSERT_TRUE(std::holds_alternative<stitch::SSK>(r1[1].operation));
    ASSERT_TRUE(std::holds_alternative<stitch::Knit>(r1[2].operation));

    // SSK consumes stitches 1 and 2 from cast-on
    ASSERT_EQ(r1[1].worked_through.size(), 2u);
    ASSERT_EQ(r1[1].worked_through[0], 1u);
    ASSERT_EQ(r1[1].worked_through[1], 2u);

    // First K works through cast-on 0, last K works through cast-on 3
    ASSERT_EQ(r1[0].worked_through[0], 0u);
    ASSERT_EQ(r1[2].worked_through[0], 3u);
}

void test_s2kp_decrease() {
    std::cout << "  test_s2kp_decrease...\n";

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

    // 5 cast-on + 3 row-1 = 8
    ASSERT_EQ(graph.size(), 8u);

    auto r1 = graph.row(1);
    ASSERT_EQ(r1.size(), 3u);

    // Verify S2KP operation
    ASSERT_TRUE(std::holds_alternative<stitch::S2KP>(r1[1].operation));

    // S2KP consumes 3 stitches
    ASSERT_EQ(r1[1].worked_through.size(), 3u);
    ASSERT_EQ(r1[1].worked_through[0], 1u);
    ASSERT_EQ(r1[1].worked_through[1], 2u);
    ASSERT_EQ(r1[1].worked_through[2], 3u);

    // First K works through cast-on 0, last K works through cast-on 4
    ASSERT_EQ(r1[0].worked_through[0], 0u);
    ASSERT_EQ(r1[2].worked_through[0], 4u);
}

void test_m1l_increase() {
    std::cout << "  test_m1l_increase...\n";

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

    // 2 cast-on + 3 row-1 = 5
    ASSERT_EQ(graph.size(), 5u);

    auto r1 = graph.row(1);
    ASSERT_EQ(r1.size(), 3u);

    // Verify M1L operation
    ASSERT_TRUE(std::holds_alternative<stitch::M1L>(r1[1].operation));

    // M1L has 0 parents (creates stitch from bar between stitches)
    ASSERT_EQ(r1[1].worked_through.size(), 0u);

    // Verify the K stitches have correct parents
    ASSERT_EQ(r1[0].worked_through[0], 0u);
    ASSERT_EQ(r1[2].worked_through[0], 1u);
}

void test_m1r_increase() {
    std::cout << "  test_m1r_increase...\n";

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

    // 2 cast-on + 3 row-1 = 5
    ASSERT_EQ(graph.size(), 5u);

    auto r1 = graph.row(1);
    ASSERT_EQ(r1.size(), 3u);

    // Verify M1R operation
    ASSERT_TRUE(std::holds_alternative<stitch::M1R>(r1[1].operation));

    // M1R has 0 parents
    ASSERT_EQ(r1[1].worked_through.size(), 0u);
}

void test_bind_off() {
    std::cout << "  test_bind_off...\n";

    // Cast on 5, row 1: K K K K K, row 2: BindOff{5}
    PatternInstructions pattern;

    RowInstruction row0;
    row0.side = RowSide::RS;
    row0.stitches = {instruction::CastOn{5}};
    pattern.rows.push_back(row0);

    RowInstruction row1;
    row1.side = RowSide::RS;
    instruction::Repeat rep;
    rep.instructions = {instruction::Knit{}};
    rep.times = 5;
    row1.stitches = {rep};
    pattern.rows.push_back(row1);

    RowInstruction row2;
    row2.side = RowSide::WS;
    row2.stitches = {instruction::BindOff{5}};
    pattern.rows.push_back(row2);

    auto graph = StitchGraph::from_instructions(pattern);

    // 5 cast-on + 5 row-1 + 5 bind-off = 15
    ASSERT_EQ(graph.size(), 15u);
    ASSERT_EQ(graph.row_count(), 3u);

    auto r2 = graph.row(2);
    ASSERT_EQ(r2.size(), 5u);

    // All row-2 stitches are BindOff with 1 parent each
    for (const auto& node : r2) {
        ASSERT_TRUE(std::holds_alternative<stitch::BindOff>(node.operation));
        ASSERT_EQ(node.worked_through.size(), 1u);

        // Parent is in row 1
        const auto* p = graph.get(node.worked_through[0]);
        ASSERT_TRUE(p != nullptr);
        ASSERT_EQ(p->row, 1u);
    }

    // Verify BindOff stitches are not live (no children)
    // Actually, we can check this through children_of
    for (const auto& node : r2) {
        auto children = graph.children_of(node.id);
        ASSERT_EQ(children.size(), 0u);
    }
}

void test_cable_right() {
    std::cout << "  test_cable_right (C4R)...\n";

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

    // 6 cast-on + 6 row-1 = 12
    ASSERT_EQ(graph.size(), 12u);

    auto r1 = graph.row(1);
    ASSERT_EQ(r1.size(), 6u);

    // Stitches 1-4 are CableRight
    for (size_t i = 1; i <= 4; ++i) {
        ASSERT_TRUE(std::holds_alternative<stitch::CableRight>(r1[i].operation));
    }

    // For CableRight{2,2}:
    // - Cross = first 2 stitches after K (IDs 1, 2)
    // - Hold = next 2 stitches (IDs 3, 4)
    // - Output order: held first, then crossed
    // So: r1[1] works through cast-on 3, r1[2] through cast-on 4
    //     r1[3] works through cast-on 1, r1[4] through cast-on 2

    // First K works through cast-on 0
    ASSERT_EQ(r1[0].worked_through[0], 0u);

    // Cable stitches - held are worked first for CableRight
    ASSERT_EQ(r1[1].worked_through[0], 3u);  // first held
    ASSERT_EQ(r1[2].worked_through[0], 4u);  // second held
    ASSERT_EQ(r1[3].worked_through[0], 1u);  // first crossed
    ASSERT_EQ(r1[4].worked_through[0], 2u);  // second crossed

    // Last K works through cast-on 5
    ASSERT_EQ(r1[5].worked_through[0], 5u);
}

void test_ws_row_parent_child_topology() {
    std::cout << "  test_ws_row_parent_child_topology...\n";

    // Cast on 4, RS row: K K K K, WS row: P P P P, RS row: K K K K
    // Verify that WS row correctly reverses stitch order
    PatternInstructions pattern;

    RowInstruction row0;
    row0.side = RowSide::RS;
    row0.stitches = {instruction::CastOn{4}};
    pattern.rows.push_back(row0);

    RowInstruction row1;
    row1.side = RowSide::RS;
    instruction::Repeat rep1;
    rep1.instructions = {instruction::Knit{}};
    rep1.times = 4;
    row1.stitches = {rep1};
    pattern.rows.push_back(row1);

    RowInstruction row2;
    row2.side = RowSide::WS;
    instruction::Repeat rep2;
    rep2.instructions = {instruction::Purl{}};
    rep2.times = 4;
    row2.stitches = {rep2};
    pattern.rows.push_back(row2);

    RowInstruction row3;
    row3.side = RowSide::RS;
    instruction::Repeat rep3;
    rep3.instructions = {instruction::Knit{}};
    rep3.times = 4;
    row3.stitches = {rep3};
    pattern.rows.push_back(row3);

    auto graph = StitchGraph::from_instructions(pattern);

    ASSERT_EQ(graph.size(), 16u);

    auto r0 = graph.row(0);  // Cast-on: IDs 0,1,2,3
    auto r1 = graph.row(1);  // RS knit: IDs 4,5,6,7
    auto r2 = graph.row(2);  // WS purl: IDs 8,9,10,11
    auto r3 = graph.row(3);  // RS knit: IDs 12,13,14,15

    // Row 1 (RS): stitch 4 works through cast-on 0, etc.
    ASSERT_EQ(r1[0].worked_through[0], 0u);
    ASSERT_EQ(r1[1].worked_through[0], 1u);
    ASSERT_EQ(r1[2].worked_through[0], 2u);
    ASSERT_EQ(r1[3].worked_through[0], 3u);

    // Row 2 (WS): stitches work from right to left
    // Live stitches were [4,5,6,7], reversed to [7,6,5,4] for WS
    // So r2[0] (ID 8) works through 7, r2[1] works through 6, etc.
    ASSERT_EQ(r2[0].worked_through[0], 7u);
    ASSERT_EQ(r2[1].worked_through[0], 6u);
    ASSERT_EQ(r2[2].worked_through[0], 5u);
    ASSERT_EQ(r2[3].worked_through[0], 4u);

    // After WS, live stitches are [8,9,10,11], reversed back to [11,10,9,8]
    // Row 3 (RS): stitch 12 works through 11, etc. but then re-reversed
    // Actually after WS, the live stitches are reversed back to normal order
    ASSERT_EQ(r3[0].worked_through[0], 11u);
    ASSERT_EQ(r3[1].worked_through[0], 10u);
    ASSERT_EQ(r3[2].worked_through[0], 9u);
    ASSERT_EQ(r3[3].worked_through[0], 8u);
}

void test_column_assignment() {
    std::cout << "  test_column_assignment...\n";

    // Cast on 3, row 1: K K K
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

    auto r0 = graph.row(0);
    auto r1 = graph.row(1);

    // Verify columns are assigned sequentially
    for (uint32_t i = 0; i < 3; ++i) {
        ASSERT_EQ(r0[i].column, i);
        ASSERT_EQ(r1[i].column, i);
    }

    // Verify row assignment
    for (const auto& node : r0) {
        ASSERT_EQ(node.row, 0u);
    }
    for (const auto& node : r1) {
        ASSERT_EQ(node.row, 1u);
    }
}

void test_row_info() {
    std::cout << "  test_row_info...\n";

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

    const auto& row_infos = graph.row_infos();
    ASSERT_EQ(row_infos.size(), 2u);

    // Row 0
    ASSERT_EQ(row_infos[0].row_number, 0u);
    ASSERT_EQ(row_infos[0].side, RowSide::RS);
    ASSERT_EQ(row_infos[0].first_stitch_id, 0u);
    ASSERT_EQ(row_infos[0].stitch_count, 3u);

    // Row 1
    ASSERT_EQ(row_infos[1].row_number, 1u);
    ASSERT_EQ(row_infos[1].side, RowSide::WS);
    ASSERT_EQ(row_infos[1].first_stitch_id, 3u);
    ASSERT_EQ(row_infos[1].stitch_count, 3u);
}

void test_complex_pattern() {
    std::cout << "  test_complex_pattern (combined instructions)...\n";

    // A more complex pattern: cast on 6, various stitches
    // Row 1 (RS): K, YO, K2tog, K, M1L, K
    // This consumes 4 stitches (K:1, YO:0, K2tog:2, K:1, M1L:0, K:0)
    // Wait, we have 6 cast-on stitches
    // K consumes 1, YO consumes 0, K2tog consumes 2, K consumes 1, M1L consumes 0, K consumes 1
    // Total consumed: 1+0+2+1+0+1 = 5... but we need to consume 6
    // Let me adjust: K, YO, K2tog, K, K, K (6 consumed, 6 produced)

    PatternInstructions pattern;

    RowInstruction row0;
    row0.side = RowSide::RS;
    row0.stitches = {instruction::CastOn{6}};
    pattern.rows.push_back(row0);

    RowInstruction row1;
    row1.side = RowSide::RS;
    // K YO K2tog K K K: consumes 1+0+2+1+1+1=6, produces 1+1+1+1+1+1=6
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

    ASSERT_EQ(graph.size(), 12u);  // 6 + 6

    auto r1 = graph.row(1);
    ASSERT_EQ(r1.size(), 6u);

    // Verify types
    ASSERT_TRUE(std::holds_alternative<stitch::Knit>(r1[0].operation));
    ASSERT_TRUE(std::holds_alternative<stitch::YarnOver>(r1[1].operation));
    ASSERT_TRUE(std::holds_alternative<stitch::K2tog>(r1[2].operation));
    ASSERT_TRUE(std::holds_alternative<stitch::Knit>(r1[3].operation));
    ASSERT_TRUE(std::holds_alternative<stitch::Knit>(r1[4].operation));
    ASSERT_TRUE(std::holds_alternative<stitch::Knit>(r1[5].operation));

    // Verify parents
    ASSERT_EQ(r1[0].worked_through[0], 0u);  // K -> cast-on 0
    ASSERT_EQ(r1[1].worked_through.size(), 0u);  // YO has no parent
    ASSERT_EQ(r1[2].worked_through.size(), 2u);  // K2tog has 2 parents
    ASSERT_EQ(r1[2].worked_through[0], 1u);
    ASSERT_EQ(r1[2].worked_through[1], 2u);
    ASSERT_EQ(r1[3].worked_through[0], 3u);  // K -> cast-on 3
    ASSERT_EQ(r1[4].worked_through[0], 4u);  // K -> cast-on 4
    ASSERT_EQ(r1[5].worked_through[0], 5u);  // K -> cast-on 5
}

void test_get_invalid_id() {
    std::cout << "  test_get_invalid_id...\n";

    PatternInstructions pattern;
    RowInstruction row0;
    row0.side = RowSide::RS;
    row0.stitches = {instruction::CastOn{3}};
    pattern.rows.push_back(row0);

    auto graph = StitchGraph::from_instructions(pattern);

    // Valid ID
    const auto* node = graph.get(0);
    ASSERT_TRUE(node != nullptr);
    ASSERT_EQ(node->id, 0u);

    // Invalid ID
    const auto* invalid = graph.get(100);
    ASSERT_TRUE(invalid == nullptr);
}

void test_row_invalid() {
    std::cout << "  test_row_invalid...\n";

    PatternInstructions pattern;
    RowInstruction row0;
    row0.side = RowSide::RS;
    row0.stitches = {instruction::CastOn{3}};
    pattern.rows.push_back(row0);

    auto graph = StitchGraph::from_instructions(pattern);

    // Valid row
    auto r0 = graph.row(0);
    ASSERT_EQ(r0.size(), 3u);

    // Invalid row
    auto invalid = graph.row(100);
    ASSERT_EQ(invalid.size(), 0u);
}

void test_multiple_kfb_children() {
    std::cout << "  test_multiple_kfb_children...\n";

    // Verify that a single cast-on stitch can have 2 children when worked with KFB
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

    ASSERT_EQ(graph.size(), 3u);  // 1 cast-on + 2 KFB

    // Cast-on stitch 0 should have 2 children
    auto children = graph.children_of(0);
    ASSERT_EQ(children.size(), 2u);
    ASSERT_EQ(children[0], 1u);
    ASSERT_EQ(children[1], 2u);
}

void test_k2tog_single_child() {
    std::cout << "  test_k2tog_single_child...\n";

    // Verify that 2 cast-on stitches each have 1 child (the K2tog stitch)
    PatternInstructions pattern;

    RowInstruction row0;
    row0.side = RowSide::RS;
    row0.stitches = {instruction::CastOn{2}};
    pattern.rows.push_back(row0);

    RowInstruction row1;
    row1.side = RowSide::RS;
    row1.stitches = {instruction::K2tog{}};
    pattern.rows.push_back(row1);

    auto graph = StitchGraph::from_instructions(pattern);

    ASSERT_EQ(graph.size(), 3u);  // 2 cast-on + 1 K2tog

    // Both cast-on stitches should have the same child
    auto children0 = graph.children_of(0);
    auto children1 = graph.children_of(1);
    ASSERT_EQ(children0.size(), 1u);
    ASSERT_EQ(children1.size(), 1u);
    ASSERT_EQ(children0[0], 2u);
    ASSERT_EQ(children1[0], 2u);
}

void test_asymmetric_cable() {
    std::cout << "  test_asymmetric_cable (C6L = hold 2, cross 4)...\n";

    // Cast on 8, row 1: K CableLeft{2,4} K
    PatternInstructions pattern;

    RowInstruction row0;
    row0.side = RowSide::RS;
    row0.stitches = {instruction::CastOn{8}};
    pattern.rows.push_back(row0);

    RowInstruction row1;
    row1.side = RowSide::RS;
    row1.stitches = {instruction::Knit{}, instruction::CableLeft{2, 4}, instruction::Knit{}};
    pattern.rows.push_back(row1);

    auto graph = StitchGraph::from_instructions(pattern);

    ASSERT_EQ(graph.size(), 16u);  // 8 + 8

    auto r1 = graph.row(1);
    ASSERT_EQ(r1.size(), 8u);

    // First K works through cast-on 0
    ASSERT_EQ(r1[0].worked_through[0], 0u);

    // CableLeft{2,4}: hold=[1,2], cross=[3,4,5,6]
    // Output order: crossed first (3,4,5,6), then held (1,2)
    ASSERT_EQ(r1[1].worked_through[0], 3u);
    ASSERT_EQ(r1[2].worked_through[0], 4u);
    ASSERT_EQ(r1[3].worked_through[0], 5u);
    ASSERT_EQ(r1[4].worked_through[0], 6u);
    ASSERT_EQ(r1[5].worked_through[0], 1u);
    ASSERT_EQ(r1[6].worked_through[0], 2u);

    // Last K works through cast-on 7
    ASSERT_EQ(r1[7].worked_through[0], 7u);
}

void test_no_panel_id() {
    std::cout << "  test_no_panel_id...\n";

    PatternInstructions pattern;
    // panel_id not set (default nullopt)

    RowInstruction row0;
    row0.side = RowSide::RS;
    row0.stitches = {instruction::CastOn{2}};
    pattern.rows.push_back(row0);

    auto graph = StitchGraph::from_instructions(pattern);

    for (const auto& node : graph.nodes()) {
        ASSERT_TRUE(!node.panel_id.has_value());
    }
}

}  // namespace

void test_stitch_graph() {
    test_stockinette();
    test_k2tog_decrease();
    test_yarn_over_increase();
    test_kfb_increase();
    test_cable_left();
    test_slip_stitch();
    test_empty_pattern();
    test_cast_on_only();
    test_single_stitch();
    test_children_of();
    test_panel_id();
    test_ssk_decrease();
    test_s2kp_decrease();
    test_m1l_increase();
    test_m1r_increase();
    test_bind_off();
    test_cable_right();
    test_ws_row_parent_child_topology();
    test_column_assignment();
    test_row_info();
    test_complex_pattern();
    test_get_invalid_id();
    test_row_invalid();
    test_multiple_kfb_children();
    test_k2tog_single_child();
    test_asymmetric_cable();
    test_no_panel_id();
}
