#include "stitch_instruction.hpp"
#include "row_instruction.hpp"
#include <gtest/gtest.h>

using namespace yarnpath;

TEST(Instructions, BasicConstruction) {
    // Create basic stitches and verify variant holds correct type
    StitchInstruction knit = instruction::Knit{};
    EXPECT_TRUE(std::holds_alternative<instruction::Knit>(knit));

    StitchInstruction purl = instruction::Purl{};
    EXPECT_TRUE(std::holds_alternative<instruction::Purl>(purl));

    StitchInstruction slip = instruction::Slip{};
    EXPECT_TRUE(std::holds_alternative<instruction::Slip>(slip));

    // Basic stitch consumption/production
    EXPECT_EQ(stitches_consumed(knit), 1u);
    EXPECT_EQ(stitches_produced(knit), 1u);
    EXPECT_EQ(stitches_consumed(purl), 1u);
    EXPECT_EQ(stitches_produced(purl), 1u);
    EXPECT_EQ(stitches_consumed(slip), 1u);
    EXPECT_EQ(stitches_produced(slip), 1u);
}

TEST(Instructions, CastOnBindOff) {
    StitchInstruction cast_on = instruction::CastOn{10};
    EXPECT_TRUE(std::holds_alternative<instruction::CastOn>(cast_on));
    EXPECT_EQ(std::get<instruction::CastOn>(cast_on).count, 10u);
    EXPECT_EQ(stitches_consumed(cast_on), 0u);
    EXPECT_EQ(stitches_produced(cast_on), 10u);

    StitchInstruction bind_off = instruction::BindOff{10};
    EXPECT_TRUE(std::holds_alternative<instruction::BindOff>(bind_off));
    EXPECT_EQ(std::get<instruction::BindOff>(bind_off).count, 10u);
    EXPECT_EQ(stitches_consumed(bind_off), 10u);
    EXPECT_EQ(stitches_produced(bind_off), 0u);
}

TEST(Instructions, Increases) {
    StitchInstruction yo = instruction::YarnOver{};
    EXPECT_EQ(stitches_consumed(yo), 0u);
    EXPECT_EQ(stitches_produced(yo), 1u);

    StitchInstruction kfb = instruction::KFB{};
    EXPECT_EQ(stitches_consumed(kfb), 1u);
    EXPECT_EQ(stitches_produced(kfb), 2u);

    StitchInstruction m1l = instruction::M1L{};
    EXPECT_EQ(stitches_consumed(m1l), 0u);
    EXPECT_EQ(stitches_produced(m1l), 1u);

    StitchInstruction m1r = instruction::M1R{};
    EXPECT_EQ(stitches_consumed(m1r), 0u);
    EXPECT_EQ(stitches_produced(m1r), 1u);
}

TEST(Instructions, Decreases) {
    StitchInstruction k2tog = instruction::K2tog{};
    EXPECT_EQ(stitches_consumed(k2tog), 2u);
    EXPECT_EQ(stitches_produced(k2tog), 1u);

    StitchInstruction ssk = instruction::SSK{};
    EXPECT_EQ(stitches_consumed(ssk), 2u);
    EXPECT_EQ(stitches_produced(ssk), 1u);

    StitchInstruction s2kp = instruction::S2KP{};
    EXPECT_EQ(stitches_consumed(s2kp), 3u);
    EXPECT_EQ(stitches_produced(s2kp), 1u);
}

TEST(Instructions, Cables) {
    StitchInstruction cable_left = instruction::CableLeft{2, 2};
    EXPECT_TRUE(std::holds_alternative<instruction::CableLeft>(cable_left));
    EXPECT_EQ(std::get<instruction::CableLeft>(cable_left).hold, 2);
    EXPECT_EQ(std::get<instruction::CableLeft>(cable_left).cross, 2);
    EXPECT_EQ(stitches_consumed(cable_left), 4u);
    EXPECT_EQ(stitches_produced(cable_left), 4u);

    StitchInstruction cable_right = instruction::CableRight{3, 3};
    EXPECT_EQ(stitches_consumed(cable_right), 6u);
    EXPECT_EQ(stitches_produced(cable_right), 6u);
}

TEST(Instructions, Repeat) {
    // Simple repeat: [K, P] x 3 = K P K P K P
    instruction::Repeat rep;
    rep.instructions = {instruction::Knit{}, instruction::Purl{}};
    rep.times = 3;

    StitchInstruction repeat_instr = rep;
    EXPECT_EQ(stitches_consumed(repeat_instr), 6u);
    EXPECT_EQ(stitches_produced(repeat_instr), 6u);

    // Expand the repeat
    std::vector<StitchInstruction> to_expand = {repeat_instr};
    auto expanded = expand_instructions(to_expand);
    ASSERT_EQ(expanded.size(), 6u);
    EXPECT_TRUE(std::holds_alternative<instruction::Knit>(expanded[0]));
    EXPECT_TRUE(std::holds_alternative<instruction::Purl>(expanded[1]));
    EXPECT_TRUE(std::holds_alternative<instruction::Knit>(expanded[2]));
    EXPECT_TRUE(std::holds_alternative<instruction::Purl>(expanded[3]));
    EXPECT_TRUE(std::holds_alternative<instruction::Knit>(expanded[4]));
    EXPECT_TRUE(std::holds_alternative<instruction::Purl>(expanded[5]));
}

TEST(Instructions, NestedRepeat) {
    // Nested: [[K] x 2, P] x 2 = K K P K K P
    instruction::Repeat inner;
    inner.instructions = {instruction::Knit{}};
    inner.times = 2;

    instruction::Repeat outer;
    outer.instructions = {inner, instruction::Purl{}};
    outer.times = 2;

    std::vector<StitchInstruction> to_expand = {outer};
    auto expanded = expand_instructions(to_expand);
    ASSERT_EQ(expanded.size(), 6u);
    EXPECT_TRUE(std::holds_alternative<instruction::Knit>(expanded[0]));
    EXPECT_TRUE(std::holds_alternative<instruction::Knit>(expanded[1]));
    EXPECT_TRUE(std::holds_alternative<instruction::Purl>(expanded[2]));
    EXPECT_TRUE(std::holds_alternative<instruction::Knit>(expanded[3]));
    EXPECT_TRUE(std::holds_alternative<instruction::Knit>(expanded[4]));
    EXPECT_TRUE(std::holds_alternative<instruction::Purl>(expanded[5]));
}

TEST(Instructions, RowInstruction) {
    RowInstruction row;
    row.side = RowSide::RS;
    row.stitches = {instruction::Knit{}, instruction::Purl{}, instruction::Knit{}};

    EXPECT_EQ(row.side, RowSide::RS);
    EXPECT_EQ(row.stitches.size(), 3u);
}

TEST(Instructions, PatternInstructions) {
    PatternInstructions pattern;
    pattern.panel_id = 42;

    RowInstruction row1;
    row1.side = RowSide::RS;
    row1.stitches = {instruction::CastOn{10}};
    pattern.rows.push_back(row1);

    RowInstruction row2;
    row2.side = RowSide::WS;
    instruction::Repeat rep;
    rep.instructions = {instruction::Purl{}};
    rep.times = 10;
    row2.stitches = {rep};
    pattern.rows.push_back(row2);

    EXPECT_TRUE(pattern.panel_id.has_value());
    EXPECT_EQ(*pattern.panel_id, 42u);
    EXPECT_EQ(pattern.rows.size(), 2u);
}
