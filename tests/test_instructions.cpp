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

void test_basic_construction() {
    std::cout << "  test_basic_construction...\n";

    // Create basic stitches and verify variant holds correct type
    StitchInstruction knit = instruction::Knit{};
    ASSERT_TRUE(std::holds_alternative<instruction::Knit>(knit));

    StitchInstruction purl = instruction::Purl{};
    ASSERT_TRUE(std::holds_alternative<instruction::Purl>(purl));

    StitchInstruction slip = instruction::Slip{};
    ASSERT_TRUE(std::holds_alternative<instruction::Slip>(slip));

    // Basic stitch consumption/production
    ASSERT_EQ(stitches_consumed(knit), 1u);
    ASSERT_EQ(stitches_produced(knit), 1u);
    ASSERT_EQ(stitches_consumed(purl), 1u);
    ASSERT_EQ(stitches_produced(purl), 1u);
    ASSERT_EQ(stitches_consumed(slip), 1u);
    ASSERT_EQ(stitches_produced(slip), 1u);
}

void test_cast_on_bind_off() {
    std::cout << "  test_cast_on_bind_off...\n";

    StitchInstruction cast_on = instruction::CastOn{10};
    ASSERT_TRUE(std::holds_alternative<instruction::CastOn>(cast_on));
    ASSERT_EQ(std::get<instruction::CastOn>(cast_on).count, 10u);
    ASSERT_EQ(stitches_consumed(cast_on), 0u);
    ASSERT_EQ(stitches_produced(cast_on), 10u);

    StitchInstruction bind_off = instruction::BindOff{10};
    ASSERT_TRUE(std::holds_alternative<instruction::BindOff>(bind_off));
    ASSERT_EQ(std::get<instruction::BindOff>(bind_off).count, 10u);
    ASSERT_EQ(stitches_consumed(bind_off), 10u);
    ASSERT_EQ(stitches_produced(bind_off), 0u);
}

void test_increases() {
    std::cout << "  test_increases...\n";

    StitchInstruction yo = instruction::YarnOver{};
    ASSERT_EQ(stitches_consumed(yo), 0u);
    ASSERT_EQ(stitches_produced(yo), 1u);

    StitchInstruction kfb = instruction::KFB{};
    ASSERT_EQ(stitches_consumed(kfb), 1u);
    ASSERT_EQ(stitches_produced(kfb), 2u);

    StitchInstruction m1l = instruction::M1L{};
    ASSERT_EQ(stitches_consumed(m1l), 0u);
    ASSERT_EQ(stitches_produced(m1l), 1u);

    StitchInstruction m1r = instruction::M1R{};
    ASSERT_EQ(stitches_consumed(m1r), 0u);
    ASSERT_EQ(stitches_produced(m1r), 1u);
}

void test_decreases() {
    std::cout << "  test_decreases...\n";

    StitchInstruction k2tog = instruction::K2tog{};
    ASSERT_EQ(stitches_consumed(k2tog), 2u);
    ASSERT_EQ(stitches_produced(k2tog), 1u);

    StitchInstruction ssk = instruction::SSK{};
    ASSERT_EQ(stitches_consumed(ssk), 2u);
    ASSERT_EQ(stitches_produced(ssk), 1u);

    StitchInstruction s2kp = instruction::S2KP{};
    ASSERT_EQ(stitches_consumed(s2kp), 3u);
    ASSERT_EQ(stitches_produced(s2kp), 1u);
}

void test_cables() {
    std::cout << "  test_cables...\n";

    StitchInstruction cable_left = instruction::CableLeft{2, 2};
    ASSERT_TRUE(std::holds_alternative<instruction::CableLeft>(cable_left));
    ASSERT_EQ(std::get<instruction::CableLeft>(cable_left).hold, 2);
    ASSERT_EQ(std::get<instruction::CableLeft>(cable_left).cross, 2);
    ASSERT_EQ(stitches_consumed(cable_left), 4u);
    ASSERT_EQ(stitches_produced(cable_left), 4u);

    StitchInstruction cable_right = instruction::CableRight{3, 3};
    ASSERT_EQ(stitches_consumed(cable_right), 6u);
    ASSERT_EQ(stitches_produced(cable_right), 6u);
}

void test_repeat() {
    std::cout << "  test_repeat...\n";

    // Simple repeat: [K, P] x 3 = K P K P K P
    instruction::Repeat rep;
    rep.instructions = {instruction::Knit{}, instruction::Purl{}};
    rep.times = 3;

    StitchInstruction repeat_instr = rep;
    ASSERT_EQ(stitches_consumed(repeat_instr), 6u);
    ASSERT_EQ(stitches_produced(repeat_instr), 6u);

    // Expand the repeat
    std::vector<StitchInstruction> to_expand = {repeat_instr};
    auto expanded = expand_instructions(to_expand);
    ASSERT_EQ(expanded.size(), 6u);
    ASSERT_TRUE(std::holds_alternative<instruction::Knit>(expanded[0]));
    ASSERT_TRUE(std::holds_alternative<instruction::Purl>(expanded[1]));
    ASSERT_TRUE(std::holds_alternative<instruction::Knit>(expanded[2]));
    ASSERT_TRUE(std::holds_alternative<instruction::Purl>(expanded[3]));
    ASSERT_TRUE(std::holds_alternative<instruction::Knit>(expanded[4]));
    ASSERT_TRUE(std::holds_alternative<instruction::Purl>(expanded[5]));
}

void test_nested_repeat() {
    std::cout << "  test_nested_repeat...\n";

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
    ASSERT_TRUE(std::holds_alternative<instruction::Knit>(expanded[0]));
    ASSERT_TRUE(std::holds_alternative<instruction::Knit>(expanded[1]));
    ASSERT_TRUE(std::holds_alternative<instruction::Purl>(expanded[2]));
    ASSERT_TRUE(std::holds_alternative<instruction::Knit>(expanded[3]));
    ASSERT_TRUE(std::holds_alternative<instruction::Knit>(expanded[4]));
    ASSERT_TRUE(std::holds_alternative<instruction::Purl>(expanded[5]));
}

void test_row_instruction() {
    std::cout << "  test_row_instruction...\n";

    RowInstruction row;
    row.side = RowSide::RS;
    row.stitches = {instruction::Knit{}, instruction::Purl{}, instruction::Knit{}};

    ASSERT_EQ(row.side, RowSide::RS);
    ASSERT_EQ(row.stitches.size(), 3u);
}

void test_pattern_instructions() {
    std::cout << "  test_pattern_instructions...\n";

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

    ASSERT_TRUE(pattern.panel_id.has_value());
    ASSERT_EQ(*pattern.panel_id, 42u);
    ASSERT_EQ(pattern.rows.size(), 2u);
}

}  // namespace

void test_instructions() {
    test_basic_construction();
    test_cast_on_bind_off();
    test_increases();
    test_decreases();
    test_cables();
    test_repeat();
    test_nested_repeat();
    test_row_instruction();
    test_pattern_instructions();
}
