#include "tokenizer.hpp"
#include "parser.hpp"
#include "emitter.hpp"
#include "stitch_instruction.hpp"
#include "row_instruction.hpp"
#include <iostream>
#include <stdexcept>
#include <cassert>

using namespace yarnpath;
using namespace yarnpath::parser;

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

// ============== Tokenizer Tests ==============

void test_tokenizer_basic_tokens() {
    std::cout << "  test_tokenizer_basic_tokens...\n";

    Tokenizer tokenizer("K P K2tog");

    Token t1 = tokenizer.next();
    ASSERT_EQ(t1.type, TokenType::Knit);
    ASSERT_EQ(t1.number.value_or(0), 1u);

    Token t2 = tokenizer.next();
    ASSERT_EQ(t2.type, TokenType::Purl);
    ASSERT_EQ(t2.number.value_or(0), 1u);

    Token t3 = tokenizer.next();
    ASSERT_EQ(t3.type, TokenType::K2tog);
}

void test_tokenizer_numbers() {
    std::cout << "  test_tokenizer_numbers...\n";

    Tokenizer tokenizer("K3 P5 42");

    Token t1 = tokenizer.next();
    ASSERT_EQ(t1.type, TokenType::Knit);
    ASSERT_EQ(t1.number.value_or(0), 3u);

    Token t2 = tokenizer.next();
    ASSERT_EQ(t2.type, TokenType::Purl);
    ASSERT_EQ(t2.number.value_or(0), 5u);

    Token t3 = tokenizer.next();
    ASSERT_EQ(t3.type, TokenType::Number);
    ASSERT_EQ(t3.number.value_or(0), 42u);
}

void test_tokenizer_multi_word_keywords() {
    std::cout << "  test_tokenizer_multi_word_keywords...\n";

    Tokenizer tokenizer("Cast on 20 sts. Bind off all sts.");

    ASSERT_EQ(tokenizer.next().type, TokenType::CastOn);
    ASSERT_EQ(tokenizer.next().type, TokenType::Number);
    ASSERT_EQ(tokenizer.next().type, TokenType::Sts);
    ASSERT_EQ(tokenizer.next().type, TokenType::Period);
    ASSERT_EQ(tokenizer.next().type, TokenType::BindOff);
    ASSERT_EQ(tokenizer.next().type, TokenType::All);
    ASSERT_EQ(tokenizer.next().type, TokenType::Sts);
}

void test_tokenizer_case_insensitivity() {
    std::cout << "  test_tokenizer_case_insensitivity...\n";

    Tokenizer tokenizer("K k YO yo SSK ssk");

    Token t1 = tokenizer.next();
    ASSERT_EQ(t1.type, TokenType::Knit);

    Token t2 = tokenizer.next();
    ASSERT_EQ(t2.type, TokenType::Knit);

    Token t3 = tokenizer.next();
    ASSERT_EQ(t3.type, TokenType::YarnOver);

    Token t4 = tokenizer.next();
    ASSERT_EQ(t4.type, TokenType::YarnOver);

    Token t5 = tokenizer.next();
    ASSERT_EQ(t5.type, TokenType::SSK);

    Token t6 = tokenizer.next();
    ASSERT_EQ(t6.type, TokenType::SSK);
}

void test_tokenizer_cables() {
    std::cout << "  test_tokenizer_cables...\n";

    Tokenizer tokenizer("C4L C6R C2/4L");

    Token t1 = tokenizer.next();
    ASSERT_EQ(t1.type, TokenType::CableLeft);
    ASSERT_EQ(t1.cable_hold.value_or(0), 2u);
    ASSERT_EQ(t1.cable_cross.value_or(0), 2u);

    Token t2 = tokenizer.next();
    ASSERT_EQ(t2.type, TokenType::CableRight);
    ASSERT_EQ(t2.cable_hold.value_or(0), 3u);
    ASSERT_EQ(t2.cable_cross.value_or(0), 3u);

    Token t3 = tokenizer.next();
    ASSERT_EQ(t3.type, TokenType::CableLeft);
    ASSERT_EQ(t3.cable_hold.value_or(0), 2u);
    ASSERT_EQ(t3.cable_cross.value_or(0), 4u);
}

void test_tokenizer_punctuation() {
    std::cout << "  test_tokenizer_punctuation...\n";

    Tokenizer tokenizer("K, P. *K* [P] (RS):");

    ASSERT_EQ(tokenizer.next().type, TokenType::Knit);
    ASSERT_EQ(tokenizer.next().type, TokenType::Comma);
    ASSERT_EQ(tokenizer.next().type, TokenType::Purl);
    ASSERT_EQ(tokenizer.next().type, TokenType::Period);
    ASSERT_EQ(tokenizer.next().type, TokenType::Asterisk);
    ASSERT_EQ(tokenizer.next().type, TokenType::Knit);
    ASSERT_EQ(tokenizer.next().type, TokenType::Asterisk);
    ASSERT_EQ(tokenizer.next().type, TokenType::LBracket);
    ASSERT_EQ(tokenizer.next().type, TokenType::Purl);
    ASSERT_EQ(tokenizer.next().type, TokenType::RBracket);
    ASSERT_EQ(tokenizer.next().type, TokenType::LParen);
    ASSERT_EQ(tokenizer.next().type, TokenType::RS);
    ASSERT_EQ(tokenizer.next().type, TokenType::RParen);
    ASSERT_EQ(tokenizer.next().type, TokenType::Colon);
}

void test_tokenizer_rep_from() {
    std::cout << "  test_tokenizer_rep_from...\n";

    Tokenizer tokenizer("rep from * to end");

    ASSERT_EQ(tokenizer.next().type, TokenType::RepFrom);
    ASSERT_EQ(tokenizer.next().type, TokenType::Asterisk);
    ASSERT_EQ(tokenizer.next().type, TokenType::To);
    ASSERT_EQ(tokenizer.next().type, TokenType::End);
}

void test_tokenizer_all_stitches() {
    std::cout << "  test_tokenizer_all_stitches...\n";

    Tokenizer tokenizer("M1L M1R KFB Sl S2KP SK2P");

    ASSERT_EQ(tokenizer.next().type, TokenType::M1L);
    ASSERT_EQ(tokenizer.next().type, TokenType::M1R);
    ASSERT_EQ(tokenizer.next().type, TokenType::KFB);
    ASSERT_EQ(tokenizer.next().type, TokenType::Slip);
    ASSERT_EQ(tokenizer.next().type, TokenType::S2KP);
    ASSERT_EQ(tokenizer.next().type, TokenType::S2KP);
}

// ============== Parser Tests ==============

void test_parser_simple_row() {
    std::cout << "  test_parser_simple_row...\n";

    Tokenizer tokenizer("Row 1 (RS): K3, P3.");
    Parser parser(std::move(tokenizer));

    auto ast = parser.parse();
    ASSERT_TRUE(!parser.has_errors());
    ASSERT_EQ(ast.nodes.size(), 1u);

    auto* row = std::get_if<ast::RowNode>(&ast.nodes[0]);
    ASSERT_TRUE(row != nullptr);
    ASSERT_EQ(row->row_number.value_or(0), 1u);
    ASSERT_EQ(row->side.value_or(RowSide::WS), RowSide::RS);
}

void test_parser_cast_on() {
    std::cout << "  test_parser_cast_on...\n";

    Tokenizer tokenizer("Cast on 20 sts.");
    Parser parser(std::move(tokenizer));

    auto ast = parser.parse();
    ASSERT_TRUE(!parser.has_errors());
    ASSERT_EQ(ast.nodes.size(), 1u);

    auto* cast_on = std::get_if<ast::CastOnNode>(&ast.nodes[0]);
    ASSERT_TRUE(cast_on != nullptr);
    ASSERT_EQ(cast_on->count, 20u);
}

void test_parser_bind_off() {
    std::cout << "  test_parser_bind_off...\n";

    Tokenizer tokenizer("Bind off all sts.");
    Parser parser(std::move(tokenizer));

    auto ast = parser.parse();
    ASSERT_TRUE(!parser.has_errors());
    ASSERT_EQ(ast.nodes.size(), 1u);

    auto* bind_off = std::get_if<ast::BindOffNode>(&ast.nodes[0]);
    ASSERT_TRUE(bind_off != nullptr);
    ASSERT_TRUE(bind_off->all);
}

void test_parser_asterisk_repeat() {
    std::cout << "  test_parser_asterisk_repeat...\n";

    Tokenizer tokenizer("Row 1 (RS): K2, *K2, P2*, rep from * to end.");
    Parser parser(std::move(tokenizer));

    auto ast = parser.parse();
    ASSERT_TRUE(!parser.has_errors());
    ASSERT_EQ(ast.nodes.size(), 1u);

    auto* row = std::get_if<ast::RowNode>(&ast.nodes[0]);
    ASSERT_TRUE(row != nullptr);
    // Should have K2 prefix and a repeat
    ASSERT_TRUE(row->elements.size() >= 2);
}

void test_parser_bracket_repeat() {
    std::cout << "  test_parser_bracket_repeat...\n";

    Tokenizer tokenizer("Row 1: [K1, P1] 5 times.");
    Parser parser(std::move(tokenizer));

    auto ast = parser.parse();
    ASSERT_TRUE(!parser.has_errors());
    ASSERT_EQ(ast.nodes.size(), 1u);

    auto* row = std::get_if<ast::RowNode>(&ast.nodes[0]);
    ASSERT_TRUE(row != nullptr);
    ASSERT_EQ(row->elements.size(), 1u);

    auto* repeat = std::get_if<ast::RepeatNode>(&row->elements[0]);
    ASSERT_TRUE(repeat != nullptr);
    ASSERT_EQ(repeat->times.value_or(0), 5u);
}

void test_parser_multiple_rows() {
    std::cout << "  test_parser_multiple_rows...\n";

    Tokenizer tokenizer(
        "Cast on 10 sts.\n"
        "Row 1 (RS): K10.\n"
        "Row 2 (WS): P10.\n"
        "Bind off all sts."
    );
    Parser parser(std::move(tokenizer));

    auto ast = parser.parse();
    ASSERT_TRUE(!parser.has_errors());
    ASSERT_EQ(ast.nodes.size(), 4u);
}

// ============== Emitter Tests ==============

void test_emitter_simple_pattern() {
    std::cout << "  test_emitter_simple_pattern...\n";

    Tokenizer tokenizer(
        "Cast on 10 sts.\n"
        "Row 1 (RS): K10.\n"
        "Bind off all sts."
    );
    Parser parser(std::move(tokenizer));
    auto ast = parser.parse();
    ASSERT_TRUE(!parser.has_errors());

    Emitter emitter;
    auto pattern = emitter.emit(ast);

    ASSERT_EQ(pattern.rows.size(), 3u);

    // Cast on row
    ASSERT_EQ(pattern.rows[0].stitches.size(), 1u);
    ASSERT_TRUE(std::holds_alternative<instruction::CastOn>(pattern.rows[0].stitches[0]));
    ASSERT_EQ(std::get<instruction::CastOn>(pattern.rows[0].stitches[0]).count, 10u);

    // Knit row
    ASSERT_EQ(pattern.rows[1].stitches.size(), 10u);
    for (const auto& st : pattern.rows[1].stitches) {
        ASSERT_TRUE(std::holds_alternative<instruction::Knit>(st));
    }

    // Bind off row
    ASSERT_EQ(pattern.rows[2].stitches.size(), 1u);
    ASSERT_TRUE(std::holds_alternative<instruction::BindOff>(pattern.rows[2].stitches[0]));
}

void test_emitter_repeat_to_end() {
    std::cout << "  test_emitter_repeat_to_end...\n";

    Tokenizer tokenizer(
        "Cast on 12 sts.\n"
        "Row 1 (RS): *K2, P2*, rep from * to end."
    );
    Parser parser(std::move(tokenizer));
    auto ast = parser.parse();
    ASSERT_TRUE(!parser.has_errors());

    Emitter emitter;
    auto pattern = emitter.emit(ast);

    ASSERT_EQ(pattern.rows.size(), 2u);

    // Row 1: should be K2 P2 repeated 3 times = 12 stitches
    ASSERT_EQ(pattern.rows[1].stitches.size(), 12u);

    // Check pattern: K K P P K K P P K K P P
    ASSERT_TRUE(std::holds_alternative<instruction::Knit>(pattern.rows[1].stitches[0]));
    ASSERT_TRUE(std::holds_alternative<instruction::Knit>(pattern.rows[1].stitches[1]));
    ASSERT_TRUE(std::holds_alternative<instruction::Purl>(pattern.rows[1].stitches[2]));
    ASSERT_TRUE(std::holds_alternative<instruction::Purl>(pattern.rows[1].stitches[3]));
}

void test_emitter_repeat_to_last() {
    std::cout << "  test_emitter_repeat_to_last...\n";

    Tokenizer tokenizer(
        "Cast on 14 sts.\n"
        "Row 1 (RS): K2, *K2, P2*, rep from * to last 4 sts, K4."
    );
    Parser parser(std::move(tokenizer));
    auto ast = parser.parse();

    // This is a complex pattern - just verify it parses
    ASSERT_TRUE(!parser.has_errors());
}

void test_emitter_bracket_repeat() {
    std::cout << "  test_emitter_bracket_repeat...\n";

    Tokenizer tokenizer(
        "Cast on 10 sts.\n"
        "Row 1: [K1, P1] 5 times."
    );
    Parser parser(std::move(tokenizer));
    auto ast = parser.parse();
    ASSERT_TRUE(!parser.has_errors());

    Emitter emitter;
    auto pattern = emitter.emit(ast);

    ASSERT_EQ(pattern.rows.size(), 2u);

    // Row 1: should be K P repeated 5 times = 10 stitches
    ASSERT_EQ(pattern.rows[1].stitches.size(), 10u);

    // Check alternating pattern
    for (size_t i = 0; i < 10; ++i) {
        if (i % 2 == 0) {
            ASSERT_TRUE(std::holds_alternative<instruction::Knit>(pattern.rows[1].stitches[i]));
        } else {
            ASSERT_TRUE(std::holds_alternative<instruction::Purl>(pattern.rows[1].stitches[i]));
        }
    }
}

void test_emitter_decreases() {
    std::cout << "  test_emitter_decreases...\n";

    Tokenizer tokenizer(
        "Cast on 10 sts.\n"
        "Row 1 (RS): K1, SSK, K4, K2tog, K1."
    );
    Parser parser(std::move(tokenizer));
    auto ast = parser.parse();
    ASSERT_TRUE(!parser.has_errors());

    Emitter emitter;
    auto pattern = emitter.emit(ast);

    // Count stitches
    uint32_t consumed = 0;
    uint32_t produced = 0;
    for (const auto& instr : pattern.rows[1].stitches) {
        consumed += stitches_consumed(instr);
        produced += stitches_produced(instr);
    }
    ASSERT_EQ(consumed, 10u);  // Uses all 10 stitches
    ASSERT_EQ(produced, 8u);   // Produces 8 (two decreases)
}

void test_emitter_increases() {
    std::cout << "  test_emitter_increases...\n";

    Tokenizer tokenizer(
        "Cast on 10 sts.\n"
        "Row 1 (RS): K1, YO, K8, YO, K1."
    );
    Parser parser(std::move(tokenizer));
    auto ast = parser.parse();
    ASSERT_TRUE(!parser.has_errors());

    Emitter emitter;
    auto pattern = emitter.emit(ast);

    // Count stitches
    uint32_t consumed = 0;
    uint32_t produced = 0;
    for (const auto& instr : pattern.rows[1].stitches) {
        consumed += stitches_consumed(instr);
        produced += stitches_produced(instr);
    }
    ASSERT_EQ(consumed, 10u);  // Uses all 10 stitches
    ASSERT_EQ(produced, 12u);  // Produces 12 (two yarn overs)
}

void test_emitter_cables() {
    std::cout << "  test_emitter_cables...\n";

    Tokenizer tokenizer(
        "Cast on 10 sts.\n"
        "Row 1 (RS): K3, C4L, K3."
    );
    Parser parser(std::move(tokenizer));
    auto ast = parser.parse();
    ASSERT_TRUE(!parser.has_errors());

    Emitter emitter;
    auto pattern = emitter.emit(ast);

    // Find the cable instruction
    bool found_cable = false;
    for (const auto& instr : pattern.rows[1].stitches) {
        if (std::holds_alternative<instruction::CableLeft>(instr)) {
            auto& cable = std::get<instruction::CableLeft>(instr);
            ASSERT_EQ(cable.hold, 2u);
            ASSERT_EQ(cable.cross, 2u);
            found_cable = true;
            break;
        }
    }
    ASSERT_TRUE(found_cable);
}

// ============== Integration Tests ==============

void test_integration_stockinette() {
    std::cout << "  test_integration_stockinette...\n";

    Tokenizer tokenizer(
        "Cast on 20 sts.\n"
        "Row 1 (RS): K20.\n"
        "Row 2 (WS): P20.\n"
        "Row 3 (RS): K20.\n"
        "Row 4 (WS): P20.\n"
        "Bind off all sts."
    );
    Parser parser(std::move(tokenizer));
    auto ast = parser.parse();
    ASSERT_TRUE(!parser.has_errors());

    Emitter emitter;
    auto pattern = emitter.emit(ast);

    ASSERT_EQ(pattern.rows.size(), 6u);

    // Verify RS rows are all knit
    for (const auto& st : pattern.rows[1].stitches) {
        ASSERT_TRUE(std::holds_alternative<instruction::Knit>(st));
    }
    ASSERT_EQ(pattern.rows[1].side, RowSide::RS);

    // Verify WS rows are all purl
    for (const auto& st : pattern.rows[2].stitches) {
        ASSERT_TRUE(std::holds_alternative<instruction::Purl>(st));
    }
    ASSERT_EQ(pattern.rows[2].side, RowSide::WS);
}

void test_integration_ribbing() {
    std::cout << "  test_integration_ribbing...\n";

    Tokenizer tokenizer(
        "Cast on 20 sts.\n"
        "Row 1 (RS): *K2, P2*, rep from * to end."
    );
    Parser parser(std::move(tokenizer));
    auto ast = parser.parse();
    ASSERT_TRUE(!parser.has_errors());

    Emitter emitter;
    auto pattern = emitter.emit(ast);

    // Verify ribbing pattern: K K P P repeated
    size_t idx = 0;
    for (const auto& st : pattern.rows[1].stitches) {
        if (idx % 4 < 2) {
            ASSERT_TRUE(std::holds_alternative<instruction::Knit>(st));
        } else {
            ASSERT_TRUE(std::holds_alternative<instruction::Purl>(st));
        }
        idx++;
    }
}

void test_integration_lace() {
    std::cout << "  test_integration_lace...\n";

    Tokenizer tokenizer(
        "Cast on 20 sts.\n"
        "Row 1 (RS): K1, YO, SSK, K14, K2tog, YO, K1."
    );
    Parser parser(std::move(tokenizer));
    auto ast = parser.parse();
    ASSERT_TRUE(!parser.has_errors());

    Emitter emitter;
    auto pattern = emitter.emit(ast);

    // Count should balance: 20 consumed, 20 produced
    uint32_t consumed = 0;
    uint32_t produced = 0;
    for (const auto& instr : pattern.rows[1].stitches) {
        consumed += stitches_consumed(instr);
        produced += stitches_produced(instr);
    }
    ASSERT_EQ(consumed, 20u);
    ASSERT_EQ(produced, 20u);
}

}  // namespace

void test_parser() {
    // Tokenizer tests
    test_tokenizer_basic_tokens();
    test_tokenizer_numbers();
    test_tokenizer_multi_word_keywords();
    test_tokenizer_case_insensitivity();
    test_tokenizer_cables();
    test_tokenizer_punctuation();
    test_tokenizer_rep_from();
    test_tokenizer_all_stitches();

    // Parser tests
    test_parser_simple_row();
    test_parser_cast_on();
    test_parser_bind_off();
    test_parser_asterisk_repeat();
    test_parser_bracket_repeat();
    test_parser_multiple_rows();

    // Emitter tests
    test_emitter_simple_pattern();
    test_emitter_repeat_to_end();
    test_emitter_repeat_to_last();
    test_emitter_bracket_repeat();
    test_emitter_decreases();
    test_emitter_increases();
    test_emitter_cables();

    // Integration tests
    test_integration_stockinette();
    test_integration_ribbing();
    test_integration_lace();
}
