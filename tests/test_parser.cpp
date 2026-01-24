#include "tokenizer.hpp"
#include "parser.hpp"
#include "emitter.hpp"
#include "stitch_instruction.hpp"
#include "row_instruction.hpp"
#include <gtest/gtest.h>

using namespace yarnpath;
using namespace yarnpath::parser;

// ============== Tokenizer Tests ==============

TEST(Tokenizer, BasicTokens) {
    Tokenizer tokenizer("K P K2tog");

    Token t1 = tokenizer.next();
    EXPECT_EQ(t1.type, TokenType::Knit);
    EXPECT_EQ(t1.number.value_or(0), 1u);

    Token t2 = tokenizer.next();
    EXPECT_EQ(t2.type, TokenType::Purl);
    EXPECT_EQ(t2.number.value_or(0), 1u);

    Token t3 = tokenizer.next();
    EXPECT_EQ(t3.type, TokenType::K2tog);
}

TEST(Tokenizer, Numbers) {
    Tokenizer tokenizer("K3 P5 42");

    Token t1 = tokenizer.next();
    EXPECT_EQ(t1.type, TokenType::Knit);
    EXPECT_EQ(t1.number.value_or(0), 3u);

    Token t2 = tokenizer.next();
    EXPECT_EQ(t2.type, TokenType::Purl);
    EXPECT_EQ(t2.number.value_or(0), 5u);

    Token t3 = tokenizer.next();
    EXPECT_EQ(t3.type, TokenType::Number);
    EXPECT_EQ(t3.number.value_or(0), 42u);
}

TEST(Tokenizer, MultiWordKeywords) {
    Tokenizer tokenizer("Cast on 20 sts. Bind off all sts.");

    EXPECT_EQ(tokenizer.next().type, TokenType::CastOn);
    EXPECT_EQ(tokenizer.next().type, TokenType::Number);
    EXPECT_EQ(tokenizer.next().type, TokenType::Sts);
    EXPECT_EQ(tokenizer.next().type, TokenType::Period);
    EXPECT_EQ(tokenizer.next().type, TokenType::BindOff);
    EXPECT_EQ(tokenizer.next().type, TokenType::All);
    EXPECT_EQ(tokenizer.next().type, TokenType::Sts);
}

TEST(Tokenizer, CaseInsensitivity) {
    Tokenizer tokenizer("K k YO yo SSK ssk");

    Token t1 = tokenizer.next();
    EXPECT_EQ(t1.type, TokenType::Knit);

    Token t2 = tokenizer.next();
    EXPECT_EQ(t2.type, TokenType::Knit);

    Token t3 = tokenizer.next();
    EXPECT_EQ(t3.type, TokenType::YarnOver);

    Token t4 = tokenizer.next();
    EXPECT_EQ(t4.type, TokenType::YarnOver);

    Token t5 = tokenizer.next();
    EXPECT_EQ(t5.type, TokenType::SSK);

    Token t6 = tokenizer.next();
    EXPECT_EQ(t6.type, TokenType::SSK);
}

TEST(Tokenizer, Cables) {
    Tokenizer tokenizer("C4L C6R C2/4L");

    Token t1 = tokenizer.next();
    EXPECT_EQ(t1.type, TokenType::CableLeft);
    EXPECT_EQ(t1.cable_hold.value_or(0), 2u);
    EXPECT_EQ(t1.cable_cross.value_or(0), 2u);

    Token t2 = tokenizer.next();
    EXPECT_EQ(t2.type, TokenType::CableRight);
    EXPECT_EQ(t2.cable_hold.value_or(0), 3u);
    EXPECT_EQ(t2.cable_cross.value_or(0), 3u);

    Token t3 = tokenizer.next();
    EXPECT_EQ(t3.type, TokenType::CableLeft);
    EXPECT_EQ(t3.cable_hold.value_or(0), 2u);
    EXPECT_EQ(t3.cable_cross.value_or(0), 4u);
}

TEST(Tokenizer, Punctuation) {
    Tokenizer tokenizer("K, P. *K* [P] (RS):");

    EXPECT_EQ(tokenizer.next().type, TokenType::Knit);
    EXPECT_EQ(tokenizer.next().type, TokenType::Comma);
    EXPECT_EQ(tokenizer.next().type, TokenType::Purl);
    EXPECT_EQ(tokenizer.next().type, TokenType::Period);
    EXPECT_EQ(tokenizer.next().type, TokenType::Asterisk);
    EXPECT_EQ(tokenizer.next().type, TokenType::Knit);
    EXPECT_EQ(tokenizer.next().type, TokenType::Asterisk);
    EXPECT_EQ(tokenizer.next().type, TokenType::LBracket);
    EXPECT_EQ(tokenizer.next().type, TokenType::Purl);
    EXPECT_EQ(tokenizer.next().type, TokenType::RBracket);
    EXPECT_EQ(tokenizer.next().type, TokenType::LParen);
    EXPECT_EQ(tokenizer.next().type, TokenType::RS);
    EXPECT_EQ(tokenizer.next().type, TokenType::RParen);
    EXPECT_EQ(tokenizer.next().type, TokenType::Colon);
}

TEST(Tokenizer, RepFrom) {
    Tokenizer tokenizer("rep from * to end");

    EXPECT_EQ(tokenizer.next().type, TokenType::RepFrom);
    EXPECT_EQ(tokenizer.next().type, TokenType::Asterisk);
    EXPECT_EQ(tokenizer.next().type, TokenType::To);
    EXPECT_EQ(tokenizer.next().type, TokenType::End);
}

TEST(Tokenizer, AllStitches) {
    Tokenizer tokenizer("M1L M1R KFB Sl S2KP SK2P");

    EXPECT_EQ(tokenizer.next().type, TokenType::M1L);
    EXPECT_EQ(tokenizer.next().type, TokenType::M1R);
    EXPECT_EQ(tokenizer.next().type, TokenType::KFB);
    EXPECT_EQ(tokenizer.next().type, TokenType::Slip);
    EXPECT_EQ(tokenizer.next().type, TokenType::S2KP);
    EXPECT_EQ(tokenizer.next().type, TokenType::S2KP);
}

// ============== Parser Tests ==============

TEST(Parser, SimpleRow) {
    Tokenizer tokenizer("Row 1 (RS): K3, P3.");
    Parser parser(std::move(tokenizer));

    auto ast = parser.parse();
    ASSERT_FALSE(parser.has_errors());
    ASSERT_EQ(ast.nodes.size(), 1u);

    auto* row = std::get_if<ast::RowNode>(&ast.nodes[0]);
    ASSERT_NE(row, nullptr);
    EXPECT_EQ(row->row_number.value_or(0), 1u);
    EXPECT_EQ(row->side.value_or(RowSide::WS), RowSide::RS);
}

TEST(Parser, CastOn) {
    Tokenizer tokenizer("Cast on 20 sts.");
    Parser parser(std::move(tokenizer));

    auto ast = parser.parse();
    ASSERT_FALSE(parser.has_errors());
    ASSERT_EQ(ast.nodes.size(), 1u);

    auto* cast_on = std::get_if<ast::CastOnNode>(&ast.nodes[0]);
    ASSERT_NE(cast_on, nullptr);
    EXPECT_EQ(cast_on->count, 20u);
}

TEST(Parser, BindOff) {
    Tokenizer tokenizer("Bind off all sts.");
    Parser parser(std::move(tokenizer));

    auto ast = parser.parse();
    ASSERT_FALSE(parser.has_errors());
    ASSERT_EQ(ast.nodes.size(), 1u);

    auto* bind_off = std::get_if<ast::BindOffNode>(&ast.nodes[0]);
    ASSERT_NE(bind_off, nullptr);
    EXPECT_TRUE(bind_off->all);
}

TEST(Parser, AsteriskRepeat) {
    Tokenizer tokenizer("Row 1 (RS): K2, *K2, P2*, rep from * to end.");
    Parser parser(std::move(tokenizer));

    auto ast = parser.parse();
    ASSERT_FALSE(parser.has_errors());
    ASSERT_EQ(ast.nodes.size(), 1u);

    auto* row = std::get_if<ast::RowNode>(&ast.nodes[0]);
    ASSERT_NE(row, nullptr);
    // Should have K2 prefix and a repeat
    EXPECT_GE(row->elements.size(), 2u);
}

TEST(Parser, BracketRepeat) {
    Tokenizer tokenizer("Row 1: [K1, P1] 5 times.");
    Parser parser(std::move(tokenizer));

    auto ast = parser.parse();
    ASSERT_FALSE(parser.has_errors());
    ASSERT_EQ(ast.nodes.size(), 1u);

    auto* row = std::get_if<ast::RowNode>(&ast.nodes[0]);
    ASSERT_NE(row, nullptr);
    ASSERT_EQ(row->elements.size(), 1u);

    auto* repeat = std::get_if<ast::RepeatNode>(&row->elements[0]);
    ASSERT_NE(repeat, nullptr);
    EXPECT_EQ(repeat->times.value_or(0), 5u);
}

TEST(Parser, MultipleRows) {
    Tokenizer tokenizer(
        "Cast on 10 sts.\n"
        "Row 1 (RS): K10.\n"
        "Row 2 (WS): P10.\n"
        "Bind off all sts."
    );
    Parser parser(std::move(tokenizer));

    auto ast = parser.parse();
    ASSERT_FALSE(parser.has_errors());
    EXPECT_EQ(ast.nodes.size(), 4u);
}

// ============== Emitter Tests ==============

TEST(Emitter, SimplePattern) {
    Tokenizer tokenizer(
        "Cast on 10 sts.\n"
        "Row 1 (RS): K10.\n"
        "Bind off all sts."
    );
    Parser parser(std::move(tokenizer));
    auto ast = parser.parse();
    ASSERT_FALSE(parser.has_errors());

    Emitter emitter;
    auto pattern = emitter.emit(ast);

    ASSERT_EQ(pattern.rows.size(), 3u);

    // Cast on row
    ASSERT_EQ(pattern.rows[0].stitches.size(), 1u);
    EXPECT_TRUE(std::holds_alternative<instruction::CastOn>(pattern.rows[0].stitches[0]));
    EXPECT_EQ(std::get<instruction::CastOn>(pattern.rows[0].stitches[0]).count, 10u);

    // Knit row
    ASSERT_EQ(pattern.rows[1].stitches.size(), 10u);
    for (const auto& st : pattern.rows[1].stitches) {
        EXPECT_TRUE(std::holds_alternative<instruction::Knit>(st));
    }

    // Bind off row
    ASSERT_EQ(pattern.rows[2].stitches.size(), 1u);
    EXPECT_TRUE(std::holds_alternative<instruction::BindOff>(pattern.rows[2].stitches[0]));
}

TEST(Emitter, RepeatToEnd) {
    Tokenizer tokenizer(
        "Cast on 12 sts.\n"
        "Row 1 (RS): *K2, P2*, rep from * to end."
    );
    Parser parser(std::move(tokenizer));
    auto ast = parser.parse();
    ASSERT_FALSE(parser.has_errors());

    Emitter emitter;
    auto pattern = emitter.emit(ast);

    ASSERT_EQ(pattern.rows.size(), 2u);

    // Row 1: should be K2 P2 repeated 3 times = 12 stitches
    ASSERT_EQ(pattern.rows[1].stitches.size(), 12u);

    // Check pattern: K K P P K K P P K K P P
    EXPECT_TRUE(std::holds_alternative<instruction::Knit>(pattern.rows[1].stitches[0]));
    EXPECT_TRUE(std::holds_alternative<instruction::Knit>(pattern.rows[1].stitches[1]));
    EXPECT_TRUE(std::holds_alternative<instruction::Purl>(pattern.rows[1].stitches[2]));
    EXPECT_TRUE(std::holds_alternative<instruction::Purl>(pattern.rows[1].stitches[3]));
}

TEST(Emitter, RepeatToLast) {
    Tokenizer tokenizer(
        "Cast on 14 sts.\n"
        "Row 1 (RS): K2, *K2, P2*, rep from * to last 4 sts, K4."
    );
    Parser parser(std::move(tokenizer));
    auto ast = parser.parse();

    // This is a complex pattern - just verify it parses
    EXPECT_FALSE(parser.has_errors());
}

TEST(Emitter, BracketRepeat) {
    Tokenizer tokenizer(
        "Cast on 10 sts.\n"
        "Row 1: [K1, P1] 5 times."
    );
    Parser parser(std::move(tokenizer));
    auto ast = parser.parse();
    ASSERT_FALSE(parser.has_errors());

    Emitter emitter;
    auto pattern = emitter.emit(ast);

    ASSERT_EQ(pattern.rows.size(), 2u);

    // Row 1: should be K P repeated 5 times = 10 stitches
    ASSERT_EQ(pattern.rows[1].stitches.size(), 10u);

    // Check alternating pattern
    for (size_t i = 0; i < 10; ++i) {
        if (i % 2 == 0) {
            EXPECT_TRUE(std::holds_alternative<instruction::Knit>(pattern.rows[1].stitches[i]));
        } else {
            EXPECT_TRUE(std::holds_alternative<instruction::Purl>(pattern.rows[1].stitches[i]));
        }
    }
}

TEST(Emitter, Decreases) {
    Tokenizer tokenizer(
        "Cast on 10 sts.\n"
        "Row 1 (RS): K1, SSK, K4, K2tog, K1."
    );
    Parser parser(std::move(tokenizer));
    auto ast = parser.parse();
    ASSERT_FALSE(parser.has_errors());

    Emitter emitter;
    auto pattern = emitter.emit(ast);

    // Count stitches
    uint32_t consumed = 0;
    uint32_t produced = 0;
    for (const auto& instr : pattern.rows[1].stitches) {
        consumed += stitches_consumed(instr);
        produced += stitches_produced(instr);
    }
    EXPECT_EQ(consumed, 10u);  // Uses all 10 stitches
    EXPECT_EQ(produced, 8u);   // Produces 8 (two decreases)
}

TEST(Emitter, Increases) {
    Tokenizer tokenizer(
        "Cast on 10 sts.\n"
        "Row 1 (RS): K1, YO, K8, YO, K1."
    );
    Parser parser(std::move(tokenizer));
    auto ast = parser.parse();
    ASSERT_FALSE(parser.has_errors());

    Emitter emitter;
    auto pattern = emitter.emit(ast);

    // Count stitches
    uint32_t consumed = 0;
    uint32_t produced = 0;
    for (const auto& instr : pattern.rows[1].stitches) {
        consumed += stitches_consumed(instr);
        produced += stitches_produced(instr);
    }
    EXPECT_EQ(consumed, 10u);  // Uses all 10 stitches
    EXPECT_EQ(produced, 12u);  // Produces 12 (two yarn overs)
}

TEST(Emitter, Cables) {
    Tokenizer tokenizer(
        "Cast on 10 sts.\n"
        "Row 1 (RS): K3, C4L, K3."
    );
    Parser parser(std::move(tokenizer));
    auto ast = parser.parse();
    ASSERT_FALSE(parser.has_errors());

    Emitter emitter;
    auto pattern = emitter.emit(ast);

    // Find the cable instruction
    bool found_cable = false;
    for (const auto& instr : pattern.rows[1].stitches) {
        if (std::holds_alternative<instruction::CableLeft>(instr)) {
            auto& cable = std::get<instruction::CableLeft>(instr);
            EXPECT_EQ(cable.hold, 2u);
            EXPECT_EQ(cable.cross, 2u);
            found_cable = true;
            break;
        }
    }
    EXPECT_TRUE(found_cable);
}

// ============== Integration Tests ==============

TEST(Integration, Stockinette) {
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
    ASSERT_FALSE(parser.has_errors());

    Emitter emitter;
    auto pattern = emitter.emit(ast);

    ASSERT_EQ(pattern.rows.size(), 6u);

    // Verify RS rows are all knit
    for (const auto& st : pattern.rows[1].stitches) {
        EXPECT_TRUE(std::holds_alternative<instruction::Knit>(st));
    }
    EXPECT_EQ(pattern.rows[1].side, RowSide::RS);

    // Verify WS rows are all purl
    for (const auto& st : pattern.rows[2].stitches) {
        EXPECT_TRUE(std::holds_alternative<instruction::Purl>(st));
    }
    EXPECT_EQ(pattern.rows[2].side, RowSide::WS);
}

TEST(Integration, Ribbing) {
    Tokenizer tokenizer(
        "Cast on 20 sts.\n"
        "Row 1 (RS): *K2, P2*, rep from * to end."
    );
    Parser parser(std::move(tokenizer));
    auto ast = parser.parse();
    ASSERT_FALSE(parser.has_errors());

    Emitter emitter;
    auto pattern = emitter.emit(ast);

    // Verify ribbing pattern: K K P P repeated
    size_t idx = 0;
    for (const auto& st : pattern.rows[1].stitches) {
        if (idx % 4 < 2) {
            EXPECT_TRUE(std::holds_alternative<instruction::Knit>(st));
        } else {
            EXPECT_TRUE(std::holds_alternative<instruction::Purl>(st));
        }
        idx++;
    }
}

TEST(Integration, Lace) {
    Tokenizer tokenizer(
        "Cast on 20 sts.\n"
        "Row 1 (RS): K1, YO, SSK, K14, K2tog, YO, K1."
    );
    Parser parser(std::move(tokenizer));
    auto ast = parser.parse();
    ASSERT_FALSE(parser.has_errors());

    Emitter emitter;
    auto pattern = emitter.emit(ast);

    // Count should balance: 20 consumed, 20 produced
    uint32_t consumed = 0;
    uint32_t produced = 0;
    for (const auto& instr : pattern.rows[1].stitches) {
        consumed += stitches_consumed(instr);
        produced += stitches_produced(instr);
    }
    EXPECT_EQ(consumed, 20u);
    EXPECT_EQ(produced, 20u);
}
