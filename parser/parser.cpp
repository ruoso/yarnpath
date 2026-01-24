#include "parser.hpp"
#include <sstream>

namespace yarnpath {
namespace parser {

Parser::Parser(Tokenizer tokenizer) : tokenizer_(std::move(tokenizer)) {
    advance();
}

void Parser::advance() {
    current_ = tokenizer_.next();
}

bool Parser::check(TokenType type) const {
    return current_.type == type;
}

bool Parser::match(TokenType type) {
    if (check(type)) {
        advance();
        return true;
    }
    return false;
}

void Parser::expect(TokenType type, const std::string& message) {
    if (!match(type)) {
        error(message);
    }
}

void Parser::error(const std::string& message) {
    std::ostringstream oss;
    oss << message << " at line " << current_.line << ", column " << current_.column;
    if (!current_.text.empty()) {
        oss << " (got '" << current_.text << "')";
    }
    errors_.push_back(oss.str());
}

void Parser::skip_to_next_line() {
    while (!check(TokenType::EndOfLine) && !check(TokenType::EndOfFile)) {
        advance();
    }
    if (check(TokenType::EndOfLine)) {
        advance();
    }
}

ast::PatternAST Parser::parse() {
    ast::PatternAST ast;

    while (!check(TokenType::EndOfFile)) {
        // Skip empty lines
        while (match(TokenType::EndOfLine)) {}

        if (check(TokenType::EndOfFile)) {
            break;
        }

        ast.nodes.push_back(parse_line());
    }

    return ast;
}

ast::PatternNode Parser::parse_line() {
    if (check(TokenType::CastOn)) {
        return parse_cast_on();
    }

    if (check(TokenType::BindOff)) {
        return parse_bind_off();
    }

    if (check(TokenType::Row)) {
        return parse_row();
    }

    // Default: try to parse as a row without "Row N:" prefix
    // This handles lines like "Purl across."
    ast::RowNode row;
    row.side = RowSide::RS;  // Default to RS if not specified
    row.elements = parse_stitch_sequence();

    // Consume end of line
    match(TokenType::Period);
    match(TokenType::EndOfLine);

    return row;
}

ast::CastOnNode Parser::parse_cast_on() {
    advance();  // consume CastOn

    uint32_t count = 0;
    if (check(TokenType::Number)) {
        count = *current_.number;
        advance();
    } else {
        error("Expected number after 'Cast on'");
    }

    // Optional "sts" or "stitches"
    match(TokenType::Sts);

    // Consume period and end of line
    match(TokenType::Period);
    match(TokenType::EndOfLine);

    return ast::CastOnNode{count};
}

ast::BindOffNode Parser::parse_bind_off() {
    advance();  // consume BindOff

    ast::BindOffNode node;

    if (match(TokenType::All)) {
        node.all = true;
        match(TokenType::Sts);
    } else if (check(TokenType::Number)) {
        node.count = *current_.number;
        advance();
        match(TokenType::Sts);
    } else {
        node.all = true;  // Default to all
    }

    match(TokenType::Period);
    match(TokenType::EndOfLine);

    return node;
}

ast::RowNode Parser::parse_row() {
    ast::RowNode row;

    advance();  // consume Row

    // Row number
    if (check(TokenType::Number)) {
        row.row_number = *current_.number;
        advance();
    }

    // Optional side indicator (RS) or (WS)
    if (match(TokenType::LParen)) {
        if (match(TokenType::RS)) {
            row.side = RowSide::RS;
        } else if (match(TokenType::WS)) {
            row.side = RowSide::WS;
        }
        expect(TokenType::RParen, "Expected ')' after side indicator");
    }

    // Colon
    expect(TokenType::Colon, "Expected ':' after row header");

    // Parse stitch sequence
    row.elements = parse_stitch_sequence();

    // Consume period and end of line
    match(TokenType::Period);
    match(TokenType::EndOfLine);

    return row;
}

std::vector<ast::RowElement> Parser::parse_stitch_sequence() {
    std::vector<ast::RowElement> elements;

    while (!check(TokenType::Period) && !check(TokenType::EndOfLine) &&
           !check(TokenType::EndOfFile) && !check(TokenType::RBracket) &&
           !check(TokenType::Asterisk)) {

        // Skip commas
        match(TokenType::Comma);

        // After consuming comma, we might be at an asterisk - break to handle it
        if (check(TokenType::Asterisk)) {
            break;
        }

        if (check(TokenType::LBracket)) {
            elements.push_back(parse_bracket_repeat());
            continue;
        }

        auto stitch = parse_stitch();
        if (stitch) {
            elements.push_back(*stitch);
        } else if (!check(TokenType::Period) && !check(TokenType::EndOfLine) &&
                   !check(TokenType::EndOfFile) && !check(TokenType::RBracket)) {
            // Skip unexpected token
            advance();
        }
    }

    // Check for asterisk repeat after the sequence
    // Pattern: elements... *body*, rep from * to ...
    if (check(TokenType::Asterisk)) {
        auto repeat = parse_asterisk_repeat();
        // Insert the collected elements before the repeat as prefix
        // The repeat body comes from inside the asterisks
        elements.push_back(repeat);
    }

    return elements;
}

ast::RepeatNode Parser::parse_asterisk_repeat() {
    ast::RepeatNode repeat;

    advance();  // consume opening *

    // Parse body until closing *
    while (!check(TokenType::Asterisk) && !check(TokenType::Period) &&
           !check(TokenType::EndOfLine) && !check(TokenType::EndOfFile)) {

        match(TokenType::Comma);

        if (check(TokenType::LBracket)) {
            repeat.body.push_back(parse_bracket_repeat());
            continue;
        }

        auto stitch = parse_stitch();
        if (stitch) {
            repeat.body.push_back(*stitch);
        } else {
            break;
        }
    }

    expect(TokenType::Asterisk, "Expected closing '*'");

    // Skip comma after repeat
    match(TokenType::Comma);

    // Parse repeat instruction: "rep from * to ..."
    if (match(TokenType::RepFrom)) {
        match(TokenType::Asterisk);  // * in "rep from *"
        expect(TokenType::To, "Expected 'to' after 'rep from *'");

        if (match(TokenType::End)) {
            repeat.to_end = true;
        } else if (match(TokenType::Last)) {
            if (check(TokenType::Number)) {
                repeat.to_last = *current_.number;
                advance();
                match(TokenType::Sts);
            }
        } else if (check(TokenType::Number)) {
            // "to last N sts" without explicit "last"
            repeat.to_last = *current_.number;
            advance();
            match(TokenType::Sts);
        }
    }

    // Check for suffix stitches after the repeat
    match(TokenType::Comma);

    return repeat;
}

ast::RepeatNode Parser::parse_bracket_repeat() {
    ast::RepeatNode repeat;

    advance();  // consume [

    // Parse body until ]
    while (!check(TokenType::RBracket) && !check(TokenType::EndOfFile)) {
        match(TokenType::Comma);

        if (check(TokenType::LBracket)) {
            repeat.body.push_back(parse_bracket_repeat());
            continue;
        }

        auto stitch = parse_stitch();
        if (stitch) {
            repeat.body.push_back(*stitch);
        } else {
            break;
        }
    }

    expect(TokenType::RBracket, "Expected closing ']'");

    // Parse repeat count: "N times" or just a number
    if (check(TokenType::Number)) {
        repeat.times = *current_.number;
        advance();
        match(TokenType::Times);
    }

    return repeat;
}

std::optional<ast::StitchNode> Parser::parse_stitch() {
    ast::StitchNode node;

    switch (current_.type) {
        case TokenType::Knit:
            node.instruction = instruction::Knit{};
            node.count = current_.number.value_or(1);
            advance();
            return node;

        case TokenType::Purl:
            node.instruction = instruction::Purl{};
            node.count = current_.number.value_or(1);
            advance();
            return node;

        case TokenType::YarnOver:
            node.instruction = instruction::YarnOver{};
            node.count = 1;
            advance();
            return node;

        case TokenType::M1L:
            node.instruction = instruction::M1L{};
            node.count = 1;
            advance();
            return node;

        case TokenType::M1R:
            node.instruction = instruction::M1R{};
            node.count = 1;
            advance();
            return node;

        case TokenType::KFB:
            node.instruction = instruction::KFB{};
            node.count = 1;
            advance();
            return node;

        case TokenType::K2tog:
            node.instruction = instruction::K2tog{};
            node.count = 1;
            advance();
            return node;

        case TokenType::SSK:
            node.instruction = instruction::SSK{};
            node.count = 1;
            advance();
            return node;

        case TokenType::S2KP:
            node.instruction = instruction::S2KP{};
            node.count = 1;
            advance();
            return node;

        case TokenType::Slip:
            node.instruction = instruction::Slip{};
            node.count = 1;
            advance();
            return node;

        case TokenType::CableLeft:
            node.instruction = instruction::CableLeft{
                current_.cable_hold.value_or(2),
                current_.cable_cross.value_or(2)
            };
            node.count = 1;
            advance();
            return node;

        case TokenType::CableRight:
            node.instruction = instruction::CableRight{
                current_.cable_hold.value_or(2),
                current_.cable_cross.value_or(2)
            };
            node.count = 1;
            advance();
            return node;

        case TokenType::Across:
            // "Knit across" or "Purl across" - handled by emitter
            advance();
            return std::nullopt;

        default:
            return std::nullopt;
    }
}

StitchInstruction Parser::token_to_instruction(const Token& token) {
    switch (token.type) {
        case TokenType::Knit: return instruction::Knit{};
        case TokenType::Purl: return instruction::Purl{};
        case TokenType::YarnOver: return instruction::YarnOver{};
        case TokenType::M1L: return instruction::M1L{};
        case TokenType::M1R: return instruction::M1R{};
        case TokenType::KFB: return instruction::KFB{};
        case TokenType::K2tog: return instruction::K2tog{};
        case TokenType::SSK: return instruction::SSK{};
        case TokenType::S2KP: return instruction::S2KP{};
        case TokenType::Slip: return instruction::Slip{};
        case TokenType::CableLeft:
            return instruction::CableLeft{
                token.cable_hold.value_or(2),
                token.cable_cross.value_or(2)
            };
        case TokenType::CableRight:
            return instruction::CableRight{
                token.cable_hold.value_or(2),
                token.cable_cross.value_or(2)
            };
        default:
            return instruction::Knit{};  // Fallback
    }
}

}  // namespace parser
}  // namespace yarnpath
