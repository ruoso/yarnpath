#ifndef YARNPATH_PARSER_TOKEN_HPP
#define YARNPATH_PARSER_TOKEN_HPP

#include <cstdint>
#include <optional>
#include <string>

namespace yarnpath {
namespace parser {

enum class TokenType {
    // Keywords
    CastOn, BindOff, Row, RS, WS,
    RepFrom, To, End, Last, Across, All, Sts,
    Times,

    // Stitches
    Knit, Purl, YarnOver, M1L, M1R, KFB,
    K2tog, SSK, S2KP, Slip,

    // Cables
    CableLeft, CableRight,  // C4L, C4R, etc.

    // Structure
    Number, Comma, Period, Colon,
    Asterisk, LParen, RParen, LBracket, RBracket,

    // Special
    EndOfLine, EndOfFile, Unknown
};

struct Token {
    TokenType type;
    std::string text;
    uint32_t line;
    uint32_t column;
    std::optional<uint32_t> number;  // For Number tokens
    // For cable tokens: hold and cross counts
    std::optional<uint8_t> cable_hold;
    std::optional<uint8_t> cable_cross;
};

}  // namespace parser
}  // namespace yarnpath

#endif // YARNPATH_PARSER_TOKEN_HPP
