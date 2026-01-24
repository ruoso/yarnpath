#ifndef YARNPATH_PARSER_TOKENIZER_HPP
#define YARNPATH_PARSER_TOKENIZER_HPP

#include "token.hpp"
#include <string_view>

namespace yarnpath {
namespace parser {

class Tokenizer {
public:
    explicit Tokenizer(std::string_view input);

    Token next();           // Get next token
    Token peek();           // Look ahead without consuming
    bool at_end() const;

private:
    std::string_view input_;
    size_t pos_ = 0;
    uint32_t line_ = 1;
    uint32_t column_ = 1;
    std::optional<Token> peeked_;

    void skip_whitespace();
    void advance();
    char current() const;
    char peek_char(size_t offset = 1) const;
    bool match(char c);
    bool match_word(std::string_view word);

    Token scan_token();
    Token scan_number();
    Token scan_word();
    Token make_token(TokenType type);
    Token make_token(TokenType type, uint32_t number);
    Token make_cable_token(TokenType type, uint8_t hold, uint8_t cross);
};

}  // namespace parser
}  // namespace yarnpath

#endif // YARNPATH_PARSER_TOKENIZER_HPP
