#ifndef YARNPATH_PARSER_PARSER_HPP
#define YARNPATH_PARSER_PARSER_HPP

#include "tokenizer.hpp"
#include "ast.hpp"
#include <string>
#include <vector>

namespace yarnpath {
namespace parser {

class Parser {
public:
    explicit Parser(Tokenizer tokenizer);

    ast::PatternAST parse();

    const std::vector<std::string>& errors() const { return errors_; }
    bool has_errors() const { return !errors_.empty(); }

private:
    Tokenizer tokenizer_;
    Token current_;
    std::vector<std::string> errors_;

    void advance();
    bool check(TokenType type) const;
    bool match(TokenType type);
    void expect(TokenType type, const std::string& message);
    void skip_to_next_line();
    void error(const std::string& message);

    ast::PatternNode parse_line();
    ast::RowNode parse_row();
    ast::CastOnNode parse_cast_on();
    ast::BindOffNode parse_bind_off();
    std::vector<ast::RowElement> parse_stitch_sequence();
    ast::RepeatNode parse_asterisk_repeat();
    ast::RepeatNode parse_bracket_repeat();
    std::optional<ast::StitchNode> parse_stitch();
    StitchInstruction token_to_instruction(const Token& token);
};

}  // namespace parser
}  // namespace yarnpath

#endif // YARNPATH_PARSER_PARSER_HPP
