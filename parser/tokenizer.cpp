#include "tokenizer.hpp"
#include <cctype>
#include <algorithm>

namespace yarnpath {
namespace parser {

Tokenizer::Tokenizer(std::string_view input) : input_(input) {}

bool Tokenizer::at_end() const {
    return pos_ >= input_.size();
}

char Tokenizer::current() const {
    if (at_end()) return '\0';
    return input_[pos_];
}

char Tokenizer::peek_char(size_t offset) const {
    if (pos_ + offset >= input_.size()) return '\0';
    return input_[pos_ + offset];
}

void Tokenizer::advance() {
    if (!at_end()) {
        if (current() == '\n') {
            line_++;
            column_ = 1;
        } else {
            column_++;
        }
        pos_++;
    }
}

bool Tokenizer::match(char c) {
    if (current() == c) {
        advance();
        return true;
    }
    return false;
}

bool Tokenizer::match_word(std::string_view word) {
    if (pos_ + word.size() > input_.size()) return false;

    for (size_t i = 0; i < word.size(); ++i) {
        if (std::tolower(input_[pos_ + i]) != std::tolower(word[i])) {
            return false;
        }
    }

    // Make sure word boundary (not followed by alphanumeric)
    size_t end_pos = pos_ + word.size();
    if (end_pos < input_.size() && std::isalnum(input_[end_pos])) {
        return false;
    }

    for (size_t i = 0; i < word.size(); ++i) {
        advance();
    }
    return true;
}

void Tokenizer::skip_whitespace() {
    while (!at_end()) {
        char c = current();
        if (c == ' ' || c == '\t' || c == '\r') {
            advance();
        } else {
            break;
        }
    }
}

Token Tokenizer::make_token(TokenType type) {
    return Token{type, "", line_, column_, std::nullopt, std::nullopt, std::nullopt};
}

Token Tokenizer::make_token(TokenType type, uint32_t number) {
    return Token{type, std::to_string(number), line_, column_, number, std::nullopt, std::nullopt};
}

Token Tokenizer::make_cable_token(TokenType type, uint8_t hold, uint8_t cross) {
    return Token{type, "", line_, column_, std::nullopt, hold, cross};
}

Token Tokenizer::peek() {
    if (!peeked_) {
        peeked_ = next();
    }
    return *peeked_;
}

Token Tokenizer::next() {
    if (peeked_) {
        Token t = *peeked_;
        peeked_.reset();
        return t;
    }
    return scan_token();
}

Token Tokenizer::scan_token() {
    skip_whitespace();

    if (at_end()) {
        return make_token(TokenType::EndOfFile);
    }

    uint32_t start_line = line_;
    uint32_t start_column = column_;

    char c = current();

    // Single character tokens
    switch (c) {
        case '\n':
            advance();
            return Token{TokenType::EndOfLine, "\n", start_line, start_column, std::nullopt, std::nullopt, std::nullopt};
        case ',':
            advance();
            return Token{TokenType::Comma, ",", start_line, start_column, std::nullopt, std::nullopt, std::nullopt};
        case '.':
            advance();
            return Token{TokenType::Period, ".", start_line, start_column, std::nullopt, std::nullopt, std::nullopt};
        case ':':
            advance();
            return Token{TokenType::Colon, ":", start_line, start_column, std::nullopt, std::nullopt, std::nullopt};
        case '*':
            advance();
            return Token{TokenType::Asterisk, "*", start_line, start_column, std::nullopt, std::nullopt, std::nullopt};
        case '(':
            advance();
            return Token{TokenType::LParen, "(", start_line, start_column, std::nullopt, std::nullopt, std::nullopt};
        case ')':
            advance();
            return Token{TokenType::RParen, ")", start_line, start_column, std::nullopt, std::nullopt, std::nullopt};
        case '[':
            advance();
            return Token{TokenType::LBracket, "[", start_line, start_column, std::nullopt, std::nullopt, std::nullopt};
        case ']':
            advance();
            return Token{TokenType::RBracket, "]", start_line, start_column, std::nullopt, std::nullopt, std::nullopt};
    }

    // Numbers
    if (std::isdigit(c)) {
        return scan_number();
    }

    // Words and keywords
    if (std::isalpha(c)) {
        return scan_word();
    }

    // Unknown character
    std::string text(1, c);
    advance();
    return Token{TokenType::Unknown, text, start_line, start_column, std::nullopt, std::nullopt, std::nullopt};
}

Token Tokenizer::scan_number() {
    uint32_t start_line = line_;
    uint32_t start_column = column_;

    std::string text;
    while (!at_end() && std::isdigit(current())) {
        text += current();
        advance();
    }

    uint32_t value = static_cast<uint32_t>(std::stoul(text));
    return Token{TokenType::Number, text, start_line, start_column, value, std::nullopt, std::nullopt};
}

Token Tokenizer::scan_word() {
    uint32_t start_line = line_;
    uint32_t start_column = column_;
    size_t start_pos = pos_;

    // Collect the word
    std::string word;
    while (!at_end() && (std::isalnum(current()) || current() == '/')) {
        word += current();
        advance();
    }

    // Convert to lowercase for comparison
    std::string lower_word = word;
    std::transform(lower_word.begin(), lower_word.end(), lower_word.begin(), ::tolower);

    // Check for multi-word keywords first (reset position and re-scan)
    pos_ = start_pos;
    line_ = start_line;
    column_ = start_column;

    if (match_word("cast")) {
        skip_whitespace();
        if (match_word("on")) {
            return Token{TokenType::CastOn, "cast on", start_line, start_column, std::nullopt, std::nullopt, std::nullopt};
        }
        // Reset and fall through
        pos_ = start_pos;
        line_ = start_line;
        column_ = start_column;
    }

    if (match_word("bind")) {
        skip_whitespace();
        if (match_word("off")) {
            return Token{TokenType::BindOff, "bind off", start_line, start_column, std::nullopt, std::nullopt, std::nullopt};
        }
        pos_ = start_pos;
        line_ = start_line;
        column_ = start_column;
    }

    if (match_word("rep")) {
        skip_whitespace();
        if (match_word("from")) {
            return Token{TokenType::RepFrom, "rep from", start_line, start_column, std::nullopt, std::nullopt, std::nullopt};
        }
        pos_ = start_pos;
        line_ = start_line;
        column_ = start_column;
    }

    // Re-consume the word
    word.clear();
    while (!at_end() && (std::isalnum(current()) || current() == '/')) {
        word += current();
        advance();
    }

    lower_word = word;
    std::transform(lower_word.begin(), lower_word.end(), lower_word.begin(), ::tolower);

    // Check special stitch patterns

    // S2KP / SK2P / s2kp / sk2p
    if (lower_word == "s2kp" || lower_word == "sk2p") {
        return Token{TokenType::S2KP, word, start_line, start_column, std::nullopt, std::nullopt, std::nullopt};
    }

    // K2tog
    if (lower_word == "k2tog") {
        return Token{TokenType::K2tog, word, start_line, start_column, std::nullopt, std::nullopt, std::nullopt};
    }

    // SSK
    if (lower_word == "ssk") {
        return Token{TokenType::SSK, word, start_line, start_column, std::nullopt, std::nullopt, std::nullopt};
    }

    // Cable patterns: C4L, C6R, C2/4L, C2/4R
    if (lower_word.size() >= 3 && lower_word[0] == 'c') {
        // Check for cable pattern
        size_t i = 1;
        uint8_t first_num = 0;
        while (i < lower_word.size() && std::isdigit(lower_word[i])) {
            first_num = first_num * 10 + (lower_word[i] - '0');
            i++;
        }

        if (first_num > 0) {
            uint8_t hold = 0, cross = 0;
            char direction = '\0';

            if (i < lower_word.size() && lower_word[i] == '/') {
                // C{H}/{C}L or C{H}/{C}R format
                i++;
                uint8_t second_num = 0;
                while (i < lower_word.size() && std::isdigit(lower_word[i])) {
                    second_num = second_num * 10 + (lower_word[i] - '0');
                    i++;
                }
                if (second_num > 0 && i < lower_word.size()) {
                    direction = lower_word[i];
                    hold = first_num;
                    cross = second_num;
                }
            } else if (i < lower_word.size()) {
                // C{N}L or C{N}R format
                direction = lower_word[i];
                hold = first_num / 2;
                cross = first_num - hold;
            }

            if (direction == 'l') {
                return Token{TokenType::CableLeft, word, start_line, start_column, std::nullopt, hold, cross};
            } else if (direction == 'r') {
                return Token{TokenType::CableRight, word, start_line, start_column, std::nullopt, hold, cross};
            }
        }
    }

    // K followed by optional number
    if (lower_word.size() >= 1 && lower_word[0] == 'k' &&
        (lower_word.size() == 1 || std::isdigit(lower_word[1]))) {
        uint32_t count = 1;
        if (lower_word.size() > 1) {
            count = static_cast<uint32_t>(std::stoul(lower_word.substr(1)));
        }
        return Token{TokenType::Knit, word, start_line, start_column, count, std::nullopt, std::nullopt};
    }

    // P followed by optional number
    if (lower_word.size() >= 1 && lower_word[0] == 'p' &&
        (lower_word.size() == 1 || std::isdigit(lower_word[1]))) {
        uint32_t count = 1;
        if (lower_word.size() > 1) {
            count = static_cast<uint32_t>(std::stoul(lower_word.substr(1)));
        }
        return Token{TokenType::Purl, word, start_line, start_column, count, std::nullopt, std::nullopt};
    }

    // YO / yo / YarnOver
    if (lower_word == "yo" || lower_word == "yarnover") {
        return Token{TokenType::YarnOver, word, start_line, start_column, std::nullopt, std::nullopt, std::nullopt};
    }

    // M1L / m1l
    if (lower_word == "m1l") {
        return Token{TokenType::M1L, word, start_line, start_column, std::nullopt, std::nullopt, std::nullopt};
    }

    // M1R / m1r
    if (lower_word == "m1r") {
        return Token{TokenType::M1R, word, start_line, start_column, std::nullopt, std::nullopt, std::nullopt};
    }

    // KFB / kfb
    if (lower_word == "kfb") {
        return Token{TokenType::KFB, word, start_line, start_column, std::nullopt, std::nullopt, std::nullopt};
    }

    // Sl / sl / Slip
    if (lower_word == "sl" || lower_word == "slip") {
        return Token{TokenType::Slip, word, start_line, start_column, std::nullopt, std::nullopt, std::nullopt};
    }

    // Keywords
    if (lower_word == "row") {
        return Token{TokenType::Row, word, start_line, start_column, std::nullopt, std::nullopt, std::nullopt};
    }
    if (lower_word == "rs") {
        return Token{TokenType::RS, word, start_line, start_column, std::nullopt, std::nullopt, std::nullopt};
    }
    if (lower_word == "ws") {
        return Token{TokenType::WS, word, start_line, start_column, std::nullopt, std::nullopt, std::nullopt};
    }
    if (lower_word == "to") {
        return Token{TokenType::To, word, start_line, start_column, std::nullopt, std::nullopt, std::nullopt};
    }
    if (lower_word == "end") {
        return Token{TokenType::End, word, start_line, start_column, std::nullopt, std::nullopt, std::nullopt};
    }
    if (lower_word == "last") {
        return Token{TokenType::Last, word, start_line, start_column, std::nullopt, std::nullopt, std::nullopt};
    }
    if (lower_word == "across") {
        return Token{TokenType::Across, word, start_line, start_column, std::nullopt, std::nullopt, std::nullopt};
    }
    if (lower_word == "all") {
        return Token{TokenType::All, word, start_line, start_column, std::nullopt, std::nullopt, std::nullopt};
    }
    if (lower_word == "sts" || lower_word == "st") {
        return Token{TokenType::Sts, word, start_line, start_column, std::nullopt, std::nullopt, std::nullopt};
    }
    if (lower_word == "times" || lower_word == "time") {
        return Token{TokenType::Times, word, start_line, start_column, std::nullopt, std::nullopt, std::nullopt};
    }
    if (lower_word == "purl") {
        return Token{TokenType::Purl, word, start_line, start_column, 1, std::nullopt, std::nullopt};
    }
    if (lower_word == "knit") {
        return Token{TokenType::Knit, word, start_line, start_column, 1, std::nullopt, std::nullopt};
    }

    // Unknown word
    return Token{TokenType::Unknown, word, start_line, start_column, std::nullopt, std::nullopt, std::nullopt};
}

}  // namespace parser
}  // namespace yarnpath
