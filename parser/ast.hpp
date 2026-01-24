#ifndef YARNPATH_PARSER_AST_HPP
#define YARNPATH_PARSER_AST_HPP

#include "row_instruction.hpp"
#include <cstdint>
#include <optional>
#include <string>
#include <variant>
#include <vector>

namespace yarnpath {
namespace parser {
namespace ast {

// Forward declarations
struct RepeatNode;

struct StitchNode {
    StitchInstruction instruction;
    uint32_t count = 1;  // K3 = Knit with count 3
};

// Element within a row: either a stitch or a nested repeat
using RowElement = std::variant<StitchNode, RepeatNode>;

struct RepeatNode {
    std::vector<RowElement> body;
    std::optional<uint32_t> times;        // [K2, P2] 3 times
    bool to_end = false;                  // rep from * to end
    std::optional<uint32_t> to_last;      // rep from * to last 3 sts
};

struct RowNode {
    std::optional<uint32_t> row_number;
    std::optional<RowSide> side;
    std::vector<RowElement> elements;
};

struct CastOnNode {
    uint32_t count;
};

struct BindOffNode {
    std::optional<uint32_t> count;
    bool all = false;
};

using PatternNode = std::variant<CastOnNode, BindOffNode, RowNode>;

struct PatternAST {
    std::vector<PatternNode> nodes;
};

}  // namespace ast
}  // namespace parser
}  // namespace yarnpath

#endif // YARNPATH_PARSER_AST_HPP
