#ifndef YARNPATH_PARSER_EMITTER_HPP
#define YARNPATH_PARSER_EMITTER_HPP

#include "ast.hpp"
#include "row_instruction.hpp"
#include <string>
#include <vector>

namespace yarnpath {
namespace parser {

class Emitter {
public:
    PatternInstructions emit(const ast::PatternAST& ast);

    RowInstruction emit_row(const ast::RowNode& row, uint32_t live_stitch_count);

    const std::vector<std::string>& warnings() const { return warnings_; }
    const std::vector<std::string>& errors() const { return errors_; }
    bool has_errors() const { return !errors_.empty(); }

private:
    std::vector<std::string> warnings_;
    std::vector<std::string> errors_;

    std::vector<StitchInstruction> emit_elements(
        const std::vector<ast::RowElement>& elements,
        uint32_t available_stitches,
        bool fill_to_end = false
    );

    std::vector<StitchInstruction> emit_stitch_node(
        const ast::StitchNode& node
    );

    std::vector<StitchInstruction> emit_repeat(
        const ast::RepeatNode& repeat,
        uint32_t remaining_stitches
    );

    uint32_t calculate_consumption(const std::vector<ast::RowElement>& elements);
    uint32_t calculate_element_consumption(const ast::RowElement& element);
    uint32_t stitch_consumption(const StitchInstruction& instr);
};

}  // namespace parser
}  // namespace yarnpath

#endif // YARNPATH_PARSER_EMITTER_HPP
