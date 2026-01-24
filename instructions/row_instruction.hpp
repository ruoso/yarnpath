#ifndef YARNPATH_ROW_INSTRUCTION_HPP
#define YARNPATH_ROW_INSTRUCTION_HPP

#include "stitch_instruction.hpp"
#include <optional>
#include <vector>

namespace yarnpath {

enum class RowSide { RS, WS };

struct RowInstruction {
    RowSide side;
    std::vector<StitchInstruction> stitches;
};

struct PatternInstructions {
    std::vector<RowInstruction> rows;
    std::optional<uint32_t> panel_id;
};

}  // namespace yarnpath

#endif // YARNPATH_ROW_INSTRUCTION_HPP
