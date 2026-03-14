#ifndef YARNPATH_ROW_INSTRUCTION_HPP
#define YARNPATH_ROW_INSTRUCTION_HPP

#include "stitch_instruction.hpp"
#include <optional>
#include <vector>

namespace yarnpath {

/// Side of the fabric being worked.
/// RS (right side) = left-to-right in fabric coordinates.
/// WS (wrong side) = right-to-left in fabric coordinates.
/// The graph builder reverses live stitches before/after WS rows to model this.
enum class RowSide { RS, WS };

/// A single row's instructions: the side and a flat sequence of stitch operations.
struct RowInstruction {
    RowSide side;
    std::vector<StitchInstruction> stitches;
};

/// A complete pattern as a flat sequence of rows for a single panel.
/// This is the intermediate representation between the emitter and the graph builder.
struct PatternInstructions {
    std::vector<RowInstruction> rows;
    std::optional<uint32_t> panel_id;
};

}  // namespace yarnpath

#endif // YARNPATH_ROW_INSTRUCTION_HPP
