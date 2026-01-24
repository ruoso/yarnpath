#ifndef YARNPATH_TEST_HELPERS_HPP
#define YARNPATH_TEST_HELPERS_HPP

#include "stitch_instruction.hpp"
#include "row_instruction.hpp"
#include <string>
#include <vector>

namespace yarnpath {
namespace test {

// Helper to create a simple pattern from string notation
// C = CastOn, K = Knit, P = Purl, B = BindOff, O = YarnOver, 2 = K2tog, S = SSK
inline PatternInstructions create_pattern(const std::vector<std::string>& rows) {
    PatternInstructions pattern;
    for (size_t i = 0; i < rows.size(); ++i) {
        RowInstruction row;
        row.side = (i % 2 == 0) ? RowSide::RS : RowSide::WS;

        // Parse simple notation
        for (char c : rows[i]) {
            if (c == 'K') {
                row.stitches.push_back(instruction::Knit{});
            } else if (c == 'P') {
                row.stitches.push_back(instruction::Purl{});
            } else if (c == 'C') {
                row.stitches.push_back(instruction::CastOn{1});
            } else if (c == 'B') {
                row.stitches.push_back(instruction::BindOff{1});
            } else if (c == 'O') {
                row.stitches.push_back(instruction::YarnOver{});
            } else if (c == '2') {
                row.stitches.push_back(instruction::K2tog{});
            } else if (c == 'S') {
                row.stitches.push_back(instruction::SSK{});
            }
        }
        pattern.rows.push_back(row);
    }
    return pattern;
}

}  // namespace test
}  // namespace yarnpath

#endif // YARNPATH_TEST_HELPERS_HPP
