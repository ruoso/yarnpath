#ifndef YARNPATH_TEST_HELPERS_HPP
#define YARNPATH_TEST_HELPERS_HPP

#include "stitch_instruction.hpp"
#include "row_instruction.hpp"
#include "../yarn/yarn_properties.hpp"
#include "../yarn/gauge.hpp"
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

// Default yarn properties for testing (typical worsted weight)
inline YarnProperties default_yarn() {
    YarnProperties yarn;
    yarn.relaxed_radius = 0.75f;      // mm
    yarn.compressed_radius = 0.5f;    // mm
    yarn.stiffness = 0.8f;            // relative stiffness
    yarn.elasticity = 0.3f;           // 30% max stretch
    yarn.tension = 0.5f;              // medium tension
    return yarn;
}

// Default gauge for testing (US size 8 needles, 5mm diameter)
inline Gauge default_gauge() {
    return Gauge{5.0f};  // 5mm needle diameter
}

}  // namespace test
}  // namespace yarnpath

#endif // YARNPATH_TEST_HELPERS_HPP
