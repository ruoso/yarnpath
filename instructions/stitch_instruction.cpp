#include "stitch_instruction.hpp"

namespace yarnpath {

uint32_t stitches_consumed(const StitchInstruction& instr) {
    return std::visit([](auto&& arg) -> uint32_t {
        using T = std::decay_t<decltype(arg)>;
        if constexpr (std::is_same_v<T, instruction::Knit>) return 1;
        else if constexpr (std::is_same_v<T, instruction::Purl>) return 1;
        else if constexpr (std::is_same_v<T, instruction::Slip>) return 1;
        else if constexpr (std::is_same_v<T, instruction::CastOn>) return 0;
        else if constexpr (std::is_same_v<T, instruction::BindOff>) return arg.count;
        else if constexpr (std::is_same_v<T, instruction::YarnOver>) return 0;
        else if constexpr (std::is_same_v<T, instruction::KFB>) return 1;
        else if constexpr (std::is_same_v<T, instruction::M1L>) return 0;
        else if constexpr (std::is_same_v<T, instruction::M1R>) return 0;
        else if constexpr (std::is_same_v<T, instruction::K2tog>) return 2;
        else if constexpr (std::is_same_v<T, instruction::SSK>) return 2;
        else if constexpr (std::is_same_v<T, instruction::S2KP>) return 3;
        else if constexpr (std::is_same_v<T, instruction::CableLeft>) return arg.hold + arg.cross;
        else if constexpr (std::is_same_v<T, instruction::CableRight>) return arg.hold + arg.cross;
        else if constexpr (std::is_same_v<T, instruction::Repeat>) {
            uint32_t total = 0;
            for (const auto& i : arg.instructions) {
                total += stitches_consumed(i);
            }
            return total * arg.times;
        }
        else return 0;
    }, instr);
}

uint32_t stitches_produced(const StitchInstruction& instr) {
    return std::visit([](auto&& arg) -> uint32_t {
        using T = std::decay_t<decltype(arg)>;
        if constexpr (std::is_same_v<T, instruction::Knit>) return 1;
        else if constexpr (std::is_same_v<T, instruction::Purl>) return 1;
        else if constexpr (std::is_same_v<T, instruction::Slip>) return 1;
        else if constexpr (std::is_same_v<T, instruction::CastOn>) return arg.count;
        else if constexpr (std::is_same_v<T, instruction::BindOff>) return 0;
        else if constexpr (std::is_same_v<T, instruction::YarnOver>) return 1;
        else if constexpr (std::is_same_v<T, instruction::KFB>) return 2;
        else if constexpr (std::is_same_v<T, instruction::M1L>) return 1;
        else if constexpr (std::is_same_v<T, instruction::M1R>) return 1;
        else if constexpr (std::is_same_v<T, instruction::K2tog>) return 1;
        else if constexpr (std::is_same_v<T, instruction::SSK>) return 1;
        else if constexpr (std::is_same_v<T, instruction::S2KP>) return 1;
        else if constexpr (std::is_same_v<T, instruction::CableLeft>) return arg.hold + arg.cross;
        else if constexpr (std::is_same_v<T, instruction::CableRight>) return arg.hold + arg.cross;
        else if constexpr (std::is_same_v<T, instruction::Repeat>) {
            uint32_t total = 0;
            for (const auto& i : arg.instructions) {
                total += stitches_produced(i);
            }
            return total * arg.times;
        }
        else return 0;
    }, instr);
}

std::vector<StitchInstruction> expand_instructions(const std::vector<StitchInstruction>& instructions) {
    std::vector<StitchInstruction> result;
    for (const auto& instr : instructions) {
        if (auto* repeat = std::get_if<instruction::Repeat>(&instr)) {
            auto expanded = expand_instructions(repeat->instructions);
            for (uint32_t i = 0; i < repeat->times; ++i) {
                result.insert(result.end(), expanded.begin(), expanded.end());
            }
        } else {
            result.push_back(instr);
        }
    }
    return result;
}

}  // namespace yarnpath
