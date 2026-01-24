#ifndef YARNPATH_STITCH_INSTRUCTION_HPP
#define YARNPATH_STITCH_INSTRUCTION_HPP

#include <cstdint>
#include <variant>
#include <vector>

namespace yarnpath {
namespace instruction {

// Basic stitches (consume 1, produce 1)
struct Knit {};
struct Purl {};
struct Slip {};  // Pass through without working

// Structural
struct CastOn { uint32_t count; };
struct BindOff { uint32_t count; };

// Increases (produce more than consume)
struct YarnOver {};                    // Consume 0, produce 1
struct KFB {};                         // Consume 1, produce 2
struct M1L {};                         // Consume 0, produce 1 (lift bar)
struct M1R {};                         // Consume 0, produce 1

// Decreases (consume more than produce)
struct K2tog {};                       // Consume 2, produce 1
struct SSK {};                         // Consume 2, produce 1
struct S2KP {};                        // Consume 3, produce 1

// Cables (consume N, produce N, but reordered)
struct CableLeft { uint8_t hold; uint8_t cross; };
struct CableRight { uint8_t hold; uint8_t cross; };

// Forward declaration for Repeat
struct Repeat;

}  // namespace instruction

// StitchInstruction variant - forward declared to allow Repeat to reference it
using StitchInstruction = std::variant<
    instruction::Knit, instruction::Purl, instruction::Slip,
    instruction::CastOn, instruction::BindOff,
    instruction::YarnOver, instruction::KFB, instruction::M1L, instruction::M1R,
    instruction::K2tog, instruction::SSK, instruction::S2KP,
    instruction::CableLeft, instruction::CableRight,
    instruction::Repeat
>;

namespace instruction {

// Repeat a sequence
struct Repeat {
    std::vector<StitchInstruction> instructions;
    uint32_t times;
};

}  // namespace instruction

// Helper functions for instruction properties
uint32_t stitches_consumed(const StitchInstruction& instr);
uint32_t stitches_produced(const StitchInstruction& instr);

// Expand repeats into flat instruction sequence
std::vector<StitchInstruction> expand_instructions(const std::vector<StitchInstruction>& instructions);

}  // namespace yarnpath

#endif // YARNPATH_STITCH_INSTRUCTION_HPP
