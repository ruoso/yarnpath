#ifndef YARNPATH_STITCH_INSTRUCTION_HPP
#define YARNPATH_STITCH_INSTRUCTION_HPP

#include <cstdint>
#include <variant>
#include <vector>

namespace yarnpath {
namespace instruction {

/// Basic stitches: consume 1 live stitch, produce 1 new stitch.
struct Knit {};
struct Purl {};
struct Slip {};  ///< Pass stitch through without working it.

/// Structural instructions: operate on multiple stitches at once via `count`.
struct CastOn { uint32_t count; };   ///< Consume 0, produce `count` new stitches.
struct BindOff { uint32_t count; };  ///< Consume `count`, produce 0 (removes from live set).

/// Increases: produce more stitches than consumed.
struct YarnOver {};                    ///< Consume 0, produce 1 (wrap yarn around needle).
struct KFB {};                         ///< Consume 1, produce 2 (knit front and back).
struct M1L {};                         ///< Consume 0, produce 1 (lift bar, left-leaning).
struct M1R {};                         ///< Consume 0, produce 1 (lift bar, right-leaning).

/// Decreases: consume more stitches than produced.
struct K2tog {};                       ///< Consume 2, produce 1 (right-leaning decrease).
struct SSK {};                         ///< Consume 2, produce 1 (left-leaning decrease).
struct S2KP {};                        ///< Consume 3, produce 1 (centered double decrease).

/// Cables: consume hold+cross stitches, produce the same number, reordered.
/// `hold` and `cross` specify the number of stitches in each group.
struct CableLeft { uint8_t hold; uint8_t cross; };   ///< Hold to front, work crossed first.
struct CableRight { uint8_t hold; uint8_t cross; };  ///< Hold to back, work held first.

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

/// Repeat a sequence of instructions a fixed number of times.
/// Repeats are expanded into flat instruction sequences by expand_instructions()
/// before being passed to the graph builder.
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
