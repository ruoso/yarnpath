#include "emitter.hpp"
#include "stitch_instruction.hpp"
#include "logging.hpp"
#include <sstream>
#include <stdexcept>

namespace yarnpath {
namespace parser {

PatternInstructions Emitter::emit(const ast::PatternAST& ast) {
    auto log = yarnpath::logging::get_logger();
    PatternInstructions pattern;
    uint32_t live_stitches = 0;

    log->debug("Emitter: processing {} AST nodes", ast.nodes.size());

    for (const auto& node : ast.nodes) {
        if (auto* cast_on = std::get_if<ast::CastOnNode>(&node)) {
            // Cast on creates the initial stitches
            RowInstruction row;
            row.side = RowSide::RS;
            row.stitches.push_back(instruction::CastOn{cast_on->count});
            pattern.rows.push_back(row);
            live_stitches = cast_on->count;
            log->debug("Emitter: cast-on {} stitches, live_stitches={}", cast_on->count, live_stitches);
        }
        else if (auto* bind_off = std::get_if<ast::BindOffNode>(&node)) {
            RowInstruction row;
            row.side = RowSide::RS;
            uint32_t count = bind_off->all ? live_stitches : bind_off->count.value_or(live_stitches);
            row.stitches.push_back(instruction::BindOff{count});
            pattern.rows.push_back(row);
            live_stitches -= count;
            log->debug("Emitter: bind-off {} stitches, live_stitches={}", count, live_stitches);
        }
        else if (auto* row_node = std::get_if<ast::RowNode>(&node)) {
            auto row = emit_row(*row_node, live_stitches);
            pattern.rows.push_back(row);

            // Calculate new stitch count
            uint32_t consumed = 0;
            uint32_t produced = 0;
            for (const auto& instr : row.stitches) {
                consumed += stitches_consumed(instr);
                produced += stitches_produced(instr);
            }
            uint32_t old_stitches = live_stitches;
            live_stitches = live_stitches - consumed + produced;
            log->debug("Emitter: row {} - consumed={}, produced={}, live_stitches: {} -> {}",
                       row_node->row_number.value_or(0), consumed, produced, old_stitches, live_stitches);
        }
    }

    log->debug("Emitter: finished with {} rows, final live_stitches={}",
               pattern.rows.size(), live_stitches);
    return pattern;
}

RowInstruction Emitter::emit_row(const ast::RowNode& row, uint32_t live_stitch_count) {
    RowInstruction result;
    result.side = row.side.value_or(RowSide::RS);
    result.stitches = emit_elements(row.elements, live_stitch_count, true);
    return result;
}

std::vector<StitchInstruction> Emitter::emit_elements(
    const std::vector<ast::RowElement>& elements,
    uint32_t available_stitches,
    [[maybe_unused]] bool fill_to_end
) {
    std::vector<StitchInstruction> result;
    uint32_t remaining = available_stitches;

    for (const auto& element : elements) {
        if (auto* stitch = std::get_if<ast::StitchNode>(&element)) {
            auto instructions = emit_stitch_node(*stitch);
            for (const auto& instr : instructions) {
                remaining -= stitches_consumed(instr);
                result.push_back(instr);
            }
        }
        else if (auto* repeat = std::get_if<ast::RepeatNode>(&element)) {
            auto instructions = emit_repeat(*repeat, remaining);
            for (const auto& instr : instructions) {
                remaining -= stitches_consumed(instr);
                result.push_back(instr);
            }
        }
    }

    return result;
}

std::vector<StitchInstruction> Emitter::emit_stitch_node(const ast::StitchNode& node) {
    std::vector<StitchInstruction> result;

    for (uint32_t i = 0; i < node.count; ++i) {
        result.push_back(node.instruction);
    }

    return result;
}

std::vector<StitchInstruction> Emitter::emit_repeat(
    const ast::RepeatNode& repeat,
    uint32_t remaining_stitches
) {
    auto log = yarnpath::logging::get_logger();
    std::vector<StitchInstruction> result;

    // Calculate stitches consumed by one iteration of the repeat body
    uint32_t body_consumption = calculate_consumption(repeat.body);

    if (body_consumption == 0) {
        log->warn("Repeat body consumes 0 stitches");
        warnings_.push_back("Repeat body consumes 0 stitches");
        return result;
    }

    uint32_t times = 0;
    uint32_t reserve = 0;  // stitches to leave unconsumed for "to last N sts"

    if (repeat.times) {
        // Explicit repeat count: [K2, P2] 3 times
        times = *repeat.times;
    }
    else if (repeat.to_end) {
        // Repeat to end: *K2, P2*, rep from * to end
        times = remaining_stitches / body_consumption;
    }
    else if (repeat.to_last) {
        // Repeat to last N stitches
        reserve = *repeat.to_last;
        if (remaining_stitches > reserve) {
            times = (remaining_stitches - reserve) / body_consumption;
        }
    }
    else {
        // Default: repeat once
        times = 1;
    }

    log->debug("Emitter: repeat body_consumption={}, remaining={}, times={}, reserve={}",
               body_consumption, remaining_stitches, times, reserve);

    // Emit the repeat body 'times' times
    for (uint32_t i = 0; i < times; ++i) {
        auto body_instructions = emit_elements(repeat.body, body_consumption, false);
        for (const auto& instr : body_instructions) {
            result.push_back(instr);
        }
    }

    return result;
}

uint32_t Emitter::calculate_consumption(const std::vector<ast::RowElement>& elements) {
    uint32_t total = 0;
    for (const auto& element : elements) {
        total += calculate_element_consumption(element);
    }
    return total;
}

uint32_t Emitter::calculate_element_consumption(const ast::RowElement& element) {
    if (auto* stitch = std::get_if<ast::StitchNode>(&element)) {
        return stitch_consumption(stitch->instruction) * stitch->count;
    }
    else if (auto* repeat = std::get_if<ast::RepeatNode>(&element)) {
        uint32_t body = calculate_consumption(repeat->body);
        uint32_t times = repeat->times.value_or(1);
        return body * times;
    }
    return 0;
}

uint32_t Emitter::stitch_consumption(const StitchInstruction& instr) {
    return stitches_consumed(instr);
}

}  // namespace parser
}  // namespace yarnpath
