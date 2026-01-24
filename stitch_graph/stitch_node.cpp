#include "stitch_node.hpp"
#include "stitch_instruction.hpp"
#include <algorithm>
#include <stdexcept>

namespace yarnpath {

namespace {

// Helper to process a single instruction and add nodes to the graph
void process_instruction(
    const StitchInstruction& instr,
    std::vector<StitchNode>& nodes,
    std::vector<StitchId>& live_stitches,
    uint32_t& column,
    uint32_t row,
    std::optional<uint32_t> panel_id
) {
    std::visit([&](auto&& arg) {
        using T = std::decay_t<decltype(arg)>;

        if constexpr (std::is_same_v<T, instruction::Knit>) {
            if (live_stitches.empty()) {
                throw std::runtime_error("No live stitches available for Knit");
            }
            StitchId parent = live_stitches.front();
            live_stitches.erase(live_stitches.begin());

            StitchId new_id = static_cast<StitchId>(nodes.size());
            nodes.push_back({new_id, stitch::Knit{}, row, column++, {parent}, panel_id});
            live_stitches.push_back(new_id);
        }
        else if constexpr (std::is_same_v<T, instruction::Purl>) {
            if (live_stitches.empty()) {
                throw std::runtime_error("No live stitches available for Purl");
            }
            StitchId parent = live_stitches.front();
            live_stitches.erase(live_stitches.begin());

            StitchId new_id = static_cast<StitchId>(nodes.size());
            nodes.push_back({new_id, stitch::Purl{}, row, column++, {parent}, panel_id});
            live_stitches.push_back(new_id);
        }
        else if constexpr (std::is_same_v<T, instruction::Slip>) {
            if (live_stitches.empty()) {
                throw std::runtime_error("No live stitches available for Slip");
            }
            StitchId parent = live_stitches.front();
            live_stitches.erase(live_stitches.begin());

            StitchId new_id = static_cast<StitchId>(nodes.size());
            nodes.push_back({new_id, stitch::Slip{}, row, column++, {parent}, panel_id});
            live_stitches.push_back(new_id);
        }
        else if constexpr (std::is_same_v<T, instruction::CastOn>) {
            for (uint32_t i = 0; i < arg.count; ++i) {
                StitchId new_id = static_cast<StitchId>(nodes.size());
                nodes.push_back({new_id, stitch::CastOn{}, row, column++, {}, panel_id});
                live_stitches.push_back(new_id);
            }
        }
        else if constexpr (std::is_same_v<T, instruction::BindOff>) {
            for (uint32_t i = 0; i < arg.count; ++i) {
                if (live_stitches.empty()) {
                    throw std::runtime_error("No live stitches available for BindOff");
                }
                StitchId parent = live_stitches.front();
                live_stitches.erase(live_stitches.begin());

                StitchId new_id = static_cast<StitchId>(nodes.size());
                nodes.push_back({new_id, stitch::BindOff{}, row, column++, {parent}, panel_id});
                // BindOff doesn't produce live stitches
            }
        }
        else if constexpr (std::is_same_v<T, instruction::YarnOver>) {
            StitchId new_id = static_cast<StitchId>(nodes.size());
            nodes.push_back({new_id, stitch::YarnOver{}, row, column++, {}, panel_id});
            live_stitches.push_back(new_id);
        }
        else if constexpr (std::is_same_v<T, instruction::KFB>) {
            if (live_stitches.empty()) {
                throw std::runtime_error("No live stitches available for KFB");
            }
            StitchId parent = live_stitches.front();
            live_stitches.erase(live_stitches.begin());

            // KFB produces 2 stitches from 1
            StitchId new_id1 = static_cast<StitchId>(nodes.size());
            nodes.push_back({new_id1, stitch::KFB{}, row, column++, {parent}, panel_id});
            live_stitches.push_back(new_id1);

            StitchId new_id2 = static_cast<StitchId>(nodes.size());
            nodes.push_back({new_id2, stitch::KFB{}, row, column++, {parent}, panel_id});
            live_stitches.push_back(new_id2);
        }
        else if constexpr (std::is_same_v<T, instruction::M1L>) {
            StitchId new_id = static_cast<StitchId>(nodes.size());
            nodes.push_back({new_id, stitch::M1L{}, row, column++, {}, panel_id});
            live_stitches.push_back(new_id);
        }
        else if constexpr (std::is_same_v<T, instruction::M1R>) {
            StitchId new_id = static_cast<StitchId>(nodes.size());
            nodes.push_back({new_id, stitch::M1R{}, row, column++, {}, panel_id});
            live_stitches.push_back(new_id);
        }
        else if constexpr (std::is_same_v<T, instruction::K2tog>) {
            if (live_stitches.size() < 2) {
                throw std::runtime_error("Not enough live stitches for K2tog");
            }
            StitchId parent1 = live_stitches[0];
            StitchId parent2 = live_stitches[1];
            live_stitches.erase(live_stitches.begin(), live_stitches.begin() + 2);

            StitchId new_id = static_cast<StitchId>(nodes.size());
            nodes.push_back({new_id, stitch::K2tog{}, row, column++, {parent1, parent2}, panel_id});
            live_stitches.push_back(new_id);
        }
        else if constexpr (std::is_same_v<T, instruction::SSK>) {
            if (live_stitches.size() < 2) {
                throw std::runtime_error("Not enough live stitches for SSK");
            }
            StitchId parent1 = live_stitches[0];
            StitchId parent2 = live_stitches[1];
            live_stitches.erase(live_stitches.begin(), live_stitches.begin() + 2);

            StitchId new_id = static_cast<StitchId>(nodes.size());
            nodes.push_back({new_id, stitch::SSK{}, row, column++, {parent1, parent2}, panel_id});
            live_stitches.push_back(new_id);
        }
        else if constexpr (std::is_same_v<T, instruction::S2KP>) {
            if (live_stitches.size() < 3) {
                throw std::runtime_error("Not enough live stitches for S2KP");
            }
            StitchId parent1 = live_stitches[0];
            StitchId parent2 = live_stitches[1];
            StitchId parent3 = live_stitches[2];
            live_stitches.erase(live_stitches.begin(), live_stitches.begin() + 3);

            StitchId new_id = static_cast<StitchId>(nodes.size());
            nodes.push_back({new_id, stitch::S2KP{}, row, column++, {parent1, parent2, parent3}, panel_id});
            live_stitches.push_back(new_id);
        }
        else if constexpr (std::is_same_v<T, instruction::CableLeft>) {
            uint32_t total = arg.hold + arg.cross;
            if (live_stitches.size() < total) {
                throw std::runtime_error("Not enough live stitches for CableLeft");
            }

            // Extract the stitches involved
            std::vector<StitchId> held(live_stitches.begin(), live_stitches.begin() + arg.hold);
            std::vector<StitchId> crossed(live_stitches.begin() + arg.hold, live_stitches.begin() + total);
            live_stitches.erase(live_stitches.begin(), live_stitches.begin() + total);

            // For CableLeft: crossed stitches are worked first, then held
            // The reordering means: output order is [crossed, held]
            // Work through crossed first
            for (size_t i = 0; i < crossed.size(); ++i) {
                StitchId new_id = static_cast<StitchId>(nodes.size());
                stitch::CableLeft op;
                op.held = held;
                op.crossed = crossed;
                nodes.push_back({new_id, op, row, column++, {crossed[i]}, panel_id});
                live_stitches.push_back(new_id);
            }
            // Then held
            for (size_t i = 0; i < held.size(); ++i) {
                StitchId new_id = static_cast<StitchId>(nodes.size());
                stitch::CableLeft op;
                op.held = held;
                op.crossed = crossed;
                nodes.push_back({new_id, op, row, column++, {held[i]}, panel_id});
                live_stitches.push_back(new_id);
            }
        }
        else if constexpr (std::is_same_v<T, instruction::CableRight>) {
            uint32_t total = arg.hold + arg.cross;
            if (live_stitches.size() < total) {
                throw std::runtime_error("Not enough live stitches for CableRight");
            }

            // Extract the stitches involved
            std::vector<StitchId> crossed(live_stitches.begin(), live_stitches.begin() + arg.cross);
            std::vector<StitchId> held(live_stitches.begin() + arg.cross, live_stitches.begin() + total);
            live_stitches.erase(live_stitches.begin(), live_stitches.begin() + total);

            // For CableRight: held stitches are worked first, then crossed
            // The reordering means: output order is [held, crossed]
            // Work through held first
            for (size_t i = 0; i < held.size(); ++i) {
                StitchId new_id = static_cast<StitchId>(nodes.size());
                stitch::CableRight op;
                op.held = held;
                op.crossed = crossed;
                nodes.push_back({new_id, op, row, column++, {held[i]}, panel_id});
                live_stitches.push_back(new_id);
            }
            // Then crossed
            for (size_t i = 0; i < crossed.size(); ++i) {
                StitchId new_id = static_cast<StitchId>(nodes.size());
                stitch::CableRight op;
                op.held = held;
                op.crossed = crossed;
                nodes.push_back({new_id, op, row, column++, {crossed[i]}, panel_id});
                live_stitches.push_back(new_id);
            }
        }
        else if constexpr (std::is_same_v<T, instruction::Repeat>) {
            // Repeat should be expanded before processing
            throw std::runtime_error("Repeat should be expanded before processing");
        }
    }, instr);
}

}  // namespace

StitchGraph StitchGraph::from_instructions(const PatternInstructions& pattern) {
    StitchGraph graph;
    std::vector<StitchId> live_stitches;

    for (uint32_t row_idx = 0; row_idx < pattern.rows.size(); ++row_idx) {
        const auto& row_instr = pattern.rows[row_idx];

        // Expand any repeats
        auto expanded = expand_instructions(row_instr.stitches);

        // For WS rows, we work from right to left (reverse the live stitches order)
        if (row_instr.side == RowSide::WS && !live_stitches.empty()) {
            std::reverse(live_stitches.begin(), live_stitches.end());
        }

        // Record row start
        RowInfo row_info;
        row_info.row_number = row_idx;
        row_info.side = row_instr.side;
        row_info.first_stitch_id = static_cast<uint32_t>(graph.nodes_.size());

        uint32_t column = 0;
        std::vector<StitchId> new_live_stitches;

        // Process each instruction
        for (const auto& instr : expanded) {
            process_instruction(instr, graph.nodes_, live_stitches, column, row_idx, pattern.panel_id);
        }

        // After processing, live_stitches contains the new row's stitches
        // For WS rows, the output stitches are in reverse order, reverse back
        if (row_instr.side == RowSide::WS && !live_stitches.empty()) {
            std::reverse(live_stitches.begin(), live_stitches.end());
        }

        row_info.stitch_count = static_cast<uint32_t>(graph.nodes_.size()) - row_info.first_stitch_id;
        graph.rows_.push_back(row_info);
    }

    return graph;
}

const StitchNode* StitchGraph::get(StitchId id) const {
    if (id < nodes_.size()) {
        return &nodes_[id];
    }
    return nullptr;
}

std::span<const StitchNode> StitchGraph::row(uint32_t row_num) const {
    if (row_num >= rows_.size()) {
        return {};
    }
    const auto& info = rows_[row_num];
    return std::span<const StitchNode>(nodes_.data() + info.first_stitch_id, info.stitch_count);
}

std::vector<StitchId> StitchGraph::children_of(StitchId id) const {
    std::vector<StitchId> children;
    for (const auto& node : nodes_) {
        for (StitchId parent : node.worked_through) {
            if (parent == id) {
                children.push_back(node.id);
                break;
            }
        }
    }
    return children;
}

}  // namespace yarnpath
