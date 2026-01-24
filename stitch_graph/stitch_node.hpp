#ifndef YARNPATH_STITCH_NODE_HPP
#define YARNPATH_STITCH_NODE_HPP

#include "stitch_operation.hpp"
#include "row_instruction.hpp"
#include <optional>
#include <span>
#include <vector>

namespace yarnpath {

struct StitchNode {
    StitchId id;
    StitchOperation operation;
    uint32_t row;
    uint32_t column;
    std::vector<StitchId> worked_through;  // Parent stitches
    std::optional<uint32_t> panel_id;
};

struct RowInfo {
    uint32_t row_number;
    RowSide side;
    uint32_t first_stitch_id;
    uint32_t stitch_count;
};

class StitchGraph {
    std::vector<StitchNode> nodes_;
    std::vector<RowInfo> rows_;

public:
    // Build from instructions
    static StitchGraph from_instructions(const PatternInstructions& pattern);

    // Access
    const StitchNode* get(StitchId id) const;
    std::span<const StitchNode> row(uint32_t row_num) const;
    std::vector<StitchId> children_of(StitchId id) const;

    size_t size() const { return nodes_.size(); }
    uint32_t row_count() const { return static_cast<uint32_t>(rows_.size()); }

    // Access all nodes
    const std::vector<StitchNode>& nodes() const { return nodes_; }
    const std::vector<RowInfo>& row_infos() const { return rows_; }
};

}  // namespace yarnpath

#endif // YARNPATH_STITCH_NODE_HPP
