#ifndef YARNPATH_STITCH_NODE_HPP
#define YARNPATH_STITCH_NODE_HPP

#include "stitch_operation.hpp"
#include "row_instruction.hpp"
#include <optional>
#include <span>
#include <vector>

namespace yarnpath {

/// A single stitch node in the fabric topology graph.
struct StitchNode {
    StitchId id;
    StitchOperation operation;
    uint32_t row;
    uint32_t column;              ///< Fabric position (consistent across RS/WS rows).
    std::vector<StitchId> worked_through;  ///< Parent stitches this one was pulled through.
    std::optional<uint32_t> panel_id;
};

struct RowInfo {
    uint32_t row_number;
    RowSide side;
    uint32_t first_stitch_id;
    uint32_t stitch_count;
};

/// Fabric topology graph built from PatternInstructions.
///
/// Invariants:
/// - Nodes are ordered by id (which equals their index in the nodes vector).
/// - Rows are contiguous spans of nodes; row(N) returns a span starting at
///   first_stitch_id with stitch_count elements.
/// - Live stitches left unconsumed at the end is valid (short rows, partial bind-off).
/// - For WS rows, live stitches are reversed before processing (modeling right-to-left
///   working direction), and columns are remapped to fabric position afterward.
class StitchGraph {
    std::vector<StitchNode> nodes_;
    std::vector<RowInfo> rows_;

public:
    StitchGraph() = default;
    StitchGraph(std::vector<StitchNode> nodes, std::vector<RowInfo> rows)
        : nodes_(std::move(nodes)), rows_(std::move(rows)) {}

    static StitchGraph from_instructions(const PatternInstructions& pattern);

    const StitchNode* get(StitchId id) const;
    std::span<const StitchNode> row(uint32_t row_num) const;
    std::vector<StitchId> children_of(StitchId id) const;

    size_t size() const { return nodes_.size(); }
    uint32_t row_count() const { return static_cast<uint32_t>(rows_.size()); }

    const std::vector<StitchNode>& nodes() const { return nodes_; }
    const std::vector<RowInfo>& row_infos() const { return rows_; }
};

}  // namespace yarnpath

#endif // YARNPATH_STITCH_NODE_HPP
