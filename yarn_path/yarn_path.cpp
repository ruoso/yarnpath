#include "yarn_path.hpp"
#include "logging.hpp"
#include <algorithm>
#include <sstream>
#include <stdexcept>

namespace yarnpath {

// === YarnPath implementation ===

YarnPath YarnPath::from_stitch_graph(const StitchGraph& graph) {
    YarnPathBuilder builder(graph);
    return builder.build();
}

bool YarnPath::is_loop(SegmentId id) const {
    if (id >= segments_.size()) {
        return false;
    }
    return segments_[id].forms_loop;
}

const std::vector<SegmentId>* YarnPath::get_through(SegmentId id) const {
    if (id >= segments_.size()) {
        return nullptr;
    }
    return &segments_[id].through;
}

std::string YarnPath::to_dot() const {
    std::ostringstream ss;

    ss << "digraph YarnPath {\n";
    ss << "  rankdir=LR;\n";  // Left to right
    ss << "  node [shape=box];\n";
    ss << "  \n";

    // Output all segment nodes
    ss << "  // Segment nodes\n";
    for (size_t i = 0; i < segments_.size(); ++i) {
        ss << "  s" << i;
        if (segments_[i].forms_loop) {
            ss << " [style=filled, fillcolor=lightblue]";
        }
        ss << ";\n";
    }
    ss << "  \n";

    // Output loop nodes for segments that form loops
    ss << "  // Loop nodes\n";
    for (size_t i = 0; i < segments_.size(); ++i) {
        if (segments_[i].forms_loop) {
            ss << "  loop" << i << " [shape=ellipse, style=filled, fillcolor=lightyellow];\n";
        }
    }
    ss << "  \n";

    // Output sequential segment connections
    ss << "  // Sequential segment connections\n";
    for (size_t i = 0; i + 1 < segments_.size(); ++i) {
        ss << "  s" << i << " -> s" << (i + 1) << ";\n";
    }
    ss << "  \n";

    // Output bidirectional connections between segments and their loops
    ss << "  // Segment-loop connections\n";
    for (size_t i = 0; i < segments_.size(); ++i) {
        if (segments_[i].forms_loop) {
            ss << "  s" << i << " -> loop" << i << ";\n";
            ss << "  loop" << i << " -> s" << i << ";\n";
        }
    }
    ss << "  \n";

    // Output connections showing which loops segments go through
    ss << "  // Through connections\n";
    for (size_t i = 0; i < segments_.size(); ++i) {
        for (SegmentId parent : segments_[i].through) {
            if (parent < segments_.size() && segments_[parent].forms_loop) {
                ss << "  s" << i << " -> loop" << parent << " [style=dashed, color=gray];\n";
                ss << "  loop" << parent << " -> s" << i << " [style=dashed, color=gray];\n";
            }
        }
    }

    ss << "}\n";

    return ss.str();
}

// === YarnPathBuilder implementation ===

YarnPathBuilder::YarnPathBuilder(const StitchGraph& graph)
    : graph_(graph)
{
    // Reserve space for stitch_to_loops_
    stitch_to_loops_.resize(graph.size());
}

YarnPath YarnPathBuilder::build() {
    auto log = yarnpath::logging::get_logger();
    log->debug("YarnPath: building from {} rows", graph_.row_count());

    // Process all rows in order
    for (uint32_t row = 0; row < graph_.row_count(); ++row) {
        auto row_nodes = graph_.row(row);
        log->debug("YarnPath: processing row {} with {} stitch nodes", row, row_nodes.size());
        for (const auto& node : row_nodes) {
            process_stitch(node);
        }
    }

    // Build the result
    YarnPath result;
    result.segments_ = std::move(segments_);

    log->debug("YarnPath: built {} segments", result.segments_.size());
    return result;
}

void YarnPathBuilder::process_stitch(const StitchNode& node) {
    auto log = yarnpath::logging::get_logger();
    
    // Collect parent loop segments that this stitch works through
    std::vector<SegmentId> through_loops;
    
    for (StitchId parent_stitch : node.worked_through) {
        if (parent_stitch < stitch_to_loops_.size() &&
            !stitch_to_loops_[parent_stitch].empty()) {
            SegmentId parent_loop = stitch_to_loops_[parent_stitch].back();
            
            // Check if this parent loop is still live
            auto it = std::find(live_loops_.begin(), live_loops_.end(), parent_loop);
            if (it != live_loops_.end()) {
                // Consume this live loop
                size_t pos = static_cast<size_t>(std::distance(live_loops_.begin(), it));
                through_loops.push_back(consume_live_loop(pos));
            } else {
                // Parent already consumed (e.g., KFB second loop)
                through_loops.push_back(parent_loop);
            }
        }
    }
    
    // Create segment for this stitch (forms a loop)
    bool is_terminal = std::holds_alternative<stitch::BindOff>(node.operation);
    SegmentId seg_id = add_segment(std::move(through_loops), true, node.id);
    
    // Add to live loops (unless it's a bind-off which terminates)
    if (!is_terminal) {
        add_live_loop(seg_id);
    }
    
    log->trace("YarnPath: processed stitch {} -> segment {}", node.id, seg_id);
}

// === Helper implementations ===

SegmentId YarnPathBuilder::add_segment(std::vector<SegmentId> through, bool forms_loop, StitchId stitch_id) {
    auto log = yarnpath::logging::get_logger();
    SegmentId id = static_cast<SegmentId>(segments_.size());
    segments_.push_back(YarnSegment{std::move(through), forms_loop});
    
    // Track stitch -> loop segment mapping (builder-only, for finding parents)
    if (forms_loop) {
        if (stitch_id >= stitch_to_loops_.size()) {
            stitch_to_loops_.resize(stitch_id + 1);
        }
        stitch_to_loops_[stitch_id].push_back(id);
    }
    
    log->trace("YarnPath: created segment {}, forms_loop={}", id, forms_loop);
    return id;
}

SegmentId YarnPathBuilder::consume_live_loop(size_t pos) {
    if (pos >= live_loops_.size()) {
        throw std::out_of_range("No live loop at position");
    }
    SegmentId id = live_loops_[pos];
    live_loops_.erase(live_loops_.begin() + static_cast<ptrdiff_t>(pos));
    return id;
}

void YarnPathBuilder::add_live_loop(SegmentId id) {
    live_loops_.push_back(id);
}

}  // namespace yarnpath
