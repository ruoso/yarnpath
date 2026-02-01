#include "yarn_path.hpp"
#include "logging.hpp"
#include <algorithm>
#include <sstream>
#include <stdexcept>

namespace yarnpath {

// === YarnPath implementation ===

YarnPath YarnPath::from_stitch_graph(const StitchGraph& graph,
                                     const YarnProperties& yarn,
                                     const Gauge& gauge) {
    YarnPathBuilder builder(graph, yarn, gauge);
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

YarnPathBuilder::YarnPathBuilder(const StitchGraph& graph,
                                 const YarnProperties& yarn,
                                 const Gauge& gauge)
    : graph_(graph), yarn_(yarn), gauge_(gauge)
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

    // Determine topological metadata based on stitch type
    YarnSegment::LoopOrientation orientation = YarnSegment::LoopOrientation::Neutral;
    YarnSegment::WrapDirection wrap_direction = YarnSegment::WrapDirection::None;
    YarnSegment::WorkType work_type = YarnSegment::WorkType::Worked;

    if (std::holds_alternative<stitch::Knit>(node.operation)) {
        orientation = YarnSegment::LoopOrientation::Front;
        work_type = YarnSegment::WorkType::Worked;
    } else if (std::holds_alternative<stitch::Purl>(node.operation)) {
        orientation = YarnSegment::LoopOrientation::Back;
        work_type = YarnSegment::WorkType::Worked;
    } else if (std::holds_alternative<stitch::Slip>(node.operation)) {
        orientation = YarnSegment::LoopOrientation::Neutral;
        work_type = YarnSegment::WorkType::Transferred;
    } else if (std::holds_alternative<stitch::K2tog>(node.operation)) {
        orientation = YarnSegment::LoopOrientation::Front;
        wrap_direction = YarnSegment::WrapDirection::Clockwise;
        work_type = YarnSegment::WorkType::Worked;
    } else if (std::holds_alternative<stitch::SSK>(node.operation)) {
        orientation = YarnSegment::LoopOrientation::Front;
        wrap_direction = YarnSegment::WrapDirection::CounterClockwise;
        work_type = YarnSegment::WorkType::Worked;
    } else if (std::holds_alternative<stitch::S2KP>(node.operation)) {
        orientation = YarnSegment::LoopOrientation::Front;
        wrap_direction = YarnSegment::WrapDirection::CounterClockwise;
        work_type = YarnSegment::WorkType::Worked;
    } else if (std::holds_alternative<stitch::M1L>(node.operation)) {
        orientation = YarnSegment::LoopOrientation::Neutral;
        wrap_direction = YarnSegment::WrapDirection::CounterClockwise;
        work_type = YarnSegment::WorkType::Created;
    } else if (std::holds_alternative<stitch::M1R>(node.operation)) {
        orientation = YarnSegment::LoopOrientation::Neutral;
        wrap_direction = YarnSegment::WrapDirection::Clockwise;
        work_type = YarnSegment::WorkType::Created;
    } else if (std::holds_alternative<stitch::YarnOver>(node.operation)) {
        orientation = YarnSegment::LoopOrientation::Neutral;
        work_type = YarnSegment::WorkType::Created;
    } else if (std::holds_alternative<stitch::CastOn>(node.operation)) {
        orientation = YarnSegment::LoopOrientation::Neutral;
        work_type = YarnSegment::WorkType::Created;
    } else if (std::holds_alternative<stitch::BindOff>(node.operation)) {
        orientation = YarnSegment::LoopOrientation::Neutral;
        work_type = YarnSegment::WorkType::Worked;
    } else if (std::holds_alternative<stitch::KFB>(node.operation)) {
        orientation = YarnSegment::LoopOrientation::Front;
        work_type = YarnSegment::WorkType::Worked;
    } else if (std::holds_alternative<stitch::CableLeft>(node.operation)) {
        orientation = YarnSegment::LoopOrientation::Front;
        work_type = YarnSegment::WorkType::Worked;
    } else if (std::holds_alternative<stitch::CableRight>(node.operation)) {
        orientation = YarnSegment::LoopOrientation::Front;
        work_type = YarnSegment::WorkType::Worked;
    }

    // Create segment for this stitch (forms a loop)
    bool is_terminal = std::holds_alternative<stitch::BindOff>(node.operation);
    SegmentId seg_id = add_segment(std::move(through_loops), true, node.id,
                                    orientation, wrap_direction, work_type);

    // Add to live loops (unless it's a bind-off which terminates)
    if (!is_terminal) {
        add_live_loop(seg_id);
    }

    log->trace("YarnPath: processed stitch {} -> segment {}", node.id, seg_id);
}

// === Helper implementations ===

namespace {

// Calculate yarn length based on segment metadata
float calculate_yarn_length(YarnSegment::LoopOrientation orientation,
                            YarnSegment::WrapDirection wrap_direction,
                            YarnSegment::WorkType work_type,
                            size_t num_parents,
                            float loop_height,
                            float stitch_width) {
    // Base knit loop formula
    float base = M_PI * loop_height + stitch_width;

    // Handle special cases first
    if (work_type == YarnSegment::WorkType::Transferred) {
        return stitch_width * 0.5f;  // Slip stitch - minimal connector
    }

    if (work_type == YarnSegment::WorkType::Created && num_parents == 0) {
        // YarnOver, CastOn, M1L/M1R - pure loop, no passthrough
        float loop_only = M_PI * loop_height;
        if (wrap_direction != YarnSegment::WrapDirection::None) {
            loop_only *= 1.1f;  // Twisted pickup (M1L/M1R)
        }
        return loop_only;
    }

    // Apply orientation factor
    if (orientation == YarnSegment::LoopOrientation::Back) {
        base *= 0.88f;  // Purl loops use 12% less yarn
    }

    // Apply wrap direction factor (decreases/increases)
    if (wrap_direction == YarnSegment::WrapDirection::Clockwise) {
        base *= 0.82f;  // K2tog - right-leaning, tight
    } else if (wrap_direction == YarnSegment::WrapDirection::CounterClockwise) {
        if (num_parents >= 3) {
            base *= 0.80f;  // S2KP triple decrease - very tight
        } else {
            base *= 0.86f;  // SSK - left-leaning, slightly looser
        }
    }

    return base;
}

}  // namespace

SegmentId YarnPathBuilder::add_segment(std::vector<SegmentId> through, bool forms_loop, StitchId stitch_id,
                                       YarnSegment::LoopOrientation orientation,
                                       YarnSegment::WrapDirection wrap_direction,
                                       YarnSegment::WorkType work_type) {
    auto log = yarnpath::logging::get_logger();
    SegmentId id = static_cast<SegmentId>(segments_.size());

    // Get geometry from gauge
    float loop_height = gauge_.loop_height(yarn_.compressed_radius);
    float stitch_width = gauge_.stitch_width(yarn_.compressed_radius);

    // Calculate yarn length for this segment
    float yarn_length = calculate_yarn_length(orientation, wrap_direction, work_type,
                                              through.size(), loop_height, stitch_width);

    // Create segment with all topology metadata
    YarnSegment segment;
    segment.through = std::move(through);
    segment.forms_loop = forms_loop;
    segment.orientation = orientation;
    segment.wrap_direction = wrap_direction;
    segment.work_type = work_type;
    segment.target_yarn_length = yarn_length;  // Store calculated yarn length
    segments_.push_back(std::move(segment));

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
