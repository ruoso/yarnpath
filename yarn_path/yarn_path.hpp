#ifndef YARNPATH_YARN_PATH_HPP
#define YARNPATH_YARN_PATH_HPP

#include "stitch_node.hpp"
#include <cstdint>
#include <string>
#include <vector>

namespace yarnpath {

// Segment index is the identity
using SegmentId = uint32_t;

// A yarn segment - may pass through parent loops, may form a loop at its end
struct YarnSegment {
    std::vector<SegmentId> through;  // Parent loop segments this passes through
    bool forms_loop;                  // True if this segment ends by forming a loop
};

// Forward declaration
class YarnPathBuilder;

// Main YarnPath class - pure topology as a sequence of segments
class YarnPath {
public:
    friend class YarnPathBuilder;

    static YarnPath from_stitch_graph(const StitchGraph& graph);

    // Segment access (index is identity)
    const std::vector<YarnSegment>& segments() const { return segments_; }
    size_t segment_count() const { return segments_.size(); }
    
    // Check if a segment is a loop
    bool is_loop(SegmentId id) const;
    
    // Get loop segments that a loop passes through (empty if not a loop or no parents)
    const std::vector<SegmentId>* get_through(SegmentId id) const;

    // Visualization
    std::string to_dot() const;

private:
    std::vector<YarnSegment> segments_;
};

// Builder class for constructing YarnPath from StitchGraph
class YarnPathBuilder {
public:
    explicit YarnPathBuilder(const StitchGraph& graph);
    YarnPath build();

private:
    const StitchGraph& graph_;

    // Needle state machine: live loop segments in current needle order
    std::vector<SegmentId> live_loops_;

    // Output accumulator
    std::vector<YarnSegment> segments_;

    // Builder-only state: maps stitches to loop segments during construction
    std::vector<std::vector<SegmentId>> stitch_to_loops_;

    // Process stitch, consuming from live_loops_ by position
    void process_stitch(const StitchNode& node);

    // Helpers
    SegmentId add_segment(std::vector<SegmentId> through, bool forms_loop, StitchId stitch_id);

    // Consume loop from live_loops_ at position
    SegmentId consume_live_loop(size_t pos);
    void add_live_loop(SegmentId id);
};

}  // namespace yarnpath

#endif  // YARNPATH_YARN_PATH_HPP
