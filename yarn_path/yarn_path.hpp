#ifndef YARNPATH_YARN_PATH_HPP
#define YARNPATH_YARN_PATH_HPP

#include "stitch_node.hpp"
#include <cstdint>
#include <optional>
#include <string>
#include <variant>
#include <vector>

namespace yarnpath {

// Forward declarations and type aliases
using LoopId = uint32_t;
using AnchorId = uint32_t;
using SegmentId = uint32_t;

// Pass mode for segments passing through loops
enum class PassMode {
    KnitWise,        // front to back (standard knit)
    PurlWise,        // back to front (standard purl)
    ThroughBackLoop, // twisted knit (ktbl)
    ThroughFrontLoop // twisted purl (ptbl)
};

// Loop formation kind
enum class FormKind {
    CastOn,
    Knit,
    Purl,
    Slip,
    YarnOver,
    M1L,
    M1R,
    KFB,    // creates 2 loops
    K2tog,  // consumes 2 parents
    SSK,    // consumes 2 parents
    S2KP,   // consumes 3 parents
    BindOff
};

// Anchor types namespace
namespace anchor {

// === Formation anchors (semantic) ===

// Single source of truth for "what happened" at loop creation
struct LoopForm {
    LoopId loop_id;
    std::vector<LoopId> parents;  // Ordered parent loops consumed
    FormKind kind;
    bool twisted = false;         // Through back loop
};

// === Shape anchors (geometric) ===

struct LoopApex { LoopId loop_id; };
struct LoopEntry { LoopId loop_id; };
struct LoopExit { LoopId loop_id; };

// Cast-on edge (no parent loop to enter through)
struct CastOnBase { LoopId loop_id; };

// Bind-off terminal
struct BindOffEnd { LoopId loop_id; };

// Yarn-over apex (created without passing through anything)
struct YarnOverApex { LoopId loop_id; };

// Cable crossover - ordered for geometry
struct CrossOver {
    std::vector<LoopId> under_loops;        // Loops yarn goes under (ordered)
    std::vector<LoopId> over_loops;         // Loops yarn goes over (ordered)
    std::vector<SegmentId> held_segments;   // Segments being held
    std::vector<SegmentId> active_segments; // Segments worked first
};

}  // namespace anchor

using Anchor = std::variant<
    anchor::LoopForm,
    anchor::LoopApex, anchor::LoopEntry, anchor::LoopExit,
    anchor::CastOnBase, anchor::BindOffEnd, anchor::YarnOverApex,
    anchor::CrossOver
>;

// Anchor node with position hints
struct AnchorNode {
    AnchorId id;
    Anchor anchor;
    StitchId stitch_id;

    // Layout hints (optional, for geometry layer)
    // Normalized coordinates [0,1] within stitch cell
    std::optional<float> hint_x;  // Horizontal in stitch
    std::optional<float> hint_y;  // Vertical (0=bottom, 1=top)
    std::optional<float> hint_z;  // Depth (front/back)
};

// Segment types namespace
namespace segment {

// Free yarn segment (doesn't interact with any loop)
struct Free {};

// Yarn passes through an existing loop (geometric traversal)
struct ThroughLoop {
    LoopId loop_id;
    PassMode mode;
};

// Yarn wraps around needle (YO)
struct Wrap {};

// Yarn segment held aside during cable
struct HeldForCable {
    LoopId cable_loop;  // Which cable operation this belongs to
};

}  // namespace segment

using SegmentType = std::variant<
    segment::Free,
    segment::ThroughLoop,
    segment::Wrap,
    segment::HeldForCable
>;

// Yarn segment connecting two anchors
struct YarnSegment {
    SegmentId id;
    AnchorId from_anchor;
    AnchorId to_anchor;
    SegmentType segment_type;
};

// Loop representing where yarn forms a loop at each stitch
struct Loop {
    LoopId id;
    StitchId stitch_id;

    // Formation (semantic)
    FormKind kind;
    std::vector<LoopId> parent_loops;  // Ordered: consumed in this order
    std::vector<LoopId> child_loops;

    // Shape (geometric anchor references)
    std::optional<AnchorId> entry_anchor;
    AnchorId apex_anchor;
    std::optional<AnchorId> exit_anchor;
    AnchorId form_anchor;              // LoopForm anchor

    // Yarn traversal links (for easy debugging/iteration)
    std::optional<LoopId> prev_in_yarn;
    std::optional<LoopId> next_in_yarn;

    // Flags
    bool is_lifted_bar = false;  // M1L/M1R
    bool is_bound_off = false;
    bool twisted = false;
};

// Forward declaration
class YarnPathBuilder;

// Main YarnPath class
class YarnPath {
public:
    friend class YarnPathBuilder;

    static YarnPath from_stitch_graph(const StitchGraph& graph);

    // Yarn-order traversal (1D linear)
    const std::vector<YarnSegment>& segments() const { return segments_; }
    const std::vector<AnchorNode>& anchors() const { return anchors_; }
    size_t segment_count() const { return segments_.size(); }

    // Loop-order traversal (topological)
    const std::vector<Loop>& loops() const { return loops_; }
    const Loop* get_loop(LoopId id) const;

    // Cross-reference lookups
    const std::vector<LoopId>& stitch_to_loops(StitchId stitch_id) const;
    StitchId loop_to_stitch(LoopId loop_id) const;
    std::vector<AnchorId> anchors_for_stitch(StitchId stitch_id) const;

    // Yarn traversal (via Loop links)
    LoopId first_loop() const { return first_loop_; }
    LoopId last_loop() const { return last_loop_; }

    // Anchor lookup
    const AnchorNode* get_anchor(AnchorId id) const;

    // Segment lookup
    const YarnSegment* get_segment(SegmentId id) const;

    // Visualization
    std::string to_dot() const;

private:
    std::vector<AnchorNode> anchors_;
    std::vector<YarnSegment> segments_;
    std::vector<Loop> loops_;
    std::vector<std::vector<LoopId>> stitch_to_loops_;
    LoopId first_loop_ = 0;
    LoopId last_loop_ = 0;
};

// Builder class for constructing YarnPath from StitchGraph
class YarnPathBuilder {
public:
    explicit YarnPathBuilder(const StitchGraph& graph);
    YarnPath build();

private:
    const StitchGraph& graph_;

    // Needle state machine: live loops in current needle order
    std::vector<LoopId> live_loops_;

    // Track yarn position
    AnchorId last_anchor_ = 0;  // For free yarn connections only
    std::optional<LoopId> last_loop_;  // For prev/next links

    // Output accumulators
    std::vector<AnchorNode> anchors_;
    std::vector<YarnSegment> segments_;
    std::vector<Loop> loops_;
    std::vector<std::vector<LoopId>> stitch_to_loops_;

    // Process stitch, consuming from live_loops_ by position
    void process_stitch(const StitchNode& node);

    // Per-stitch-type handlers
    void process_cast_on(StitchId stitch_id);
    void process_knit(StitchId stitch_id, size_t live_pos);
    void process_purl(StitchId stitch_id, size_t live_pos);
    void process_slip(StitchId stitch_id, size_t live_pos);
    void process_yarn_over(StitchId stitch_id);
    void process_kfb(StitchId stitch_id, size_t live_pos);
    void process_m1l(StitchId stitch_id);
    void process_m1r(StitchId stitch_id);
    void process_k2tog(StitchId stitch_id, size_t live_pos);
    void process_ssk(StitchId stitch_id, size_t live_pos);
    void process_s2kp(StitchId stitch_id, size_t live_pos);
    void process_bind_off(StitchId stitch_id, size_t live_pos);
    void process_cable_left(StitchId stitch_id, const stitch::CableLeft& cable);
    void process_cable_right(StitchId stitch_id, const stitch::CableRight& cable);

    // Helpers
    AnchorId add_anchor(const Anchor& anchor, StitchId stitch_id);
    SegmentId add_segment(AnchorId from, AnchorId to, const SegmentType& type);
    LoopId create_loop(StitchId stitch_id, FormKind kind,
                       const std::vector<LoopId>& parents);

    // Consume loop from live_loops_ at position, returns LoopId
    LoopId consume_live_loop(size_t pos);
    void add_live_loop(LoopId id);

    // Update prev/next chain
    void link_loop(LoopId id);
};

}  // namespace yarnpath

#endif // YARNPATH_YARN_PATH_HPP
