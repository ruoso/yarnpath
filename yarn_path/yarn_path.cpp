#include "yarn_path.hpp"
#include <algorithm>
#include <stdexcept>

namespace yarnpath {

// === YarnPath implementation ===

YarnPath YarnPath::from_stitch_graph(const StitchGraph& graph) {
    YarnPathBuilder builder(graph);
    return builder.build();
}

const Loop* YarnPath::get_loop(LoopId id) const {
    if (id >= loops_.size()) {
        return nullptr;
    }
    return &loops_[id];
}

const std::vector<LoopId>& YarnPath::stitch_to_loops(StitchId stitch_id) const {
    static const std::vector<LoopId> empty;
    if (stitch_id >= stitch_to_loops_.size()) {
        return empty;
    }
    return stitch_to_loops_[stitch_id];
}

StitchId YarnPath::loop_to_stitch(LoopId loop_id) const {
    if (loop_id >= loops_.size()) {
        throw std::out_of_range("Invalid loop ID");
    }
    return loops_[loop_id].stitch_id;
}

std::vector<AnchorId> YarnPath::anchors_for_stitch(StitchId stitch_id) const {
    std::vector<AnchorId> result;
    for (const auto& anchor : anchors_) {
        if (anchor.stitch_id == stitch_id) {
            result.push_back(anchor.id);
        }
    }
    return result;
}

const AnchorNode* YarnPath::get_anchor(AnchorId id) const {
    if (id >= anchors_.size()) {
        return nullptr;
    }
    return &anchors_[id];
}

const YarnSegment* YarnPath::get_segment(SegmentId id) const {
    if (id >= segments_.size()) {
        return nullptr;
    }
    return &segments_[id];
}

// === YarnPathBuilder implementation ===

YarnPathBuilder::YarnPathBuilder(const StitchGraph& graph)
    : graph_(graph)
{
    // Reserve space for stitch_to_loops_
    stitch_to_loops_.resize(graph.size());
}

YarnPath YarnPathBuilder::build() {
    // Process all rows in order
    for (uint32_t row = 0; row < graph_.row_count(); ++row) {
        auto row_nodes = graph_.row(row);
        for (const auto& node : row_nodes) {
            process_stitch(node);
        }
    }

    // Build the result
    YarnPath result;
    result.anchors_ = std::move(anchors_);
    result.segments_ = std::move(segments_);
    result.loops_ = std::move(loops_);
    result.stitch_to_loops_ = std::move(stitch_to_loops_);

    if (!result.loops_.empty()) {
        // Find first and last loops in yarn order
        for (const auto& loop : result.loops_) {
            if (!loop.prev_in_yarn.has_value()) {
                result.first_loop_ = loop.id;
            }
            if (!loop.next_in_yarn.has_value()) {
                result.last_loop_ = loop.id;
            }
        }
    }

    return result;
}

void YarnPathBuilder::process_stitch(const StitchNode& node) {
    // Find position in live_loops_ by looking at worked_through
    // For most stitches, we find the first parent in live_loops_
    size_t live_pos = 0;

    if (!node.worked_through.empty()) {
        // Find the position of the first parent in live_loops_
        StitchId first_parent = node.worked_through[0];
        // Find which loop belongs to this parent stitch
        if (first_parent < stitch_to_loops_.size() &&
            !stitch_to_loops_[first_parent].empty()) {
            LoopId parent_loop = stitch_to_loops_[first_parent].back();
            auto it = std::find(live_loops_.begin(), live_loops_.end(), parent_loop);
            if (it != live_loops_.end()) {
                live_pos = static_cast<size_t>(std::distance(live_loops_.begin(), it));
            }
        }
    }

    std::visit([&](const auto& op) {
        using T = std::decay_t<decltype(op)>;

        if constexpr (std::is_same_v<T, stitch::CastOn>) {
            process_cast_on(node.id);
        } else if constexpr (std::is_same_v<T, stitch::Knit>) {
            process_knit(node.id, live_pos);
        } else if constexpr (std::is_same_v<T, stitch::Purl>) {
            process_purl(node.id, live_pos);
        } else if constexpr (std::is_same_v<T, stitch::Slip>) {
            process_slip(node.id, live_pos);
        } else if constexpr (std::is_same_v<T, stitch::YarnOver>) {
            process_yarn_over(node.id);
        } else if constexpr (std::is_same_v<T, stitch::KFB>) {
            process_kfb(node.id, live_pos);
        } else if constexpr (std::is_same_v<T, stitch::M1L>) {
            process_m1l(node.id);
        } else if constexpr (std::is_same_v<T, stitch::M1R>) {
            process_m1r(node.id);
        } else if constexpr (std::is_same_v<T, stitch::K2tog>) {
            process_k2tog(node.id, live_pos);
        } else if constexpr (std::is_same_v<T, stitch::SSK>) {
            process_ssk(node.id, live_pos);
        } else if constexpr (std::is_same_v<T, stitch::S2KP>) {
            process_s2kp(node.id, live_pos);
        } else if constexpr (std::is_same_v<T, stitch::BindOff>) {
            process_bind_off(node.id, live_pos);
        } else if constexpr (std::is_same_v<T, stitch::CableLeft>) {
            process_cable_left(node.id, op);
        } else if constexpr (std::is_same_v<T, stitch::CableRight>) {
            process_cable_right(node.id, op);
        }
    }, node.operation);
}

void YarnPathBuilder::process_cast_on(StitchId stitch_id) {
    // Cast-on creates a new loop with no parents
    LoopId loop_id = create_loop(stitch_id, FormKind::CastOn, {});

    Loop& loop = loops_[loop_id];

    // Create anchors for cast-on: CastOnBase -> LoopApex -> (LoopExit when worked)
    AnchorId base_anchor = add_anchor(anchor::CastOnBase{loop_id}, stitch_id);
    AnchorId apex_anchor = add_anchor(anchor::LoopApex{loop_id}, stitch_id);
    AnchorId form_anchor = add_anchor(
        anchor::LoopForm{loop_id, {}, FormKind::CastOn, false}, stitch_id);

    loop.apex_anchor = apex_anchor;
    loop.form_anchor = form_anchor;

    // Connect: last_anchor -> base (free) -> apex
    if (last_anchor_ != 0 || !anchors_.empty()) {
        add_segment(last_anchor_, base_anchor, segment::Free{});
    }
    add_segment(base_anchor, apex_anchor, segment::Free{});

    last_anchor_ = apex_anchor;
    add_live_loop(loop_id);
    link_loop(loop_id);
}

void YarnPathBuilder::process_knit(StitchId stitch_id, size_t live_pos) {
    // Consume parent loop
    LoopId parent_loop = consume_live_loop(live_pos);
    LoopId loop_id = create_loop(stitch_id, FormKind::Knit, {parent_loop});

    Loop& loop = loops_[loop_id];
    loops_[parent_loop].child_loops.push_back(loop_id);

    // Create anchors: LoopEntry (into parent) -> ThroughLoop -> LoopApex -> LoopForm
    AnchorId entry_anchor = add_anchor(anchor::LoopEntry{parent_loop}, stitch_id);
    AnchorId apex_anchor = add_anchor(anchor::LoopApex{loop_id}, stitch_id);
    AnchorId form_anchor = add_anchor(
        anchor::LoopForm{loop_id, {parent_loop}, FormKind::Knit, false}, stitch_id);
    AnchorId exit_anchor = add_anchor(anchor::LoopExit{parent_loop}, stitch_id);

    loop.entry_anchor = entry_anchor;
    loop.apex_anchor = apex_anchor;
    loop.form_anchor = form_anchor;
    loop.exit_anchor = exit_anchor;

    // Set parent's exit anchor
    loops_[parent_loop].exit_anchor = exit_anchor;

    // Connect segments
    add_segment(last_anchor_, entry_anchor, segment::Free{});
    add_segment(entry_anchor, apex_anchor,
                segment::ThroughLoop{parent_loop, PassMode::KnitWise});
    add_segment(apex_anchor, exit_anchor, segment::Free{});

    last_anchor_ = exit_anchor;
    add_live_loop(loop_id);
    link_loop(loop_id);
}

void YarnPathBuilder::process_purl(StitchId stitch_id, size_t live_pos) {
    // Consume parent loop
    LoopId parent_loop = consume_live_loop(live_pos);
    LoopId loop_id = create_loop(stitch_id, FormKind::Purl, {parent_loop});

    Loop& loop = loops_[loop_id];
    loops_[parent_loop].child_loops.push_back(loop_id);

    // Create anchors: LoopEntry -> ThroughLoop -> LoopApex -> LoopForm
    AnchorId entry_anchor = add_anchor(anchor::LoopEntry{parent_loop}, stitch_id);
    AnchorId apex_anchor = add_anchor(anchor::LoopApex{loop_id}, stitch_id);
    AnchorId form_anchor = add_anchor(
        anchor::LoopForm{loop_id, {parent_loop}, FormKind::Purl, false}, stitch_id);
    AnchorId exit_anchor = add_anchor(anchor::LoopExit{parent_loop}, stitch_id);

    loop.entry_anchor = entry_anchor;
    loop.apex_anchor = apex_anchor;
    loop.form_anchor = form_anchor;
    loop.exit_anchor = exit_anchor;

    loops_[parent_loop].exit_anchor = exit_anchor;

    // Connect segments (PurlWise pass mode)
    add_segment(last_anchor_, entry_anchor, segment::Free{});
    add_segment(entry_anchor, apex_anchor,
                segment::ThroughLoop{parent_loop, PassMode::PurlWise});
    add_segment(apex_anchor, exit_anchor, segment::Free{});

    last_anchor_ = exit_anchor;
    add_live_loop(loop_id);
    link_loop(loop_id);
}

void YarnPathBuilder::process_slip(StitchId stitch_id, size_t live_pos) {
    // Slip doesn't create a new loop, it just passes the old one along
    // But we still create a Loop record to track it
    LoopId parent_loop = consume_live_loop(live_pos);
    LoopId loop_id = create_loop(stitch_id, FormKind::Slip, {parent_loop});

    Loop& loop = loops_[loop_id];
    loops_[parent_loop].child_loops.push_back(loop_id);

    // For slip, we create minimal anchors - the loop isn't really worked
    AnchorId form_anchor = add_anchor(
        anchor::LoopForm{loop_id, {parent_loop}, FormKind::Slip, false}, stitch_id);
    AnchorId apex_anchor = add_anchor(anchor::LoopApex{loop_id}, stitch_id);

    loop.apex_anchor = apex_anchor;
    loop.form_anchor = form_anchor;

    // Free segment past the slipped stitch
    add_segment(last_anchor_, form_anchor, segment::Free{});

    last_anchor_ = form_anchor;
    add_live_loop(loop_id);
    link_loop(loop_id);
}

void YarnPathBuilder::process_yarn_over(StitchId stitch_id) {
    // Yarn over creates a loop from nothing
    LoopId loop_id = create_loop(stitch_id, FormKind::YarnOver, {});

    Loop& loop = loops_[loop_id];

    // YO has no entry through parent, just wraps around needle
    AnchorId yo_apex = add_anchor(anchor::YarnOverApex{loop_id}, stitch_id);
    AnchorId apex_anchor = add_anchor(anchor::LoopApex{loop_id}, stitch_id);
    AnchorId form_anchor = add_anchor(
        anchor::LoopForm{loop_id, {}, FormKind::YarnOver, false}, stitch_id);

    loop.apex_anchor = apex_anchor;
    loop.form_anchor = form_anchor;

    // Wrap segment for the YO
    add_segment(last_anchor_, yo_apex, segment::Free{});
    add_segment(yo_apex, apex_anchor, segment::Wrap{});

    last_anchor_ = apex_anchor;
    add_live_loop(loop_id);
    link_loop(loop_id);
}

void YarnPathBuilder::process_kfb(StitchId stitch_id, size_t live_pos) {
    // Note: In StitchGraph, KFB is expanded into 2 separate stitch nodes
    // (each with the same parent). So this function is called once per stitch node.
    // We treat each call as creating 1 loop, similar to a knit.

    // Check if this is the first or second KFB stitch by looking at which
    // stitch nodes share the same parent.
    const StitchNode* node = graph_.get(stitch_id);
    if (!node || node->worked_through.empty()) {
        return;
    }

    // Find the parent loop for this stitch
    StitchId parent_stitch = node->worked_through[0];
    LoopId parent_loop = 0;
    bool found_parent = false;

    // Check if parent loop is still live (first KFB) or already consumed (second KFB)
    if (parent_stitch < stitch_to_loops_.size() &&
        !stitch_to_loops_[parent_stitch].empty()) {
        LoopId candidate = stitch_to_loops_[parent_stitch].back();
        auto it = std::find(live_loops_.begin(), live_loops_.end(), candidate);
        if (it != live_loops_.end()) {
            // First KFB stitch - parent is still live
            live_pos = static_cast<size_t>(std::distance(live_loops_.begin(), it));
            parent_loop = consume_live_loop(live_pos);
            found_parent = true;
        } else {
            // Second KFB stitch - parent already consumed by first KFB
            parent_loop = candidate;
            found_parent = true;
        }
    }

    if (!found_parent) {
        // Fallback - shouldn't happen in well-formed input
        return;
    }

    // Create a single loop for this KFB stitch
    LoopId loop_id = create_loop(stitch_id, FormKind::KFB, {parent_loop});
    Loop& loop = loops_[loop_id];
    loops_[parent_loop].child_loops.push_back(loop_id);

    AnchorId entry = add_anchor(anchor::LoopEntry{parent_loop}, stitch_id);
    AnchorId apex = add_anchor(anchor::LoopApex{loop_id}, stitch_id);
    AnchorId form = add_anchor(
        anchor::LoopForm{loop_id, {parent_loop}, FormKind::KFB, false}, stitch_id);
    AnchorId exit = add_anchor(anchor::LoopExit{parent_loop}, stitch_id);

    loop.entry_anchor = entry;
    loop.apex_anchor = apex;
    loop.form_anchor = form;
    loop.exit_anchor = exit;

    loops_[parent_loop].exit_anchor = exit;

    add_segment(last_anchor_, entry, segment::Free{});
    add_segment(entry, apex, segment::ThroughLoop{parent_loop, PassMode::KnitWise});
    add_segment(apex, exit, segment::Free{});

    last_anchor_ = exit;
    add_live_loop(loop_id);
    link_loop(loop_id);
}

void YarnPathBuilder::process_m1l(StitchId stitch_id) {
    // M1L picks up the bar between stitches (left-leaning)
    LoopId loop_id = create_loop(stitch_id, FormKind::M1L, {});

    Loop& loop = loops_[loop_id];
    loop.is_lifted_bar = true;

    AnchorId apex_anchor = add_anchor(anchor::LoopApex{loop_id}, stitch_id);
    AnchorId form_anchor = add_anchor(
        anchor::LoopForm{loop_id, {}, FormKind::M1L, false}, stitch_id);

    loop.apex_anchor = apex_anchor;
    loop.form_anchor = form_anchor;

    add_segment(last_anchor_, apex_anchor, segment::Free{});

    last_anchor_ = apex_anchor;
    add_live_loop(loop_id);
    link_loop(loop_id);
}

void YarnPathBuilder::process_m1r(StitchId stitch_id) {
    // M1R picks up the bar between stitches (right-leaning)
    LoopId loop_id = create_loop(stitch_id, FormKind::M1R, {});

    Loop& loop = loops_[loop_id];
    loop.is_lifted_bar = true;

    AnchorId apex_anchor = add_anchor(anchor::LoopApex{loop_id}, stitch_id);
    AnchorId form_anchor = add_anchor(
        anchor::LoopForm{loop_id, {}, FormKind::M1R, false}, stitch_id);

    loop.apex_anchor = apex_anchor;
    loop.form_anchor = form_anchor;

    add_segment(last_anchor_, apex_anchor, segment::Free{});

    last_anchor_ = apex_anchor;
    add_live_loop(loop_id);
    link_loop(loop_id);
}

void YarnPathBuilder::process_k2tog(StitchId stitch_id, size_t live_pos) {
    // K2tog consumes 2 adjacent loops, creates 1
    // Right-leaning decrease: work through right loop first
    LoopId parent1 = consume_live_loop(live_pos);      // First (right on RS)
    LoopId parent2 = consume_live_loop(live_pos);      // Second (left on RS, now at same position)

    LoopId loop_id = create_loop(stitch_id, FormKind::K2tog, {parent1, parent2});
    Loop& loop = loops_[loop_id];

    loops_[parent1].child_loops.push_back(loop_id);
    loops_[parent2].child_loops.push_back(loop_id);

    // Enter through both parents
    AnchorId entry1 = add_anchor(anchor::LoopEntry{parent1}, stitch_id);
    AnchorId entry2 = add_anchor(anchor::LoopEntry{parent2}, stitch_id);
    AnchorId apex = add_anchor(anchor::LoopApex{loop_id}, stitch_id);
    AnchorId form = add_anchor(
        anchor::LoopForm{loop_id, {parent1, parent2}, FormKind::K2tog, false}, stitch_id);
    AnchorId exit1 = add_anchor(anchor::LoopExit{parent1}, stitch_id);
    AnchorId exit2 = add_anchor(anchor::LoopExit{parent2}, stitch_id);

    loop.entry_anchor = entry1;
    loop.apex_anchor = apex;
    loop.form_anchor = form;
    loop.exit_anchor = exit2;

    loops_[parent1].exit_anchor = exit1;
    loops_[parent2].exit_anchor = exit2;

    // Segments: free -> entry1 -> through parent1 -> entry2 -> through parent2 -> apex -> exits
    add_segment(last_anchor_, entry1, segment::Free{});
    add_segment(entry1, entry2, segment::ThroughLoop{parent1, PassMode::KnitWise});
    add_segment(entry2, apex, segment::ThroughLoop{parent2, PassMode::KnitWise});
    add_segment(apex, exit1, segment::Free{});
    add_segment(exit1, exit2, segment::Free{});

    last_anchor_ = exit2;
    add_live_loop(loop_id);
    link_loop(loop_id);
}

void YarnPathBuilder::process_ssk(StitchId stitch_id, size_t live_pos) {
    // SSK consumes 2 adjacent loops, creates 1
    // Left-leaning decrease: slip 2, knit together through back
    LoopId parent1 = consume_live_loop(live_pos);
    LoopId parent2 = consume_live_loop(live_pos);

    LoopId loop_id = create_loop(stitch_id, FormKind::SSK, {parent1, parent2});
    Loop& loop = loops_[loop_id];

    loops_[parent1].child_loops.push_back(loop_id);
    loops_[parent2].child_loops.push_back(loop_id);

    AnchorId entry1 = add_anchor(anchor::LoopEntry{parent1}, stitch_id);
    AnchorId entry2 = add_anchor(anchor::LoopEntry{parent2}, stitch_id);
    AnchorId apex = add_anchor(anchor::LoopApex{loop_id}, stitch_id);
    AnchorId form = add_anchor(
        anchor::LoopForm{loop_id, {parent1, parent2}, FormKind::SSK, false}, stitch_id);
    AnchorId exit1 = add_anchor(anchor::LoopExit{parent1}, stitch_id);
    AnchorId exit2 = add_anchor(anchor::LoopExit{parent2}, stitch_id);

    loop.entry_anchor = entry1;
    loop.apex_anchor = apex;
    loop.form_anchor = form;
    loop.exit_anchor = exit2;

    loops_[parent1].exit_anchor = exit1;
    loops_[parent2].exit_anchor = exit2;

    add_segment(last_anchor_, entry1, segment::Free{});
    add_segment(entry1, entry2, segment::ThroughLoop{parent1, PassMode::KnitWise});
    add_segment(entry2, apex, segment::ThroughLoop{parent2, PassMode::KnitWise});
    add_segment(apex, exit1, segment::Free{});
    add_segment(exit1, exit2, segment::Free{});

    last_anchor_ = exit2;
    add_live_loop(loop_id);
    link_loop(loop_id);
}

void YarnPathBuilder::process_s2kp(StitchId stitch_id, size_t live_pos) {
    // S2KP (sl2-k1-p2sso): centered decrease consuming 3 loops
    LoopId parent1 = consume_live_loop(live_pos);
    LoopId parent2 = consume_live_loop(live_pos);
    LoopId parent3 = consume_live_loop(live_pos);

    LoopId loop_id = create_loop(stitch_id, FormKind::S2KP, {parent1, parent2, parent3});
    Loop& loop = loops_[loop_id];

    loops_[parent1].child_loops.push_back(loop_id);
    loops_[parent2].child_loops.push_back(loop_id);
    loops_[parent3].child_loops.push_back(loop_id);

    AnchorId entry1 = add_anchor(anchor::LoopEntry{parent1}, stitch_id);
    AnchorId entry2 = add_anchor(anchor::LoopEntry{parent2}, stitch_id);
    AnchorId entry3 = add_anchor(anchor::LoopEntry{parent3}, stitch_id);
    AnchorId apex = add_anchor(anchor::LoopApex{loop_id}, stitch_id);
    AnchorId form = add_anchor(
        anchor::LoopForm{loop_id, {parent1, parent2, parent3}, FormKind::S2KP, false}, stitch_id);
    AnchorId exit1 = add_anchor(anchor::LoopExit{parent1}, stitch_id);
    AnchorId exit2 = add_anchor(anchor::LoopExit{parent2}, stitch_id);
    AnchorId exit3 = add_anchor(anchor::LoopExit{parent3}, stitch_id);

    loop.entry_anchor = entry1;
    loop.apex_anchor = apex;
    loop.form_anchor = form;
    loop.exit_anchor = exit3;

    loops_[parent1].exit_anchor = exit1;
    loops_[parent2].exit_anchor = exit2;
    loops_[parent3].exit_anchor = exit3;

    add_segment(last_anchor_, entry1, segment::Free{});
    add_segment(entry1, entry2, segment::ThroughLoop{parent1, PassMode::KnitWise});
    add_segment(entry2, entry3, segment::ThroughLoop{parent2, PassMode::KnitWise});
    add_segment(entry3, apex, segment::ThroughLoop{parent3, PassMode::KnitWise});
    add_segment(apex, exit1, segment::Free{});
    add_segment(exit1, exit2, segment::Free{});
    add_segment(exit2, exit3, segment::Free{});

    last_anchor_ = exit3;
    add_live_loop(loop_id);
    link_loop(loop_id);
}

void YarnPathBuilder::process_bind_off(StitchId stitch_id, size_t live_pos) {
    // Bind off consumes a loop and terminates it
    LoopId parent_loop = consume_live_loop(live_pos);
    LoopId loop_id = create_loop(stitch_id, FormKind::BindOff, {parent_loop});

    Loop& loop = loops_[loop_id];
    loop.is_bound_off = true;
    loops_[parent_loop].child_loops.push_back(loop_id);

    AnchorId entry_anchor = add_anchor(anchor::LoopEntry{parent_loop}, stitch_id);
    AnchorId apex_anchor = add_anchor(anchor::LoopApex{loop_id}, stitch_id);
    AnchorId form_anchor = add_anchor(
        anchor::LoopForm{loop_id, {parent_loop}, FormKind::BindOff, false}, stitch_id);
    AnchorId exit_anchor = add_anchor(anchor::LoopExit{parent_loop}, stitch_id);
    AnchorId bind_off_end = add_anchor(anchor::BindOffEnd{loop_id}, stitch_id);

    loop.entry_anchor = entry_anchor;
    loop.apex_anchor = apex_anchor;
    loop.form_anchor = form_anchor;
    loop.exit_anchor = exit_anchor;

    loops_[parent_loop].exit_anchor = exit_anchor;

    add_segment(last_anchor_, entry_anchor, segment::Free{});
    add_segment(entry_anchor, apex_anchor,
                segment::ThroughLoop{parent_loop, PassMode::KnitWise});
    add_segment(apex_anchor, exit_anchor, segment::Free{});
    add_segment(exit_anchor, bind_off_end, segment::Free{});

    last_anchor_ = bind_off_end;
    // Bind-off doesn't add to live loops - it's terminal
    link_loop(loop_id);
}

void YarnPathBuilder::process_cable_left(StitchId stitch_id, const stitch::CableLeft& cable) {
    // Cable left: held stitches go to front, crossed stitches worked first
    // This is a single stitch node that represents one stitch in the cable
    // The cable info tells us which stitches are involved

    // For now, treat each cable stitch like a regular knit
    // Full cable implementation would need CrossOver anchor

    // Find parent loop
    const StitchNode* node = graph_.get(stitch_id);
    if (!node || node->worked_through.empty()) {
        return;
    }

    // Find position of first parent in live loops
    StitchId first_parent = node->worked_through[0];
    size_t live_pos = 0;
    if (first_parent < stitch_to_loops_.size() &&
        !stitch_to_loops_[first_parent].empty()) {
        LoopId parent_loop = stitch_to_loops_[first_parent].back();
        auto it = std::find(live_loops_.begin(), live_loops_.end(), parent_loop);
        if (it != live_loops_.end()) {
            live_pos = static_cast<size_t>(std::distance(live_loops_.begin(), it));
        }
    }

    // For simplicity, process as knit (full cable tracking would be more complex)
    process_knit(stitch_id, live_pos);

    // Mark unused to avoid warnings
    (void)cable;
}

void YarnPathBuilder::process_cable_right(StitchId stitch_id, const stitch::CableRight& cable) {
    // Cable right: held stitches go to back, crossed stitches worked first
    // Similar to cable_left

    const StitchNode* node = graph_.get(stitch_id);
    if (!node || node->worked_through.empty()) {
        return;
    }

    StitchId first_parent = node->worked_through[0];
    size_t live_pos = 0;
    if (first_parent < stitch_to_loops_.size() &&
        !stitch_to_loops_[first_parent].empty()) {
        LoopId parent_loop = stitch_to_loops_[first_parent].back();
        auto it = std::find(live_loops_.begin(), live_loops_.end(), parent_loop);
        if (it != live_loops_.end()) {
            live_pos = static_cast<size_t>(std::distance(live_loops_.begin(), it));
        }
    }

    process_knit(stitch_id, live_pos);

    (void)cable;
}

// === Helper implementations ===

AnchorId YarnPathBuilder::add_anchor(const Anchor& anchor, StitchId stitch_id) {
    AnchorId id = static_cast<AnchorId>(anchors_.size());
    anchors_.push_back(AnchorNode{id, anchor, stitch_id, {}, {}, {}});
    return id;
}

SegmentId YarnPathBuilder::add_segment(AnchorId from, AnchorId to, const SegmentType& type) {
    SegmentId id = static_cast<SegmentId>(segments_.size());
    segments_.push_back(YarnSegment{id, from, to, type});
    return id;
}

LoopId YarnPathBuilder::create_loop(StitchId stitch_id, FormKind kind,
                                     const std::vector<LoopId>& parents) {
    LoopId id = static_cast<LoopId>(loops_.size());

    Loop loop;
    loop.id = id;
    loop.stitch_id = stitch_id;
    loop.kind = kind;
    loop.parent_loops = parents;
    loop.apex_anchor = 0;  // Will be set by caller
    loop.form_anchor = 0;  // Will be set by caller

    loops_.push_back(loop);

    // Track stitch -> loop mapping
    if (stitch_id >= stitch_to_loops_.size()) {
        stitch_to_loops_.resize(stitch_id + 1);
    }
    stitch_to_loops_[stitch_id].push_back(id);

    return id;
}

LoopId YarnPathBuilder::consume_live_loop(size_t pos) {
    if (pos >= live_loops_.size()) {
        throw std::out_of_range("No live loop at position");
    }
    LoopId id = live_loops_[pos];
    live_loops_.erase(live_loops_.begin() + static_cast<ptrdiff_t>(pos));
    return id;
}

void YarnPathBuilder::add_live_loop(LoopId id) {
    // Add to the end - position tracking happens in process_stitch
    live_loops_.push_back(id);
}

void YarnPathBuilder::link_loop(LoopId id) {
    if (last_loop_.has_value()) {
        loops_[last_loop_.value()].next_in_yarn = id;
        loops_[id].prev_in_yarn = last_loop_.value();
    }
    last_loop_ = id;
}

}  // namespace yarnpath
