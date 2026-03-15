#include "position_resolver.hpp"
#include "logging.hpp"
#include <limits>

namespace yarnpath {

std::vector<SegmentFrame> resolve_segment_frames(
    const YarnPath& yarn_path,
    const SurfaceGraph& surface,
    const YarnProperties& yarn,
    const std::map<SegmentId, std::vector<SegmentId>>& parent_map,
    const std::map<SegmentId, std::vector<SegmentId>>& children_map) {

    auto log = yarnpath::logging::get_logger();
    const auto& segments = yarn_path.segments();
    std::vector<SegmentFrame> frames(segments.size());

    // First pass: read all data from surface nodes
    for (size_t i = 0; i < segments.size(); ++i) {
        SegmentId seg_id = static_cast<SegmentId>(i);
        SegmentFrame& frame = frames[i];

        if (surface.has_segment(seg_id)) {
            NodeId node_id = surface.node_for_segment(seg_id);
            const auto& node = surface.node(node_id);

            frame.position = node.position;
            frame.stitch_axis = node.stitch_axis;
            frame.fabric_normal = node.fabric_normal;
            frame.wale_axis = node.wale_axis;
            frame.shape = node.shape;
        } else {
            log->warn("resolve_segment_frames: segment {} not in surface graph, using defaults", seg_id);
            frame.position = Vec3::zero();
            frame.stitch_axis = Vec3::unit_x();
            frame.fabric_normal = Vec3::unit_z();
            frame.wale_axis = Vec3::unit_y();
            frame.shape = StitchShapeParams{};
        }
    }

    // Second pass: adjust parentless segments to sit below their children
    for (size_t i = 0; i < segments.size(); ++i) {
        SegmentId seg_id = static_cast<SegmentId>(i);
        auto parents_it = parent_map.find(seg_id);
        bool has_parents = (parents_it != parent_map.end() && !parents_it->second.empty());

        if (!has_parents) {
            auto children_it = children_map.find(seg_id);
            if (children_it != children_map.end() && !children_it->second.empty()) {
                float min_y = std::numeric_limits<float>::max();
                for (SegmentId child_id : children_it->second) {
                    if (child_id < frames.size()) {
                        float child_y = frames[child_id].position.y - yarn.compressed_radius * 2;
                        min_y = std::min(min_y, child_y);
                    }
                }
                if (min_y < frames[i].position.y) {
                    log->debug("resolve_segment_frames: segment {} has no parents, adjusting Y from {} to {}",
                               seg_id, frames[i].position.y, min_y - yarn.compressed_radius);
                    frames[i].position.y = min_y - yarn.compressed_radius;
                }
            }
        }
    }

    return frames;
}

}  // namespace yarnpath
