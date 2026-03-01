#include "position_resolver.hpp"
#include "logging.hpp"
#include <limits>

namespace yarnpath {

std::optional<Vec3> get_segment_base_position(
    const YarnPath& yarn_path,
    const SurfaceGraph& surface,
    const YarnProperties& yarn,
    const std::map<SegmentId, std::vector<SegmentId>>& parent_map,
    const std::map<SegmentId, std::vector<SegmentId>>& children_map,
    SegmentId segment_id) {
    if (surface.has_segment(segment_id)) {
        NodeId node_id = surface.node_for_segment(segment_id);
        Vec3 base_pos = surface.node(node_id).position;
        auto parents_it = parent_map.find(segment_id);
        auto children_it = children_map.find(segment_id);
        if (parents_it == parent_map.end() ||
            (parents_it != parent_map.end() && parents_it->second.empty())) {
            // no parents, go through children, find the lowest Y,
            // make the base that - yarn diameter
            float min_y = std::numeric_limits<float>::max();
            if (children_it != children_map.end()) {
                for (SegmentId child_id : children_it->second) {
                    auto child_pos_opt = get_segment_base_position(
                        yarn_path, surface, yarn,
                        parent_map, children_map, child_id);
                    if (child_pos_opt.has_value()) {
                        Vec3 child_pos = child_pos_opt.value();
                        min_y = std::min(min_y, child_pos.y - yarn.compressed_radius * 2);
                    }
                }
            }
            if (min_y > base_pos.y) {
                yarnpath::logging::get_logger()->debug(
                    "get_segment_base_position: segment {} has no parents, adjusting base position from {} to {}",
                    segment_id, base_pos.y, min_y - yarn.compressed_radius);
                base_pos.y = min_y - yarn.compressed_radius;
            }
        }
        return base_pos;
    }
    return std::nullopt;
}

}  // namespace yarnpath
