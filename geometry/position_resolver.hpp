#ifndef YARNPATH_GEOMETRY_POSITION_RESOLVER_HPP
#define YARNPATH_GEOMETRY_POSITION_RESOLVER_HPP

#include <math/vec3.hpp>
#include "yarn_path.hpp"
#include <yarn/yarn_properties.hpp>
#include <surface/surface_graph.hpp>
#include <vector>
#include <map>
#include <optional>

namespace yarnpath {

// Recursively resolve the base position for a segment from the surface graph.
// Parentless segments are adjusted to sit below their children by one yarn diameter.
std::optional<Vec3> get_segment_base_position(
    const YarnPath& yarn_path,
    const SurfaceGraph& surface,
    const YarnProperties& yarn,
    const std::map<SegmentId, std::vector<SegmentId>>& parent_map,
    const std::map<SegmentId, std::vector<SegmentId>>& children_map,
    SegmentId segment_id);

}  // namespace yarnpath

#endif // YARNPATH_GEOMETRY_POSITION_RESOLVER_HPP
