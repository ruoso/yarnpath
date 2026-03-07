#ifndef YARNPATH_GEOMETRY_POSITION_RESOLVER_HPP
#define YARNPATH_GEOMETRY_POSITION_RESOLVER_HPP

#include <math/vec3.hpp>
#include <stitch_shape/stitch_shape.hpp>
#include "yarn_path.hpp"
#include <yarn/yarn_properties.hpp>
#include <surface/surface_graph.hpp>
#include <vector>
#include <map>
#include <optional>

namespace yarnpath {

// Complete local coordinate frame for a segment, derived from the enriched SurfaceNode.
// Provides all the information the geometry builder needs to construct curves in the
// segment's local frame without re-deriving anything from raw positions.
struct SegmentFrame {
    Vec3 position;        // Relaxed world-space position (may be adjusted for parentless segments)
    Vec3 stitch_axis;     // Course direction (along the row)
    Vec3 fabric_normal;   // Perpendicular to fabric surface (front-facing)
    Vec3 wale_axis;       // Row-stacking direction = fabric_normal.cross(stitch_axis)
    StitchShapeParams shape;  // z_bulge, loop_height, stitch_width, multipliers, etc.
};

// Resolve a complete SegmentFrame for each segment in the yarn path.
// Reads position, stitch_axis, fabric_normal, and shape from the enriched SurfaceNode.
// Derives wale_axis as fabric_normal.cross(stitch_axis).
// Adjusts parentless segments to sit below their children by one yarn diameter.
std::vector<SegmentFrame> resolve_segment_frames(
    const YarnPath& yarn_path,
    const SurfaceGraph& surface,
    const YarnProperties& yarn,
    const std::map<SegmentId, std::vector<SegmentId>>& parent_map,
    const std::map<SegmentId, std::vector<SegmentId>>& children_map);

}  // namespace yarnpath

#endif // YARNPATH_GEOMETRY_POSITION_RESOLVER_HPP
