#ifndef YARNPATH_GEOMETRY_BUILDER_HPP
#define YARNPATH_GEOMETRY_BUILDER_HPP

#include "geometry_path.hpp"
#include "yarn_path.hpp"
#include <yarn/yarn_properties.hpp>
#include <yarn/gauge.hpp>
#include <surface/surface_graph.hpp>
#include <math/vec3.hpp>
#include "cubic_bezier.hpp"
#include <vector>
#include <map>

namespace yarnpath {

// Build complete geometry from yarn path using relaxed surface positions
// The surface graph provides the actual positions from physics simulation
GeometryPath build_geometry(const YarnPath& yarn_path,
                            const SurfaceGraph& surface,
                            const YarnProperties& yarn,
                            const Gauge& gauge);

// Helper: create smooth bezier curve between two points with tangent constraints
BezierSpline make_connector(const YarnProperties& yarn,
                            const Vec3& from, const Vec3& from_dir,
                            const Vec3& to, const Vec3& to_dir);

// Helper: build the curve for a single segment
// - For loop segments: creates loop shape from base to apex (child positions)
// - For non-loop segments: simple connector curve
BezierSpline build_segment_curve(
    const Vec3& prev_pos,
    const Vec3& curr_pos,
    const Vec3& next_pos,
    bool forms_loop,
    const std::vector<Vec3>& child_positions,
    const YarnProperties& yarn);

}  // namespace yarnpath

#endif // YARNPATH_GEOMETRY_BUILDER_HPP
