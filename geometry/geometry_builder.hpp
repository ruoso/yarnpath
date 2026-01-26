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
#include <functional>

namespace yarnpath {

// Callback invoked after each curve segment is added during geometry building
// Parameters: segment_id, description of curve, current spline being built
using GeometryBuildCallback = std::function<void(
    SegmentId segment_id,
    const std::string& curve_description,
    const BezierSpline& current_spline)>;

// Build complete geometry from yarn path using relaxed surface positions
// The surface graph provides the actual positions from physics simulation
GeometryPath build_geometry(const YarnPath& yarn_path,
                            const SurfaceGraph& surface,
                            const YarnProperties& yarn,
                            const Gauge& gauge);

// Build geometry with callback for visualization/debugging
// The callback is invoked after each curve segment is added
GeometryPath build_geometry_with_callback(
    const YarnPath& yarn_path,
    const SurfaceGraph& surface,
    const YarnProperties& yarn,
    const Gauge& gauge,
    const GeometryBuildCallback& callback);

// Helper: create smooth bezier curve between two points with tangent constraints
BezierSpline make_connector(const YarnProperties& yarn,
                            const Vec3& from, const Vec3& from_dir,
                            const Vec3& to, const Vec3& to_dir);


}  // namespace yarnpath

#endif // YARNPATH_GEOMETRY_BUILDER_HPP
