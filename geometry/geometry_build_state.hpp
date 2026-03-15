#ifndef YARNPATH_GEOMETRY_BUILD_STATE_HPP
#define YARNPATH_GEOMETRY_BUILD_STATE_HPP

#include <math/cubic_bezier.hpp>
#include <yarn/yarn_properties.hpp>
#include <yarn/gauge.hpp>
#include <stitch_shape/loop_dimensions.hpp>
#include <functional>
#include <string>

namespace yarnpath {

// Helper struct to track state during geometry building
struct GeometryBuildState {
    BezierSpline running_spline;
    float max_curvature;
    float yarn_compressed_radius;       // Radius of the yarn
    float yarn_compressed_diameter;     // Diameter = 2 * compressed_radius, minimum distance between yarn centers

    // Gauge-derived dimensions (from LoopDimensions::calculate)
    LoopDimensions loop_dims;
    float effective_loop_height;        // loop_dims.loop_height * yarn.loop_size_factor()
    float effective_stitch_width;       // loop_dims.loop_width (already includes loop_size_factor via opening_diameter)
    float effective_opening_diameter;   // loop_dims.opening_diameter

    // Store references for compute_stitch_shape calls
    const YarnProperties& yarn;
    const Gauge& gauge;

    GeometryBuildState(const YarnProperties& yarn_, const Gauge& gauge_);
};

// Callback type for reporting each curve as it's added
// Parameters: description of the curve, the running spline after adding
using CurveAddedCallback = std::function<void(const std::string& description)>;

// Callback type for reporting each waypoint as it's added to a chain
// Parameters: name of the waypoint, position
using WaypointAddedCallback = std::function<void(const std::string& name, const Vec3& position)>;

}  // namespace yarnpath

#endif // YARNPATH_GEOMETRY_BUILD_STATE_HPP
