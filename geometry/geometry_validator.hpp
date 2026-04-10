#ifndef YARNPATH_GEOMETRY_VALIDATOR_HPP
#define YARNPATH_GEOMETRY_VALIDATOR_HPP

#include "geometry_path.hpp"
#include <math/vec3.hpp>
#include <vector>

namespace yarnpath {

// Center all geometry segments so the bounding box is centered at X=0.
// Modifies waypoints in-place.
// Returns the X offset that was subtracted from all waypoints.
float center_geometry_x(std::vector<SegmentGeometry>& segments);

// Validate geometry quality and log warnings for:
//   - Zero-length spline segments
//   - C0 positional gaps between consecutive segments
void validate_geometry(const std::vector<SegmentGeometry>& segments);

}  // namespace yarnpath

#endif // YARNPATH_GEOMETRY_VALIDATOR_HPP
