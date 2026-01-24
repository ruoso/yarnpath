#ifndef YARNPATH_GEOMETRY_HPP
#define YARNPATH_GEOMETRY_HPP

// Geometry layer public API
// Transforms YarnPath (1D yarn sequence with topology) into 3D geometry

#include "vec3.hpp"
#include "yarn_properties.hpp"
#include "gauge.hpp"
#include "cubic_bezier.hpp"
#include "fabric_surface.hpp"
#include "physical_loop.hpp"
#include "geometry_path.hpp"
#include "geometry_builder.hpp"

namespace yarnpath {

// Main entry point: convert YarnPath to 3D geometry
//
// Usage:
//   YarnPath yarn_path = YarnPath::from_stitch_graph(graph);
//   YarnProperties yarn = YarnProperties::worsted();
//   Gauge gauge = Gauge::worsted();
//   PlaneSurface surface;
//
//   GeometryPath geometry = GeometryPath::from_yarn_path(
//       yarn_path, yarn, gauge, surface);
//
//   // Export as OBJ for visualization
//   std::string obj = geometry.to_obj();
//
//   // Or get polyline for rendering
//   std::vector<Vec3> points = geometry.to_polyline(0.1f);

} // namespace yarnpath

#endif // YARNPATH_GEOMETRY_HPP
