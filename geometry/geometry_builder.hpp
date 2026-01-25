#ifndef YARNPATH_GEOMETRY_BUILDER_HPP
#define YARNPATH_GEOMETRY_BUILDER_HPP

#include "geometry_path.hpp"
#include "yarn_path.hpp"
#include "yarn_properties.hpp"
#include "gauge.hpp"
#include "physical_loop.hpp"
#include "vec3.hpp"
#include "cubic_bezier.hpp"
#include <vector>
#include <map>

namespace yarnpath {

// Immutable context for geometry building (doesn't change during build)
struct GeometryContext {
    const YarnPath& yarn_path;
    const YarnProperties& yarn;
    const Gauge& gauge;
    LoopDimensions loop_dims;

    static GeometryContext create(const YarnPath& yarn_path,
                                   const YarnProperties& yarn,
                                   const Gauge& gauge);
};

// Current state of the yarn as we build geometry (passed through functions)
struct YarnState {
    Vec3 position;
    Vec3 direction;
    std::map<SegmentId, PhysicalLoop> formed_loops;
};

// Result of building a single segment
struct SegmentBuildResult {
    SegmentGeometry geometry;
    YarnState new_state;
};

// Build complete geometry from yarn path - main entry point
GeometryPath build_geometry(const YarnPath& yarn_path,
                            const YarnProperties& yarn,
                            const Gauge& gauge);

// Build geometry for a single segment
SegmentBuildResult build_segment(const GeometryContext& ctx,
                                  const YarnState& state,
                                  SegmentId seg_id,
                                  const YarnSegment& seg);

// Build a cast-on loop (no parents)
SegmentBuildResult build_cast_on_loop(const GeometryContext& ctx,
                                       const YarnState& state,
                                       SegmentId seg_id);

// Build a segment that passes through parent loops and forms a new loop
SegmentBuildResult build_through_and_form_loop(const GeometryContext& ctx,
                                                const YarnState& state,
                                                SegmentId seg_id,
                                                const std::vector<SegmentId>& through);

// Build a segment that just passes through loops (no new loop formed)
SegmentBuildResult build_through_only(const GeometryContext& ctx,
                                       const YarnState& state,
                                       SegmentId seg_id,
                                       const std::vector<SegmentId>& through);

// Build a simple connector segment (no loops involved)
SegmentBuildResult build_connector_segment(const GeometryContext& ctx,
                                            const YarnState& state,
                                            SegmentId seg_id);

// Helper: create smooth bezier curve between two points with tangent constraints
BezierSpline make_connector(const YarnProperties& yarn,
                            const Vec3& from, const Vec3& from_dir,
                            const Vec3& to, const Vec3& to_dir);

// Helper: create curve that passes through the opening of a loop
// Returns the spline segment and the exit position/direction
struct PassThroughResult {
    BezierSpline spline;
    Vec3 exit_pos;
    Vec3 exit_dir;
};

PassThroughResult pass_through_loop(const GeometryContext& ctx,
                                     const PhysicalLoop& loop,
                                     const Vec3& from, const Vec3& from_dir);

// Create a physical loop at a given position
PhysicalLoop create_loop_at_position(const GeometryContext& ctx,
                                      const Vec3& position);

}  // namespace yarnpath

#endif // YARNPATH_GEOMETRY_BUILDER_HPP
