#ifndef YARNPATH_GEOMETRY_LOOP_PRECOMPUTE_HPP
#define YARNPATH_GEOMETRY_LOOP_PRECOMPUTE_HPP

#include <math/vec3.hpp>
#include <math/catmull_rom_spline.hpp>
#include "crossover_geometry.hpp"
#include "geometry_build_state.hpp"
#include "position_resolver.hpp"
#include "yarn_path.hpp"
#include <yarn/yarn_properties.hpp>
#include <yarn/gauge.hpp>
#include <stitch_shape/stitch_shape.hpp>
#include <vector>
#include <map>

namespace yarnpath {

// LoopShapeParams is StitchShapeParams from stitch_shape library
using LoopShapeParams = StitchShapeParams;

// Pre-calculated geometry for a single loop (computed before spline generation)
struct PrecomputedLoopGeometry {
    // Topology-based shape (from get_loop_shape_params)
    LoopShapeParams shape;

    // Key positions (world coordinates)
    Vec3 apex;
    Vec3 apex_entry;
    Vec3 apex_exit;

    // U-shape through-opening waypoints (where yarn passes through parent opening)
    Vec3 entry_through;   // Below entry, at base Z (no z_bulge)
    Vec3 exit_through;    // Below exit, at base Z (no z_bulge)

    // Fabric normal direction for computing leg bulge at build time
    Vec3 fabric_normal;

    // Oriented wale axis: consistently points in the row-stacking direction
    Vec3 oriented_wale;

    // Course direction (along the row) — used for wrap waypoint distribution.
    Vec3 stitch_axis;

    // Needle wrap radius: curvature at apex <= 1/wrap_radius
    float wrap_radius = 0.0f;

    // Crossover slots distributed evenly along the parent loop path.
    std::vector<CrossoverSlot> crossover_slots;

    // Clearance based on child distribution
    float clearance_radius;
};

// Derive loop shape parameters from segment topology and gauge dimensions
LoopShapeParams get_loop_shape_params(
    const YarnSegment& segment,
    const YarnProperties& yarn,
    const Gauge& gauge);

// Calculate apex position from child positions or default height.
Vec3 calculate_apex_position(
    const Vec3& curr_pos,
    const std::vector<Vec3>& child_positions,
    float effective_loop_height,
    float yarn_compressed_diameter,
    const Vec3& wale_axis);

// Pre-compute all loop geometry before spline generation.
std::map<SegmentId, PrecomputedLoopGeometry> precompute_loop_geometry(
    const std::vector<YarnSegment>& segments,
    const std::vector<SegmentFrame>& frames,
    const std::map<SegmentId, std::vector<Vec3>>& loop_child_positions,
    const std::map<SegmentId, std::vector<SegmentId>>& loop_child_ids,
    const YarnProperties& yarn,
    const Gauge& gauge,
    float effective_loop_height,
    float yarn_compressed_diameter,
    float yarn_compressed_radius);

// Add a waypoint to both running and segment splines, skipping if degenerate.
bool add_waypoint_to_splines(
    GeometryBuildState& state,
    CatmullRomSpline& segment_spline,
    const Vec3& point,
    const std::string& description,
    const CurveAddedCallback& on_curve_added);

// Build a passthrough through a parent loop's opening.
void build_parent_passthrough(
    GeometryBuildState& state,
    CatmullRomSpline& segment_spline,
    const CrossoverData& crossover,
    const CurveAddedCallback& on_curve_added);

// Build the full chain for a loop-forming segment as one unified spline.
// Includes crossover entry waypoints, loop interior waypoints (with child
// crossover slots if present), and crossover exit waypoints.
void build_full_loop_chain(
    GeometryBuildState& state,
    CatmullRomSpline& segment_spline,
    const PrecomputedLoopGeometry& loop_geom,
    const std::vector<CrossoverData>& entry_crossovers,
    const std::vector<CrossoverData>& exit_crossovers,
    const CurveAddedCallback& on_curve_added,
    const WaypointAddedCallback& on_waypoint_added = nullptr);

}  // namespace yarnpath

#endif // YARNPATH_GEOMETRY_LOOP_PRECOMPUTE_HPP
