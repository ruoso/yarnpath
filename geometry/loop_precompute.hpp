#ifndef YARNPATH_GEOMETRY_LOOP_PRECOMPUTE_HPP
#define YARNPATH_GEOMETRY_LOOP_PRECOMPUTE_HPP

#include <math/vec3.hpp>
#include <math/curvature_utils.hpp>
#include <math/cubic_bezier.hpp>
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
    Vec3 pass_through_end;
    Vec3 pass_through_dir;

    // Crossover slots distributed evenly along the parent loop path.
    // Ordered from entry → apex → exit. Count = sum(2 if child.forms_loop else 1).
    std::vector<CrossoverSlot> crossover_slots;

    // Clearance based on child distribution
    float clearance_radius;
};

// Derive loop shape parameters from segment topology and gauge dimensions
// Delegates to the shared compute_stitch_shape from stitch_shape library
LoopShapeParams get_loop_shape_params(
    const YarnSegment& segment,
    const YarnProperties& yarn,
    const Gauge& gauge);

// Calculate apex position from child positions or default height
// Uses gauge-derived loop height for proper dimensioning
Vec3 calculate_apex_position(
    const Vec3& curr_pos,
    const std::vector<Vec3>& child_positions,
    float effective_loop_height,
    float yarn_compressed_diameter);

// Pre-compute all loop geometry before spline generation.
// Two-pass approach:
//   Pass 1: Compute basic geometry (shape, apex, apex_entry, apex_exit) for all loops.
//   Pass 2: Compute crossover data, referencing each child's precomputed positions.
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

// Add a curve to both running and segment splines, skipping if degenerate.
// Curves should be curvature-correct by construction (via build_curvature_safe_hermite).
bool add_curve_if_valid(
    GeometryBuildState& state,
    BezierSpline& segment_spline,
    const CubicBezier& curve,
    const std::string& description,
    const CurveAddedCallback& on_curve_added);

// Add a connector curve, curvature-correct by construction.
// Uses multi-segment splitting for S-curves that exceed curvature limits.
void add_connector_with_curvature_check(
    GeometryBuildState& state,
    BezierSpline& segment_spline,
    const Vec3& start, const Vec3& start_dir,
    const Vec3& end, const Vec3& end_dir,
    const std::string& description,
    const CurveAddedCallback& on_curve_added);

// Build a surface-guided loop using explicit Hermite curves.
// 2 curves: rise to apex, fall back down.
void build_surface_guided_loop(
    GeometryBuildState& state,
    BezierSpline& segment_spline,
    const Vec3& entry,
    const Vec3& apex,
    const Vec3& exit_pt,
    float z_bulge,
    const CurveAddedCallback& on_curve_added);

// Build a surface-guided loop with waypoints at crossover slot positions.
// Slots are clustered near the apex; they replace the explicit apex waypoint.
void build_surface_guided_loop_with_crossings(
    GeometryBuildState& state,
    BezierSpline& segment_spline,
    const Vec3& entry,
    const Vec3& apex,
    const Vec3& exit_pt,
    float z_bulge,
    const std::vector<CrossoverSlot>& crossover_slots,
    const CurveAddedCallback& on_curve_added);

// Build a passthrough through a parent loop's opening using a single connector.
void build_parent_passthrough(
    GeometryBuildState& state,
    BezierSpline& segment_spline,
    const CrossoverData& crossover,
    const CurveAddedCallback& on_curve_added);

// Initialize the running spline with a tail segment pointing toward first_target.
void initialize_running_spline(
    GeometryBuildState& state,
    const Vec3& start_pos,
    const Vec3& first_target);

}  // namespace yarnpath

#endif // YARNPATH_GEOMETRY_LOOP_PRECOMPUTE_HPP
