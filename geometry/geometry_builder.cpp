#include "geometry_builder.hpp"
#include "physical_loop.hpp"
#include <stitch_shape/stitch_shape.hpp>
#include "logging.hpp"
#include <cmath>
#include <algorithm>
#include <limits>

namespace yarnpath {

// Phase A3: Safe normalization that returns fallback instead of zero vector
static Vec3 safe_normalized(const Vec3& v, const Vec3& fallback = Vec3(1,0,0)) {
    float len = v.length();
    return (len > 1e-8f) ? v * (1.0f / len) : fallback;
}

// Compute fabric normal from neighboring segment positions.
// The tangent along the yarn (from prev to next position) lies in the fabric plane.
// Cross with approximate vertical (Y-up) to get the normal perpendicular to the fabric.
// Surface relaxation can push positions in Z, tilting the fabric plane.
static Vec3 compute_fabric_normal(
    const std::vector<Vec3>& positions, size_t idx, float z_sign) {

    Vec3 prev = (idx > 0) ? positions[idx - 1] : positions[idx];
    Vec3 next = (idx + 1 < positions.size()) ? positions[idx + 1] : positions[idx];
    Vec3 tangent = safe_normalized(next - prev);

    // Cross tangent with up to get a vector perpendicular to the fabric
    Vec3 up = Vec3(0.0f, 1.0f, 0.0f);
    Vec3 raw_normal = tangent.cross(up);
    if (raw_normal.length() < 1e-6f) {
        raw_normal = Vec3(0.0f, 0.0f, 1.0f);  // fallback for vertical tangents
    }
    Vec3 normal = safe_normalized(raw_normal);

    // Orient so that positive normal = front side (matching z_bulge convention)
    if (normal.dot(Vec3(0, 0, z_sign)) < 0) {
        normal = normal * (-1.0f);
    }
    return normal;
}

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

    GeometryBuildState(const YarnProperties& yarn_, const Gauge& gauge_)
        : max_curvature(yarn_.max_curvature())
        , yarn_compressed_radius(yarn_.compressed_radius)
        , yarn_compressed_diameter(yarn_.compressed_radius * 2.0f)
        , loop_dims(LoopDimensions::calculate(yarn_, gauge_))
        , effective_loop_height(LoopDimensions::calculate(yarn_, gauge_).loop_height * yarn_.loop_size_factor())
        // Phase A1: Fix double loop_size_factor - loop_width already includes it via opening_diameter
        , effective_stitch_width(LoopDimensions::calculate(yarn_, gauge_).loop_width)
        , effective_opening_diameter(LoopDimensions::calculate(yarn_, gauge_).opening_diameter)
        , yarn(yarn_)
        , gauge(gauge_)
    {}
};

// Callback type for reporting each curve as it's added
// Parameters: description of the curve, the running spline after adding
using CurveAddedCallback = std::function<void(const std::string& description)>;

// Number of samples to use for curvature checking (matches test expectations)
static constexpr int CURVATURE_SAMPLES = 32;

// Subdivide a curve recursively until max_curvature is below limit,
// using dense sampling for accurate curvature measurement
static std::vector<CubicBezier> subdivide_for_curvature_dense(
    const CubicBezier& curve, float max_k, int max_depth) {
    if (max_depth <= 0 || curve.max_curvature(CURVATURE_SAMPLES) <= max_k) {
        return {curve};
    }
    auto [left, right] = curve.split(0.5f);
    auto left_result = subdivide_for_curvature_dense(left, max_k, max_depth - 1);
    auto right_result = subdivide_for_curvature_dense(right, max_k, max_depth - 1);
    left_result.insert(left_result.end(), right_result.begin(), right_result.end());
    return left_result;
}

// Check if a curve has valid (finite) control points
static bool has_valid_control_points(const CubicBezier& curve) {
    for (const auto& cp : curve.control_points) {
        if (std::isnan(cp.x) || std::isnan(cp.y) || std::isnan(cp.z) ||
            std::isinf(cp.x) || std::isinf(cp.y) || std::isinf(cp.z)) {
            return false;
        }
    }
    return true;
}

// Add a single curve segment (no subdivision)
static bool add_single_curve(
    GeometryBuildState& state,
    BezierSpline& segment_spline,
    const CubicBezier& curve,
    const std::string& description,
    const CurveAddedCallback& on_curve_added) {
    float chord = (curve.end() - curve.start()).length();
    if (chord < 1e-5f) {
        return false;  // Skip degenerate curve
    }
    if (!has_valid_control_points(curve)) {
        return false;  // Skip curves with NaN/Inf
    }
    state.running_spline.add_segment(curve);
    segment_spline.add_segment(curve);
    if (on_curve_added) on_curve_added(description);
    return true;
}

// Add a curve to both running and segment splines, skipping if degenerate.
// Curves should be curvature-correct by construction (via build_curvature_safe_hermite).
static bool add_curve_if_valid(
    GeometryBuildState& state,
    BezierSpline& segment_spline,
    const CubicBezier& curve,
    const std::string& description,
    const CurveAddedCallback& on_curve_added) {
    float chord = (curve.end() - curve.start()).length();
    if (chord < 1e-5f) {
        return false;  // Skip degenerate curve
    }
    if (!has_valid_control_points(curve)) {
        return false;  // Skip curves with NaN/Inf
    }

    state.running_spline.add_segment(curve);
    segment_spline.add_segment(curve);
    if (on_curve_added) on_curve_added(description);
    return true;
}

// LoopShapeParams is now StitchShapeParams from stitch_shape library
using LoopShapeParams = StitchShapeParams;

// Pre-calculated crossover data for a single child passing through a loop
struct CrossoverData {
    Vec3 entry;           // Where the crossing starts
    Vec3 exit;            // Where the crossing ends
    Vec3 exit_direction;  // Tangent direction at exit
};

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

    // Crossover data for children passing through this loop
    // Key: child segment ID, Value: entry crossover geometry
    std::map<SegmentId, CrossoverData> child_crossovers;

    // Exit crossover data for children passing back through this loop after the loop
    std::map<SegmentId, CrossoverData> child_exit_crossovers;

    // Clearance based on child distribution
    float clearance_radius;
};

// Derive loop shape parameters from segment topology and gauge dimensions
// Delegates to the shared compute_stitch_shape from stitch_shape library
static LoopShapeParams get_loop_shape_params(
    const YarnSegment& segment,
    const GeometryBuildState& state) {

    return compute_stitch_shape(
        state.yarn,
        state.gauge,
        segment.orientation,
        segment.wrap_direction,
        segment.work_type);
}

// Calculate apex position from child positions or default height
// Uses gauge-derived loop height for proper dimensioning
static Vec3 calculate_apex_position(
    const Vec3& curr_pos,
    const std::vector<Vec3>& child_positions,
    const GeometryBuildState& state) {

    float apex_height = state.effective_loop_height * 0.5f;
    // Ensure minimum clearance of one yarn diameter above children
    apex_height = std::max(apex_height, state.yarn_compressed_diameter);

    if (!child_positions.empty()) {
        Vec3 apex = Vec3::zero();
        for (const auto& cp : child_positions) {
            apex += cp;
        }
        apex = apex * (1.0f / static_cast<float>(child_positions.size()));
        apex.y += apex_height;
        return apex;
    } else {
        // No children - loop above current position
        return curr_pos + Vec3(0.0f, apex_height, 0.0f);
    }
}

// Compute tangent magnitudes for a Hermite curve that keep endpoint curvature ≤ max_k.
//
// For Hermite cubic Bezier with T0 = m0*d0, T1 = m1*d1, chord D = P1-P0:
//   κ(0) = 6 * |d0 × (D - m1*d1/3)| / m0²
//   κ(1) = 6 * |d1 × (-D + m0*d0/3)| / m1²
//
// We iteratively solve for the minimum m0, m1 satisfying both constraints,
// then apply a safety factor for interior curvature peaks.
static std::pair<float, float> compute_curvature_safe_magnitudes(
    const Vec3& d0_unit, const Vec3& d1_unit,
    const Vec3& chord, float max_k) {

    float dist = chord.length();
    if (dist < 1e-6f || max_k < 1e-8f) {
        return {0.0f, 0.0f};
    }

    // Precompute cross products
    Vec3 d0xD = d0_unit.cross(chord);
    Vec3 d0xd1 = d0_unit.cross(d1_unit);
    Vec3 d1xD = d1_unit.cross(chord);

    // Start with rule-of-thirds as baseline
    float m0 = dist / 3.0f;
    float m1 = dist / 3.0f;

    // Iteratively solve the coupled endpoint curvature constraints
    for (int iter = 0; iter < 3; ++iter) {
        // κ(0) = 6 * |d0×D - (m1/3)(d0×d1)| / m0²  ≤ max_k
        Vec3 num0_vec = d0xD - d0xd1 * (m1 / 3.0f);
        float num0 = num0_vec.length();
        if (num0 > 1e-8f) {
            float needed_m0 = std::sqrt(6.0f * num0 / max_k);
            m0 = std::max(m0, needed_m0);
        }

        // κ(1) = 6 * |d1×D + (m0/3)(d0×d1)| / m1²  ≤ max_k
        // (sign: d1×d0 = -(d0×d1), so d1×(-D+m0*d0/3) = -d1×D-(m0/3)(d0×d1)
        //  magnitude = |d1×D + (m0/3)(d0×d1)| )
        Vec3 num1_vec = d1xD + d0xd1 * (m0 / 3.0f);
        float num1 = num1_vec.length();
        if (num1 > 1e-8f) {
            float needed_m1 = std::sqrt(6.0f * num1 / max_k);
            m1 = std::max(m1, needed_m1);
        }
    }

    // Safety factor for interior curvature peaks (endpoints don't bound the whole curve).
    // The interior peak can be up to ~2x the endpoint curvature for S-curves,
    // so we use a generous factor to ensure compliance.
    m0 *= 2.0f;
    m1 *= 2.0f;

    // Cap at 2*dist to prevent self-intersecting control polygons
    m0 = std::min(m0, dist * 2.0f);
    m1 = std::min(m1, dist * 2.0f);

    return {m0, m1};
}

// Build one or more curvature-safe Hermite curves between two points.
// Computes tangent magnitudes analytically from the curvature constraint, then
// verifies the actual curve. If interior curvature exceeds the limit (S-curves),
// splits into two curves through the midpoint.
static std::vector<CubicBezier> build_curvature_safe_hermite_segments(
    const Vec3& start, const Vec3& start_dir,
    const Vec3& end, const Vec3& end_dir,
    float max_k, int max_splits = 3) {

    Vec3 chord = end - start;
    float dist = chord.length();
    if (dist < 1e-6f) return {CubicBezier(start, start, end, end)};

    Vec3 d0 = safe_normalized(start_dir);
    Vec3 d1 = safe_normalized(end_dir);

    auto [m0, m1] = compute_curvature_safe_magnitudes(d0, d1, chord, max_k);

    auto curve = CubicBezier::from_hermite(start, d0 * m0, end, d1 * m1);

    float actual_k = curve.max_curvature(CURVATURE_SAMPLES);
    if (actual_k <= max_k) {
        return {curve};
    }

    // Interior curvature exceeds limit. This typically happens with S-curves
    // where start_dir and end_dir create an inflection. Split into two curves
    // through the midpoint with a blended intermediate direction.
    if (max_splits <= 0) {
        // Can't split further; return best-effort curve
        return {curve};
    }

    Vec3 mid = (start + end) * 0.5f;
    // Intermediate direction: blend of chord direction and average of endpoint dirs
    Vec3 chord_dir = safe_normalized(chord);
    Vec3 mid_dir = safe_normalized(chord_dir * 0.6f + (d0 + d1) * 0.2f, chord_dir);

    auto left = build_curvature_safe_hermite_segments(
        start, start_dir, mid, mid_dir, max_k, max_splits - 1);
    auto right = build_curvature_safe_hermite_segments(
        mid, mid_dir, end, end_dir, max_k, max_splits - 1);

    left.insert(left.end(), right.begin(), right.end());
    return left;
}

// Convenience: build a single curvature-safe curve (may return multiple segments)
static CubicBezier build_curvature_safe_hermite(
    const Vec3& start, const Vec3& start_dir,
    const Vec3& end, const Vec3& end_dir,
    float max_k) {

    Vec3 chord = end - start;
    float dist = chord.length();
    if (dist < 1e-6f) return CubicBezier(start, start, end, end);

    Vec3 d0 = safe_normalized(start_dir);
    Vec3 d1 = safe_normalized(end_dir);

    auto [m0, m1] = compute_curvature_safe_magnitudes(d0, d1, chord, max_k);
    return CubicBezier::from_hermite(start, d0 * m0, end, d1 * m1);
}

// Add a connector curve, curvature-correct by construction.
// Uses multi-segment splitting for S-curves that exceed curvature limits.
static void add_connector_with_curvature_check(
    GeometryBuildState& state,
    BezierSpline& segment_spline,
    const Vec3& start, const Vec3& start_dir,
    const Vec3& end, const Vec3& end_dir,
    const std::string& description,
    const CurveAddedCallback& on_curve_added) {

    auto segments = build_curvature_safe_hermite_segments(
        start, start_dir, end, end_dir, state.max_curvature);
    for (auto& seg : segments) {
        add_curve_if_valid(state, segment_spline, seg, description, on_curve_added);
    }
}

// Build a surface-guided loop using explicit Hermite curves.
// 2 curves: rise to apex, fall back down.
// When entry ≈ exit (stitch through parent), the loop is a vertical arch
// and travel direction is derived from the apex offset.
static void build_surface_guided_loop(
    GeometryBuildState& state,
    BezierSpline& segment_spline,
    const Vec3& entry,
    const Vec3& apex,
    const Vec3& exit_pt,
    float z_bulge,
    const CurveAddedCallback& on_curve_added) {

    // Travel direction: use entry→exit if they differ, otherwise use entry→apex projected horizontal
    Vec3 entry_to_exit = exit_pt - entry;
    Vec3 travel_dir;
    if (entry_to_exit.length() > 1e-5f) {
        travel_dir = safe_normalized(entry_to_exit);
    } else {
        // Entry ≈ exit: loop is a vertical arch at the crossing point.
        // Use the horizontal component of entry→apex as the travel direction.
        Vec3 to_apex = apex - entry;
        Vec3 horizontal_to_apex = Vec3(to_apex.x, 0.0f, to_apex.z);
        travel_dir = safe_normalized(horizontal_to_apex, Vec3(1.0f, 0.0f, 0.0f));
    }

    // Tangent lean: even mix of Z (fabric normal) and travel direction.
    // The passthrough handles the pure front/back crossing; the loop lean
    // adds both Z depth and horizontal spread to avoid overlapping with it.
    Vec3 z_component = Vec3(0.0f, 0.0f, z_bulge);
    Vec3 x_component = travel_dir * z_bulge;
    Vec3 lean = (z_component + x_component) * 0.5f;
    Vec3 entry_dir = safe_normalized(travel_dir + lean * 0.5f, travel_dir);
    Vec3 apex_dir = travel_dir;
    Vec3 exit_dir = safe_normalized(travel_dir - lean * 0.5f, travel_dir);

    // Curve 1: entry -> apex (rising half with front-side Z bulge)
    auto rise_segs = build_curvature_safe_hermite_segments(
        entry, entry_dir, apex, apex_dir, state.max_curvature);
    for (auto& seg : rise_segs) {
        add_curve_if_valid(state, segment_spline, seg, "loop_rise", on_curve_added);
    }

    // Curve 2: apex -> exit (falling half with back-side Z bulge)
    auto fall_segs = build_curvature_safe_hermite_segments(
        apex, apex_dir, exit_pt, exit_dir, state.max_curvature);
    for (auto& seg : fall_segs) {
        add_curve_if_valid(state, segment_spline, seg, "loop_fall", on_curve_added);
    }
}

// Build a passthrough through a parent loop's opening using a single connector.
// The crossover is a displacement of compressed_diameter in the fabric normal direction,
// creating a smooth curve from the approach side to the loop side (or vice versa for exit).
static void build_parent_passthrough(
    GeometryBuildState& state,
    BezierSpline& segment_spline,
    const CrossoverData& crossover,
    const CurveAddedCallback& on_curve_added) {

    if (state.running_spline.empty()) return;

    Vec3 current_end = state.running_spline.segments().back().end();
    Vec3 current_dir = state.running_spline.segments().back().tangent(1.0f);
    Vec3 to_exit = crossover.exit - current_end;
    if (to_exit.length() < 1e-5f) return;

    add_connector_with_curvature_check(
        state, segment_spline,
        current_end, current_dir,
        crossover.exit, crossover.exit_direction,
        "passthrough", on_curve_added);
}

// Initialize the running spline with a tail segment pointing toward first_target.
// This ensures the first real connector doesn't need a sharp direction change.
static void initialize_running_spline(
    GeometryBuildState& state,
    const Vec3& start_pos,
    const Vec3& first_target) {

    Vec3 initial_dir = safe_normalized(first_target - start_pos);

    // Tail length: enough runway for smooth transitions
    float min_bend_radius = 1.0f / state.max_curvature;
    float init_length = std::max(state.effective_stitch_width, min_bend_radius * 3.0f);
    auto init_seg = build_curvature_safe_hermite(
        start_pos - initial_dir * init_length, initial_dir,
        start_pos, initial_dir,
        state.max_curvature);
    state.running_spline.add_segment(init_seg);
}

static std::optional<Vec3> get_segment_base_position(
    const YarnPath& yarn_path,
    const SurfaceGraph& surface,
    const YarnProperties& yarn,
    const std::map<SegmentId, std::vector<SegmentId>>& parent_map,
    const std::map<SegmentId, std::vector<SegmentId>>& children_map,
    SegmentId segment_id) {
    if (surface.has_segment(segment_id)) {
        NodeId node_id = surface.node_for_segment(segment_id);
        Vec3 base_pos = surface.node(node_id).position;
        auto parents_it = parent_map.find(segment_id);
        auto children_it = children_map.find(segment_id);
        if (parents_it == parent_map.end() ||
            (parents_it != parent_map.end() && parents_it->second.empty())) {
            // no parents, go through children, find the lowest Y,
            // make the base that - yarn diameter
            float min_y = std::numeric_limits<float>::max();
            if (children_it != children_map.end()) {
                for (SegmentId child_id : children_it->second) {
                    auto child_pos_opt = get_segment_base_position(
                        yarn_path, surface, yarn,
                        parent_map, children_map, child_id);
                    if (child_pos_opt.has_value()) {
                        Vec3 child_pos = child_pos_opt.value();
                        min_y = std::min(min_y, child_pos.y - yarn.compressed_radius * 2);
                    }
                }
            }
            if (min_y > base_pos.y) {
                yarnpath::logging::get_logger()->debug(
                    "get_segment_base_position: segment {} has no parents, adjusting base position from {} to {}",
                    segment_id, base_pos.y, min_y - yarn.compressed_radius);
                base_pos.y = min_y - yarn.compressed_radius;
            }
        }
        return base_pos;
    }
    return std::nullopt;
}

// Pre-compute all loop geometry before spline generation.
// Two-pass approach:
//   Pass 1: Compute basic geometry (shape, apex, apex_entry, apex_exit) for all loops.
//   Pass 2: Compute crossover data, referencing each child's precomputed positions.
static std::map<SegmentId, PrecomputedLoopGeometry> precompute_loop_geometry(
    const std::vector<YarnSegment>& segments,
    const std::vector<Vec3>& positions,
    const std::map<SegmentId, std::vector<Vec3>>& loop_child_positions,
    const std::map<SegmentId, std::vector<SegmentId>>& loop_child_ids,
    const GeometryBuildState& state) {

    std::map<SegmentId, PrecomputedLoopGeometry> result;

    // --- Pass 1: Basic geometry for all loops ---
    for (size_t i = 0; i < segments.size(); ++i) {
        const auto& seg = segments[i];
        if (!seg.forms_loop) continue;

        SegmentId seg_id = static_cast<SegmentId>(i);
        PrecomputedLoopGeometry geom;

        // 1. Get topology-based shape
        geom.shape = get_loop_shape_params(seg, state);

        // 2. Get child positions for this loop
        std::vector<Vec3> child_positions;
        auto child_pos_it = loop_child_positions.find(seg_id);
        if (child_pos_it != loop_child_positions.end()) {
            child_positions = child_pos_it->second;
        }

        // 3. Calculate apex position using gauge-derived height
        Vec3 curr_pos = positions[i];
        Vec3 next_pos = (i < positions.size() - 1) ? positions[i + 1] : positions[i];
        geom.apex = calculate_apex_position(curr_pos, child_positions, state);

        // Apply shape modifiers to apex
        geom.apex.x += geom.shape.apex_lean_x;
        geom.apex.y = curr_pos.y + (geom.apex.y - curr_pos.y) * geom.shape.apex_height_factor * geom.shape.height_multiplier;

        // 4. Entry/exit at base positions
        geom.apex_entry = curr_pos;
        geom.apex_exit = next_pos;

        geom.pass_through_end = curr_pos;
        geom.pass_through_dir = safe_normalized(geom.apex - curr_pos, Vec3(0, 1, 0));

        // 5. Calculate clearance radius
        geom.clearance_radius = state.yarn_compressed_diameter * 1.5f;
        if (child_positions.size() > 1) {
            geom.clearance_radius *= 1.0f + 0.2f * (child_positions.size() - 1);
        }

        result[seg_id] = geom;
    }

    // --- Pass 2: Crossover data and parent/child offsets ---
    // Now all loops have their basic geometry computed, so we can reference
    // any child's precomputed apex_entry/apex_exit.
    for (size_t i = 0; i < segments.size(); ++i) {
        const auto& seg = segments[i];
        if (!seg.forms_loop) continue;

        SegmentId seg_id = static_cast<SegmentId>(i);
        auto& geom = result[seg_id];

        // Get child IDs for this loop
        std::vector<SegmentId> child_ids;
        auto child_id_it = loop_child_ids.find(seg_id);
        if (child_id_it != loop_child_ids.end()) {
            child_ids = child_id_it->second;
        }
        if (child_ids.empty()) continue;

        // Determine the z_sign from the first child's orientation
        LoopShapeParams first_child_shape = get_loop_shape_params(segments[child_ids[0]], state);
        float z_sign = (first_child_shape.z_bulge >= 0) ? 1.0f : -1.0f;

        // Compute parent travel direction for blending offsets
        Vec3 parent_travel = safe_normalized(
            geom.apex_exit - geom.apex_entry, Vec3(1.0f, 0.0f, 0.0f));

        // Offset parent loop entry/exit: even mix of fabric normal and travel direction
        Vec3 normal_entry = compute_fabric_normal(positions, i, z_sign);
        Vec3 normal_exit = compute_fabric_normal(positions, i + 1 < positions.size() ? i + 1 : i, z_sign);
        Vec3 offset_dir_entry = safe_normalized(normal_entry + parent_travel);
        Vec3 offset_dir_exit = safe_normalized(normal_exit + parent_travel);
        geom.apex_entry = geom.apex_entry + offset_dir_entry * state.yarn_compressed_radius;
        geom.apex_exit = geom.apex_exit + offset_dir_exit * state.yarn_compressed_radius;

        for (size_t j = 0; j < child_ids.size(); ++j) {
            SegmentId child_id = child_ids[j];

            // Look up the child's precomputed loop geometry for entry/exit positions
            auto child_geom_it = result.find(child_id);
            Vec3 child_loop_entry = positions[child_id];  // fallback
            Vec3 child_loop_exit = (child_id + 1 < static_cast<SegmentId>(positions.size()))
                ? positions[child_id + 1] : positions[child_id];  // fallback
            if (child_geom_it != result.end()) {
                child_loop_entry = child_geom_it->second.apex_entry;
                child_loop_exit = child_geom_it->second.apex_exit;
            }

            // Child's travel direction
            Vec3 child_travel = safe_normalized(
                child_loop_exit - child_loop_entry,
                Vec3(1.0f, 0.0f, 0.0f));

            LoopShapeParams child_shape = get_loop_shape_params(segments[child_id], state);
            float child_z_sign = (child_shape.z_bulge >= 0) ? 1.0f : -1.0f;

            // Crossing offset: even mix of fabric normal and parent's travel direction.
            // Use parent_travel for the X bias so the crossing direction is consistent
            // with the parent loop's orientation, regardless of child travel direction.
            Vec3 normal_at_entry = compute_fabric_normal(positions, child_id, child_z_sign);
            Vec3 crossing_dir = safe_normalized(normal_at_entry + parent_travel);
            Vec3 parent_offset_entry = crossing_dir * state.yarn_compressed_radius;
            Vec3 child_offset_entry = crossing_dir * (-state.yarn_compressed_radius);

            // --- Entry crossover: crossing through parent loop at child's loop entry ---
            CrossoverData entry_crossover;
            entry_crossover.entry = child_loop_entry + parent_offset_entry;
            entry_crossover.exit = child_loop_entry + child_offset_entry;
            entry_crossover.exit_direction = child_travel;

            geom.child_crossovers[child_id] = entry_crossover;

            // --- Exit crossover: crossing back through the SAME parent loop opening ---
            // Negate the parent_travel X bias so the exit ends up on the correct side
            // for the child to continue in its travel direction.
            Vec3 exit_crossing_dir = safe_normalized(normal_at_entry - parent_travel);
            Vec3 parent_offset_exit = exit_crossing_dir * state.yarn_compressed_radius;
            Vec3 child_offset_exit = exit_crossing_dir * (-state.yarn_compressed_radius);

            CrossoverData exit_crossover;
            exit_crossover.entry = child_loop_entry + child_offset_exit;
            exit_crossover.exit = child_loop_entry + parent_offset_exit;
            exit_crossover.exit_direction = child_travel;

            geom.child_exit_crossovers[child_id] = exit_crossover;
        }
    }

    return result;
}

// Build geometry with callback for visualization/debugging
GeometryPath build_geometry_with_callback(
    const YarnPath& yarn_path,
    const SurfaceGraph& surface,
    const YarnProperties& yarn,
    const Gauge& gauge,
    const GeometryBuildCallback& callback) {

    const GeometryBuildCallback* callback_ptr = callback ? &callback : nullptr;

    auto log = yarnpath::logging::get_logger();
    log->info("build_geometry: building geometry for {} segments using surface positions",
              yarn_path.segment_count());

    GeometryPath result;
    const auto& segments = yarn_path.segments();

    if (segments.empty()) {
        return result;
    }

    // Build reverse lookup of parents for a given segment
    std::map<SegmentId, std::vector<SegmentId>> segment_parents;
    for (size_t i = 0; i < segments.size(); ++i) {
        const auto& seg = segments[i];
        for (SegmentId parent_id : seg.through) {
            segment_parents[i].push_back(parent_id);
        }
    }

    // Also a simple lookup of children for a given segment
    std::map<SegmentId, std::vector<SegmentId>> segment_children;
    for (size_t i = 0; i < segments.size(); ++i) {
        const auto& seg = segments[i];
        for (SegmentId parent_id : seg.through) {
            segment_children[parent_id].push_back(i);
        }
    }

    // Collect all positions from surface in yarn order
    std::vector<Vec3> positions;
    positions.reserve(segments.size());

    for (size_t i = 0; i < segments.size(); ++i) {
        SegmentId seg_id = static_cast<SegmentId>(i);
        auto pos_opt = get_segment_base_position(
            yarn_path, surface, yarn,
            segment_parents, segment_children, seg_id);
        if (!pos_opt.has_value()) {
            log->warn("build_geometry: segment {} position could not be determined, using origin", seg_id);
            pos_opt = Vec3::zero();
        }
        positions.push_back(pos_opt.value());
    }

    // Create a cache of which segments have parents
    std::vector<bool> segment_has_parent(segments.size(), false);
    for (size_t i = 0; i < segments.size(); ++i) {
        const auto& seg = segments[i];
        if (!seg.through.empty()) {
            segment_has_parent[i] = true;
        }
    }

    // Build reverse lookup: loop_segment_id -> positions of segments that pass through it
    std::map<SegmentId, std::vector<Vec3>> loop_apex_positions;
    std::map<SegmentId, std::vector<SegmentId>> loop_child_ids;
    for (size_t i = 0; i < segments.size(); ++i) {
        const auto& seg = segments[i];
        SegmentId child_id = static_cast<SegmentId>(i);
        for (SegmentId parent_id : seg.through) {
            loop_apex_positions[parent_id].push_back(positions[i]);
            loop_child_ids[parent_id].push_back(child_id);
        }
    }

    log->debug("build_geometry: built reverse lookup for {} loops",
               loop_apex_positions.size());

    // Initialize geometry build state with gauge-derived dimensions
    GeometryBuildState state(yarn, gauge);
    log->debug("build_geometry: effective_loop_height={:.3f}, effective_stitch_width={:.3f}, effective_opening={:.3f}",
               state.effective_loop_height, state.effective_stitch_width, state.effective_opening_diameter);

    // Pre-compute all loop geometry before spline generation
    auto precomputed_loops = precompute_loop_geometry(
        segments, positions, loop_apex_positions, loop_child_ids, state);
    log->debug("build_geometry: pre-computed geometry for {} loops", precomputed_loops.size());

    // Compute the first target point for the init tail direction.
    // This ensures the init tail points where segment 0 actually goes,
    // so there's no sharp direction change at the start.
    Vec3 first_target = positions.size() > 1 ? positions[1] : positions[0] + Vec3(1, 0, 0);
    if (!segments.empty() && segments[0].forms_loop) {
        auto precomp_it = precomputed_loops.find(0);
        if (precomp_it != precomputed_loops.end()) {
            // Point init toward the apex — the loop rises toward the apex,
            // so this gives the smoothest entry into the first loop.
            first_target = precomp_it->second.apex;
        }
    }
    initialize_running_spline(state, positions[0], first_target);

    // Build geometry segment by segment
    for (size_t i = 0; i < segments.size(); ++i) {
        SegmentId seg_id = static_cast<SegmentId>(i);
        const auto& seg = segments[i];

        SegmentGeometry geom;
        geom.segment_id = seg_id;

        Vec3 curr_pos = positions[i];
        Vec3 next_pos = (i < positions.size() - 1) ? positions[i + 1] : positions[i];

        // Get previous and next positions for normal computation
        Vec3 prev_pos = (i > 0) ? positions[i - 1] : curr_pos;
        Vec3 next_next_pos = (i + 1 < positions.size()) ? positions[i + 1] : curr_pos;

        // Get child positions if this is a loop
        std::vector<Vec3> child_positions;
        auto it = loop_apex_positions.find(seg_id);
        if (it != loop_apex_positions.end()) {
            child_positions = it->second;
        }

        // Create callback for reporting each curve as it's added
        CurveAddedCallback on_curve_added = nullptr;
        if (callback_ptr) {
            on_curve_added = [&](const std::string& description) {
                (*callback_ptr)(seg_id, description, state.running_spline);
            };
        }

        // Build curve for this segment
        BezierSpline segment_spline;

        // First, route through parent crossover points (if this segment passes through parents)
        for (SegmentId parent_id : seg.through) {
            auto parent_geom_it = precomputed_loops.find(parent_id);
            if (parent_geom_it != precomputed_loops.end()) {
                auto crossover_it = parent_geom_it->second.child_crossovers.find(seg_id);
                if (crossover_it != parent_geom_it->second.child_crossovers.end()) {
                    build_parent_passthrough(state, segment_spline, crossover_it->second, on_curve_added);
                }
            }
        }

        // Then build this segment's own geometry
        if (seg.forms_loop) {
            auto precomp_it = precomputed_loops.find(seg_id);
            if (precomp_it != precomputed_loops.end()) {
                const auto& precomp = precomp_it->second;

                // Determine loop entry/exit positions.
                // If this segment passes through parents, use the crossover points
                // directly so the loop starts/ends at the side-by-side crossing positions.
                Vec3 loop_entry = precomp.apex_entry;
                Vec3 loop_exit = precomp.apex_exit;
                for (SegmentId parent_id : seg.through) {
                    auto parent_geom_it = precomputed_loops.find(parent_id);
                    if (parent_geom_it != precomputed_loops.end()) {
                        auto entry_it = parent_geom_it->second.child_crossovers.find(seg_id);
                        if (entry_it != parent_geom_it->second.child_crossovers.end()) {
                            loop_entry = entry_it->second.exit;  // entry crossover exit = loop start
                        }
                        auto exit_it = parent_geom_it->second.child_exit_crossovers.find(seg_id);
                        if (exit_it != parent_geom_it->second.child_exit_crossovers.end()) {
                            loop_exit = exit_it->second.entry;  // exit crossover entry = loop end
                        }
                    }
                }

                // Phase B: Surface-guided loop construction

                // 1. Connector from running spline end to loop entry
                if (!state.running_spline.empty()) {
                    Vec3 spline_end = state.running_spline.segments().back().end();
                    Vec3 spline_dir = state.running_spline.segments().back().tangent(1.0f);
                    Vec3 entry_dir = safe_normalized(loop_exit - loop_entry, Vec3(1, 0, 0));
                    add_connector_with_curvature_check(
                        state, segment_spline,
                        spline_end, spline_dir,
                        loop_entry, entry_dir,
                        "loop_entry_connector", on_curve_added);
                }

                // 2. Build the 2-segment loop using positions
                //    Z-crossing encoded in tangent directions for smooth bulge
                build_surface_guided_loop(
                    state, segment_spline,
                    loop_entry,
                    precomp.apex,
                    loop_exit,
                    precomp.shape.z_bulge,
                    on_curve_added);

            } else {
                // Fallback: simple connector if no precomputed data
                if (!state.running_spline.empty()) {
                    Vec3 spline_end = state.running_spline.segments().back().end();
                    Vec3 spline_dir = state.running_spline.segments().back().tangent(1.0f);
                    Vec3 target_dir = safe_normalized(next_pos - curr_pos, Vec3(1, 0, 0));
                    add_connector_with_curvature_check(
                        state, segment_spline,
                        spline_end, spline_dir,
                        next_pos, target_dir,
                        "loop_fallback_connector", on_curve_added);
                }
            }
        } else {
            // Non-loop segment: simple connector
            if (!state.running_spline.empty()) {
                Vec3 spline_end = state.running_spline.segments().back().end();
                Vec3 spline_dir = state.running_spline.segments().back().tangent(1.0f);
                Vec3 target_dir = safe_normalized(next_pos - curr_pos, Vec3(1, 0, 0));
                add_connector_with_curvature_check(
                    state, segment_spline,
                    spline_end, spline_dir,
                    next_pos, target_dir,
                    "connector", on_curve_added);
            }
        }

        // After loop, build exit passthrough (crossing back through parent)
        for (SegmentId parent_id : seg.through) {
            auto parent_geom_it = precomputed_loops.find(parent_id);
            if (parent_geom_it != precomputed_loops.end()) {
                auto exit_it = parent_geom_it->second.child_exit_crossovers.find(seg_id);
                if (exit_it != parent_geom_it->second.child_exit_crossovers.end()) {
                    build_parent_passthrough(state, segment_spline, exit_it->second, on_curve_added);
                }
            }
        }

        geom.curve = segment_spline;

        // Calculate arc length and max curvature
        geom.arc_length = geom.curve.total_arc_length();
        geom.max_curvature = geom.curve.max_curvature();

        log->debug("  segment {}: forms_loop={}, children={}, arc_length={:.3f}, max_curvature={:.3f}, has_parent={}",
                   seg_id, seg.forms_loop, child_positions.size(),
                   geom.arc_length, geom.max_curvature,
                   segment_has_parent[seg_id] ? "true" : "false");

        result.segments_.push_back(std::move(geom));
    }

    log->info("build_geometry: built {} segment geometries", result.segments_.size());

    // Phase E: Center geometry in X after building all segments
    {
        auto [bb_min, bb_max] = result.bounding_box();
        float x_center = (bb_min.x + bb_max.x) * 0.5f;
        if (std::isfinite(x_center) && std::abs(x_center) > 1e-6f) {
            for (auto& seg_geom : result.segments_) {
                for (auto& bez : seg_geom.curve.segments()) {
                    for (auto& cp : bez.control_points) {
                        cp.x -= x_center;
                    }
                }
            }
        }
    }

    // Validation: report geometry quality issues per segment
    int curvature_violations = 0;
    int c0_violations = 0;
    int c1_violations = 0;
    int zero_length_violations = 0;

    for (size_t i = 0; i < result.segments_.size(); ++i) {
        const auto& seg_geom = result.segments_[i];
        if (seg_geom.curve.empty()) continue;

        // Check curvature
        if (seg_geom.max_curvature > state.max_curvature) {
            log->warn("build_geometry: segment {} curvature violation: {:.3f} > {:.3f}",
                       seg_geom.segment_id, seg_geom.max_curvature, state.max_curvature);
            curvature_violations++;
        }

        // Check for zero-length beziers
        for (const auto& bez : seg_geom.curve.segments()) {
            float chord = (bez.end() - bez.start()).length();
            if (chord < 1e-6f) {
                log->warn("build_geometry: segment {} has zero-length bezier (chord={:.9f})",
                           seg_geom.segment_id, chord);
                zero_length_violations++;
            }
        }

        // Check C0/C1 continuity with next segment
        if (i + 1 < result.segments_.size()) {
            const auto& next_geom = result.segments_[i + 1];
            if (!next_geom.curve.empty()) {
                Vec3 end_pt = seg_geom.curve.segments().back().end();
                Vec3 start_pt = next_geom.curve.segments().front().start();
                float gap = (end_pt - start_pt).length();
                if (gap > 1e-5f) {
                    log->warn("build_geometry: C0 gap between segments {} and {}: {:.6f}",
                               seg_geom.segment_id, next_geom.segment_id, gap);
                    c0_violations++;
                }

                Vec3 tan_a = seg_geom.curve.segments().back().tangent(1.0f);
                Vec3 tan_b = next_geom.curve.segments().front().tangent(0.0f);
                float dot = tan_a.dot(tan_b);
                if (dot < 0.95f) { // ~18 degrees
                    float angle_deg = std::acos(std::clamp(dot, -1.0f, 1.0f)) * 180.0f / 3.14159f;
                    log->warn("build_geometry: C1 tangent discontinuity between segments {} and {}: {:.1f} degrees",
                               seg_geom.segment_id, next_geom.segment_id, angle_deg);
                    c1_violations++;
                }
            }
        }
    }

    if (curvature_violations > 0 || c0_violations > 0 || c1_violations > 0 || zero_length_violations > 0) {
        log->warn("build_geometry: validation found {} curvature, {} C0, {} C1, {} zero-length violations",
                   curvature_violations, c0_violations, c1_violations, zero_length_violations);
    }

    return result;
}

// Main entry point: build geometry using surface-relaxed positions
GeometryPath build_geometry(const YarnPath& yarn_path,
                            const SurfaceGraph& surface,
                            const YarnProperties& yarn,
                            const Gauge& gauge) {
    // Call the callback version with an empty callback
    return build_geometry_with_callback(yarn_path, surface, yarn, gauge, nullptr);
}

}  // namespace yarnpath
