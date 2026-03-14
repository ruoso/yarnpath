#ifndef YARNPATH_STITCH_SHAPE_STITCH_SHAPE_HPP
#define YARNPATH_STITCH_SHAPE_STITCH_SHAPE_HPP

#include "stitch_enums.hpp"
#include "loop_dimensions.hpp"
#include <math/vec3.hpp>
#include <yarn/yarn_properties.hpp>
#include <yarn/gauge.hpp>
#include <cmath>

namespace yarnpath {

// Parameters describing the 3D shape of a stitch, computed from topology and yarn/gauge properties.
// Used by both the surface relaxer (for bounding volumes and initial Z placement)
// and the geometry builder (for Bézier spline construction).
//
// Shape is determined by three independent axes:
//   - LoopOrientation (Front/Back/Neutral) → sets z_bulge_factor and apex_height_factor
//   - WrapDirection (Clockwise/CounterClockwise/None) → sets apex_lean_x
//   - WorkType (Worked/Transferred/Created) → modifies z_bulge_factor, height/width multipliers
//
// Derived values (z_bulge, loop_width, loop_height) are computed last from
// the modifier fields and the base LoopDimensions.
struct StitchShapeParams {
    // Signed Z depth of the yarn crossing through the fabric plane.
    // Positive = yarn bulges toward the viewer (knit on RS).
    // Negative = yarn bulges away from the viewer (purl on RS).
    // Magnitude = compressed_diameter × 2 × z_bulge_factor.
    float z_bulge = 0.0f;

    // Final loop dimensions after applying width_multiplier and height_multiplier
    // to the base values from LoopDimensions::calculate().
    float loop_height = 0.0f;
    float loop_width = 0.0f;

    // --- Shape modifier fields (set by orientation/wrap/work combos) ---

    // Sign and magnitude of the Z crossing relative to compressed_diameter × 2.
    // +1.0 = full forward crossing (knit), -1.0 = full backward (purl),
    // 0.5 = shallow (neutral/cast-on). WorkType::Transferred multiplies this by 0.3.
    float z_bulge_factor = 1.0f;

    // Horizontal lean of the loop apex, in mm. Nonzero for directional decreases:
    // positive = right lean (K2tog, Clockwise), negative = left lean (SSK, CounterClockwise).
    // Magnitude = loop_width × 0.4.
    float apex_lean_x = 0.0f;

    // Vertical scaling of the loop apex relative to base loop height.
    // Neutral orientation sets 0.8 (shorter). Transferred sets 2.0 (spans two rows).
    // YarnOver (Neutral+Created) sets 1.2.
    float apex_height_factor = 1.0f;

    // Whether the exit leg of the loop mirrors the entry leg.
    // True for all standard stitches; reserved for future asymmetric constructions.
    bool symmetric_exit = true;

    // Scaling factor for the entry Bézier tangent length.
    // 1.0 = default. Created+non-Neutral uses 0.7 for a gentler entry curve.
    float entry_tangent_scale = 1.0f;

    // Multiplier on base loop_width from LoopDimensions. YarnOver uses 1.4 (wider eyelet).
    float width_multiplier = 1.0f;

    // Multiplier on base loop_height from LoopDimensions. Transferred uses 1.8 (elongated slip).
    float height_multiplier = 1.0f;

    // Compute axis-aligned bounding half-extents for this stitch shape.
    // The bounding box is oriented as: X = course (width), Y = wale (height), Z = fabric normal (depth).
    // compressed_radius is added to the depth to account for yarn thickness.
    Vec3 bounding_half_extent(float compressed_radius) const {
        return Vec3(
            loop_width * 0.5f,
            loop_height * 0.5f,
            std::abs(z_bulge) + compressed_radius
        );
    }
};

// Compute the 3D shape parameters for a stitch from its topology and physical properties.
// This is the single source of truth for stitch shape, used by both surface and geometry phases.
StitchShapeParams compute_stitch_shape(
    const YarnProperties& yarn,
    const Gauge& gauge,
    LoopOrientation orientation,
    WrapDirection wrap_direction,
    WorkType work_type);

}  // namespace yarnpath

#endif // YARNPATH_STITCH_SHAPE_STITCH_SHAPE_HPP
