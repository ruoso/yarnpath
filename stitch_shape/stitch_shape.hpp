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
struct StitchShapeParams {
    // Signed Z depth: positive = front bulge (knit), negative = back bulge (purl)
    float z_bulge = 0.0f;

    // Loop dimensions (from LoopDimensions, possibly modified by multipliers)
    float loop_height = 0.0f;
    float loop_width = 0.0f;

    // Shape modifiers (used primarily by geometry builder for Bézier construction)
    float z_bulge_factor = 1.0f;        // Sign and magnitude of Z crossing
    float apex_lean_x = 0.0f;           // X-offset for twist/lean in decreases
    float apex_height_factor = 1.0f;    // Multiplier for apex height
    bool symmetric_exit = true;         // Whether exit mirrors entry
    float entry_tangent_scale = 1.0f;   // Entry curve sharpness adjustment
    float width_multiplier = 1.0f;      // X-span scaling relative to base
    float height_multiplier = 1.0f;     // Y-span scaling relative to base

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
