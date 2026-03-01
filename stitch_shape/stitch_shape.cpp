#include "stitch_shape.hpp"

namespace yarnpath {

StitchShapeParams compute_stitch_shape(
    const YarnProperties& yarn,
    const Gauge& gauge,
    LoopOrientation orientation,
    WrapDirection wrap_direction,
    WorkType work_type) {

    // Calculate base loop dimensions from yarn + gauge
    auto dims = LoopDimensions::calculate(yarn, gauge);

    StitchShapeParams params;

    // --- Orientation-based variation ---
    if (orientation == LoopOrientation::Front) {
        params.z_bulge_factor = 1.0f;     // Standard forward crossing (knit)
        params.symmetric_exit = true;
    } else if (orientation == LoopOrientation::Back) {
        params.z_bulge_factor = -1.0f;    // Full backward crossing (purl) - mirror of knit
        params.symmetric_exit = true;
    } else {  // Neutral
        params.z_bulge_factor = 0.5f;     // Shallow crossing (cast-on, yarn-over)
        params.apex_height_factor = 0.8f;
        params.symmetric_exit = true;
    }

    // --- WrapDirection-based lean (gauge-proportional) ---
    float effective_stitch_width = dims.loop_width;  // Already includes loop_size_factor via opening_diameter
    if (wrap_direction == WrapDirection::Clockwise) {
        params.apex_lean_x = effective_stitch_width * 0.4f;   // Right lean (K2tog, M1R)
    } else if (wrap_direction == WrapDirection::CounterClockwise) {
        params.apex_lean_x = -effective_stitch_width * 0.4f;  // Left lean (SSK, S2KP, M1L)
    }

    // --- WorkType-based shape ---
    if (work_type == WorkType::Transferred) {
        // Slip stitch: physically spans two rows, elongated vertically
        params.z_bulge_factor *= 0.3f;      // Very shallow in Z (barely crosses)
        params.apex_height_factor = 2.0f;   // Much taller - spans two rows
        params.height_multiplier = 1.8f;    // Visible Y elongation
    } else if (work_type == WorkType::Created) {
        if (orientation == LoopOrientation::Neutral) {
            // Yarn-over: creates an eyelet opening, wider and more yarn
            params.width_multiplier = 1.4f;
            params.apex_height_factor = 1.2f;
        } else {
            params.entry_tangent_scale = 0.7f;  // Gentler entry for other created stitches
        }
    }

    // --- Compute derived dimensions ---
    float yarn_compressed_diameter = yarn.compressed_radius * 2.0f;

    // Z bulge magnitude: how far the yarn crosses through the fabric plane
    params.z_bulge = yarn_compressed_diameter * 2.0f * params.z_bulge_factor;

    // Apply multipliers to base dimensions
    params.loop_height = dims.loop_height * params.height_multiplier;
    params.loop_width = dims.loop_width * params.width_multiplier;

    return params;
}

}  // namespace yarnpath
