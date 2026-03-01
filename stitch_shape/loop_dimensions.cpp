#include "loop_dimensions.hpp"
#include <cmath>

namespace yarnpath {

LoopDimensions LoopDimensions::calculate(const YarnProperties& yarn, const Gauge& gauge) {
    LoopDimensions dim;

    // Loop opening is needle diameter minus yarn wrapping around both sides
    dim.opening_diameter = gauge.needle_diameter - 2.0f * yarn.compressed_radius;
    if (dim.opening_diameter < yarn.compressed_radius) {
        dim.opening_diameter = yarn.compressed_radius;  // Minimum opening
    }

    // Apply tension: tighter knitting = smaller loops
    dim.opening_diameter *= yarn.loop_size_factor();

    // Loop height: yarn wraps around needle, so height is related to
    // half the circumference of (needle + yarn thickness)
    float wrap_circumference = 3.14159f * (gauge.needle_diameter + 2.0f * yarn.compressed_radius);
    dim.loop_height = wrap_circumference * 0.5f;

    // Loop width is similar to opening diameter plus yarn on both sides
    dim.loop_width = dim.opening_diameter + 2.0f * yarn.compressed_radius;

    // Total yarn length in one loop (approximate)
    dim.yarn_length = wrap_circumference * (1.0f + yarn.loop_slack);

    return dim;
}

}  // namespace yarnpath
