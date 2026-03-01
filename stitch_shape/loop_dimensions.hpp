#ifndef YARNPATH_STITCH_SHAPE_LOOP_DIMENSIONS_HPP
#define YARNPATH_STITCH_SHAPE_LOOP_DIMENSIONS_HPP

#include <yarn/yarn_properties.hpp>
#include <yarn/gauge.hpp>

namespace yarnpath {

// Physical dimensions of a knitting loop, derived from yarn and gauge properties.
// This is a pure computation with no geometry dependencies.
struct LoopDimensions {
    float opening_diameter;   // The hole in the middle (yarn passes through here)
    float loop_height;        // Vertical extent of the loop
    float loop_width;         // Horizontal extent of the loop
    float yarn_length;        // Total yarn consumed in one loop

    // Calculate dimensions from yarn properties and gauge
    static LoopDimensions calculate(const YarnProperties& yarn, const Gauge& gauge);
};

}  // namespace yarnpath

#endif // YARNPATH_STITCH_SHAPE_LOOP_DIMENSIONS_HPP
