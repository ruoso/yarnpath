#ifndef YARNPATH_GEOMETRY_GAUGE_HPP
#define YARNPATH_GEOMETRY_GAUGE_HPP

namespace yarnpath {

// Mapping from stitch grid to fabric space
struct Gauge {
    float stitches_per_unit = 4.0f;   // Horizontal density
    float rows_per_unit = 5.0f;       // Vertical density
    float fabric_thickness = 2.0f;    // Z-extent in yarn radius units

    // Needle properties (determines loop size)
    float needle_diameter = 4.0f;     // Needle diameter in mm (US 6 = 4mm, US 8 = 5mm, etc.)

    // Derived measurements
    float stitch_width() const {
        return 1.0f / stitches_per_unit;
    }

    float row_height() const {
        return 1.0f / rows_per_unit;
    }

    // Loop opening diameter (the hole in the middle of the loop)
    // This is the needle diameter minus the yarn that wraps around it
    float loop_opening(float yarn_radius) const {
        // The loop opening is roughly the needle diameter minus twice the yarn radius
        // (yarn wraps around the needle on both sides)
        float opening = needle_diameter - 2.0f * yarn_radius;
        return opening > yarn_radius ? opening : yarn_radius;  // Minimum opening
    }

    // Loop height based on needle and yarn
    // The loop wraps around the needle, so its height is related to the needle circumference
    float loop_height(float yarn_radius, float aspect_ratio) const {
        // Loop height is approximately: (needle_diameter + yarn_diameter) * aspect_ratio * scale_factor
        // The scale factor accounts for how loops relax after being formed
        float loop_circumference = 3.14159f * (needle_diameter + 2.0f * yarn_radius);
        // Height is roughly half the circumference times aspect ratio, scaled to fabric units
        return (loop_circumference * 0.5f * aspect_ratio) / (stitches_per_unit * needle_diameter);
    }

    // Convert stitch coordinates to fabric coordinates
    float stitch_to_u(float stitch_index) const {
        return stitch_index * stitch_width();
    }

    float row_to_v(float row_index) const {
        return row_index * row_height();
    }

    // Factory methods for common gauges
    static Gauge fingering() {
        return Gauge{
            .stitches_per_unit = 7.0f,
            .rows_per_unit = 10.0f,
            .fabric_thickness = 1.0f,
            .needle_diameter = 2.75f   // US 2 needle
        };
    }

    static Gauge worsted() {
        return Gauge{
            .stitches_per_unit = 4.0f,
            .rows_per_unit = 5.0f,
            .fabric_thickness = 2.0f,
            .needle_diameter = 5.0f    // US 8 needle
        };
    }

    static Gauge bulky() {
        return Gauge{
            .stitches_per_unit = 3.0f,
            .rows_per_unit = 4.0f,
            .fabric_thickness = 3.0f,
            .needle_diameter = 8.0f    // US 11 needle
        };
    }
};

}  // namespace yarnpath

#endif // YARNPATH_GEOMETRY_GAUGE_HPP
