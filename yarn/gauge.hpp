#ifndef YARNPATH_YARN_GAUGE_HPP
#define YARNPATH_YARN_GAUGE_HPP

namespace yarnpath {

// Mapping from stitch grid to fabric space
struct Gauge {
    // Needle properties (determines loop size)
    float needle_diameter = 4.0f;     // Needle diameter in mm (US 6 = 4mm, US 8 = 5mm, etc.)

    // Loop height based on needle and yarn (VERTICAL spacing)
    // The loop wraps around the needle, so its height is related to the needle circumference
    float loop_height(float yarn_compressed_radius) const {
        // Loop height is approximately: (needle_diameter + yarn_compressed_diameter) * aspect_ratio * scale_factor
        // The scale factor accounts for how loops relax after being formed
        float loop_circumference = 3.14159f * (needle_diameter + 2.0f * yarn_compressed_radius);
        // Height is roughly half the circumference
        return (loop_circumference * 0.5f);
    }

    // Stitch width (HORIZONTAL spacing along a row)
    // In knitting, stitches along a row are spaced approximately by the needle diameter
    // This represents the distance between consecutive loop centers in the same row
    float stitch_width(float yarn_compressed_radius) const {
        // Base width is needle diameter (where the loop forms)
        // Add some spacing for yarn thickness
        return needle_diameter + yarn_compressed_radius * 2.0f;
    }

    // Factory methods for common gauges
    static Gauge fingering() {
        return Gauge{
            .needle_diameter = 2.75f   // US 2 needle
        };
    }

    static Gauge worsted() {
        return Gauge{
            .needle_diameter = 5.0f    // US 8 needle
        };
    }

    static Gauge bulky() {
        return Gauge{
            .needle_diameter = 8.0f    // US 11 needle
        };
    }
};

}  // namespace yarnpath

#endif // YARNPATH_YARN_GAUGE_HPP
