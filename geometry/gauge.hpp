#ifndef YARNPATH_GEOMETRY_GAUGE_HPP
#define YARNPATH_GEOMETRY_GAUGE_HPP

namespace yarnpath {

// Mapping from stitch grid to fabric space
struct Gauge {
    float stitches_per_unit = 4.0f;   // Horizontal density
    float rows_per_unit = 5.0f;       // Vertical density
    float fabric_thickness = 2.0f;    // Z-extent in yarn radius units

    // Derived measurements
    float stitch_width() const {
        return 1.0f / stitches_per_unit;
    }

    float row_height() const {
        return 1.0f / rows_per_unit;
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
            .fabric_thickness = 1.0f
        };
    }

    static Gauge worsted() {
        return Gauge{
            .stitches_per_unit = 4.0f,
            .rows_per_unit = 5.0f,
            .fabric_thickness = 2.0f
        };
    }

    static Gauge bulky() {
        return Gauge{
            .stitches_per_unit = 3.0f,
            .rows_per_unit = 4.0f,
            .fabric_thickness = 3.0f
        };
    }
};

}  // namespace yarnpath

#endif // YARNPATH_GEOMETRY_GAUGE_HPP
