#ifndef YARNPATH_GEOMETRY_PHYSICAL_LOOP_HPP
#define YARNPATH_GEOMETRY_PHYSICAL_LOOP_HPP

#include <math/vec3.hpp>
#include "cubic_bezier.hpp"
#include <yarn/yarn_properties.hpp>
#include <yarn/gauge.hpp>
#include <vector>

namespace yarnpath {

// A physical loop with dimensions determined by needle and yarn properties.
// This represents the actual 3D shape of a loop of yarn.
struct PhysicalLoop {
    // Loop dimensions (in mm or consistent units)
    float opening_diameter;   // The hole in the middle (yarn passes through here)
    float loop_height;        // Vertical extent of the loop
    float loop_width;         // Horizontal extent of the loop
    float yarn_compressed_radius;        // Radius of the yarn itself

    // Position in 3D space
    Vec3 center;              // Center of the loop
    Vec3 normal;              // Normal to the loop plane (direction the opening faces)

    // The loop shape as a closed Bezier spline
    // This represents the path of the yarn forming this loop
    BezierSpline shape;

    // Key points on the loop (for yarn connectivity)
    Vec3 entry_point;         // Where yarn enters this loop
    Vec3 exit_point;          // Where yarn exits this loop
    Vec3 entry_tangent;       // Tangent direction at entry (for C1 continuity)
    Vec3 exit_tangent;        // Tangent direction at exit (for C1 continuity)
    Vec3 apex_point;          // Top of the loop
    Vec3 base_point;          // Bottom of the loop (where it interlocks with parent)

    // The interior surface of the loop (for intersection testing)
    // Modeled as a disk at the loop center with opening_diameter
    Vec3 opening_center;      // Center of the opening
    float opening_radius() const { return opening_diameter / 2.0f; }

    // Create a physical loop from properties
    static PhysicalLoop from_properties(
        const YarnProperties& yarn,
        const Gauge& gauge,
        const Vec3& position,
        const Vec3& up_direction = Vec3(0.0f, 1.0f, 0.0f)
    );

    // Check if a line segment passes through the loop opening
    bool segment_passes_through(const Vec3& p1, const Vec3& p2) const;

    // Generate the Bezier spline shape of this loop
    // The shape is derived from wrapping around the needle cylinder
    void generate_shape(const Gauge& gauge);
};

// Physical dimensions calculator
struct LoopDimensions {
    float opening_diameter;
    float loop_height;
    float loop_width;
    float yarn_length;        // Total yarn in one loop

    static LoopDimensions calculate(const YarnProperties& yarn, const Gauge& gauge);
};

}  // namespace yarnpath

#endif // YARNPATH_GEOMETRY_PHYSICAL_LOOP_HPP
