#ifndef YARNPATH_GEOMETRY_PATH_HPP
#define YARNPATH_GEOMETRY_PATH_HPP

#include "vec3.hpp"
#include "cubic_bezier.hpp"
#include "yarn_properties.hpp"
#include "gauge.hpp"
#include "fabric_surface.hpp"
#include "yarn_path.hpp"
#include <vector>
#include <optional>
#include <map>
#include <string>

namespace yarnpath {

// Geometry for a yarn segment (Bezier spline)
struct SegmentGeometry {
    SegmentId segment_id;
    BezierSpline curve;
    float arc_length;
    float max_curvature;
};

// Validation result for geometry constraints
struct ValidationResult {
    bool valid = true;
    std::vector<std::string> warnings;
    std::vector<std::string> errors;

    void add_warning(const std::string& msg) {
        warnings.push_back(msg);
    }

    void add_error(const std::string& msg) {
        errors.push_back(msg);
        valid = false;
    }
};

// Loop position in fabric coordinates
struct LoopPosition {
    LoopId loop_id;
    float u;           // Horizontal position in fabric (mm)
    float v;           // Vertical position in fabric (mm)
};

// Main output: 3D geometry for a yarn path
class GeometryPath {
public:
    // Construction from YarnPath
    static GeometryPath from_yarn_path(
        const YarnPath& yarn_path,
        const YarnProperties& yarn,
        const Gauge& gauge,
        const FabricSurface& surface
    );

    // Access segment geometry
    const SegmentGeometry* get_segment(SegmentId id) const;
    const std::vector<SegmentGeometry>& segments() const { return segments_; }

    // Access loop positions
    const LoopPosition* get_loop_position(LoopId id) const;
    const std::vector<LoopPosition>& loop_positions() const { return loop_positions_; }

    // Convert entire path to polyline
    std::vector<Vec3> to_polyline(float segment_length) const;

    // Convert to polyline with fixed samples per Bezier segment
    std::vector<Vec3> to_polyline_fixed(int samples_per_segment) const;

    // Validate against yarn properties
    ValidationResult validate(const YarnProperties& yarn) const;

    // Total arc length of all segments
    float total_arc_length() const;

    // Bounding box
    std::pair<Vec3, Vec3> bounding_box() const;

    // Export to OBJ format (returns string content)
    std::string to_obj(int samples_per_segment = 10) const;

private:
    friend class GeometryBuilder;

    std::vector<SegmentGeometry> segments_;
    std::vector<LoopPosition> loop_positions_;

    // Index maps for fast lookup
    std::map<SegmentId, size_t> segment_index_;
    std::map<LoopId, size_t> loop_position_index_;
};

}  // namespace yarnpath

#endif // YARNPATH_GEOMETRY_PATH_HPP
