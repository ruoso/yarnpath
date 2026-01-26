#include "geometry_path.hpp"
#include "geometry_builder.hpp"
#include <surface/surface_graph.hpp>
#include <yarn/yarn_properties.hpp>
#include <yarn/gauge.hpp>
#include "logging.hpp"
#include <sstream>
#include <iomanip>
#include <limits>

namespace yarnpath {

GeometryPath GeometryPath::from_yarn_path(
    const YarnPath& yarn_path,
    const SurfaceGraph& surface,
    const YarnProperties& yarn,
    const Gauge& gauge) {

    return build_geometry(yarn_path, surface, yarn, gauge);
}

const SegmentGeometry* GeometryPath::get_segment(SegmentId id) const {
    if (id < segments_.size()) {
        return &segments_[id];
    }
    return nullptr;
}

std::vector<Vec3> GeometryPath::to_polyline(float segment_length) const {
    std::vector<Vec3> points;

    for (const auto& seg : segments_) {
        auto seg_points = seg.curve.to_polyline(segment_length);

        // Avoid duplicating start point if continuing from previous segment
        size_t start_idx = points.empty() ? 0 : 1;
        for (size_t i = start_idx; i < seg_points.size(); ++i) {
            points.push_back(seg_points[i]);
        }
    }

    return points;
}

std::vector<Vec3> GeometryPath::to_polyline_fixed(int samples_per_segment) const {
    std::vector<Vec3> points;

    for (const auto& seg : segments_) {
        auto seg_points = seg.curve.to_polyline_fixed(samples_per_segment);

        size_t start_idx = points.empty() ? 0 : 1;
        for (size_t i = start_idx; i < seg_points.size(); ++i) {
            points.push_back(seg_points[i]);
        }
    }

    return points;
}

ValidationResult GeometryPath::validate(const YarnProperties& yarn) const {
    auto log = yarnpath::logging::get_logger();
    ValidationResult result;

    float max_curvature = yarn.max_curvature();

    log->debug("GeometryPath: validating {} segments", segments_.size());
    log->debug("GeometryPath: max_curvature={}", max_curvature);

    // Check curvature constraints
    for (const auto& seg : segments_) {
        if (seg.max_curvature > max_curvature) {
            std::string msg = "Segment " + std::to_string(seg.segment_id) +
                " exceeds max curvature: " + std::to_string(seg.max_curvature) +
                " > " + std::to_string(max_curvature);
            log->warn("GeometryPath validation: {}", msg);
            result.add_warning(msg);
        }
    }

    log->debug("GeometryPath: validation complete, {} warnings", result.warnings.size());
    return result;
}

float GeometryPath::total_arc_length() const {
    auto log = yarnpath::logging::get_logger();
    float total = 0.0f;
    for (const auto& seg : segments_) {
        total += seg.arc_length;
    }
    log->info("GeometryPath: total arc length = {}", total);
    return total;
}

std::pair<Vec3, Vec3> GeometryPath::bounding_box() const {
    auto log = yarnpath::logging::get_logger();

    if (segments_.empty()) {
        return {vec3::zero(), vec3::zero()};
    }

    Vec3 min_pt(std::numeric_limits<float>::max(),
                std::numeric_limits<float>::max(),
                std::numeric_limits<float>::max());
    Vec3 max_pt(std::numeric_limits<float>::lowest(),
                std::numeric_limits<float>::lowest(),
                std::numeric_limits<float>::lowest());

    // Check all segment control points
    for (const auto& seg : segments_) {
        for (const auto& bezier : seg.curve.segments()) {
            for (const auto& cp : bezier.control_points) {
                min_pt.x = std::min(min_pt.x, cp.x);
                min_pt.y = std::min(min_pt.y, cp.y);
                min_pt.z = std::min(min_pt.z, cp.z);
                max_pt.x = std::max(max_pt.x, cp.x);
                max_pt.y = std::max(max_pt.y, cp.y);
                max_pt.z = std::max(max_pt.z, cp.z);
            }
        }
    }

    log->info("GeometryPath: bounding box = ({}, {}, {}) to ({}, {}, {})",
              min_pt.x, min_pt.y, min_pt.z, max_pt.x, max_pt.y, max_pt.z);
    return {min_pt, max_pt};
}

std::string GeometryPath::to_obj(int samples_per_segment) const {
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(6);

    ss << "# YarnPath OBJ Export\n";
    ss << "# Segments: " << segments_.size() << "\n\n";

    // Export polyline as vertices
    auto points = to_polyline_fixed(samples_per_segment);

    for (const auto& p : points) {
        ss << "v " << p.x << " " << p.y << " " << p.z << "\n";
    }

    // Create line elements
    if (points.size() > 1) {
        ss << "\n# Yarn path\nl";
        for (size_t i = 1; i <= points.size(); ++i) {
            ss << " " << i;
        }
        ss << "\n";
    }

    return ss.str();
}

}  // namespace yarnpath
