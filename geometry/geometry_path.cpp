#include "geometry_path.hpp"
#include "geometry_builder.hpp"
#include <sstream>
#include <iomanip>
#include <limits>

namespace yarnpath {

GeometryPath GeometryPath::from_yarn_path(
    const YarnPath& yarn_path,
    const YarnProperties& yarn,
    const Gauge& gauge,
    const FabricSurface& surface) {

    GeometryBuilder builder(yarn_path, yarn, gauge, surface);
    return builder.build();
}

const AnchorGeometry* GeometryPath::get_anchor(AnchorId id) const {
    auto it = anchor_index_.find(id);
    if (it != anchor_index_.end()) {
        return &anchors_[it->second];
    }
    return nullptr;
}

const SegmentGeometry* GeometryPath::get_segment(SegmentId id) const {
    auto it = segment_index_.find(id);
    if (it != segment_index_.end()) {
        return &segments_[it->second];
    }
    return nullptr;
}

const LoopPosition* GeometryPath::get_loop_position(LoopId id) const {
    auto it = loop_position_index_.find(id);
    if (it != loop_position_index_.end()) {
        return &loop_positions_[it->second];
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
    ValidationResult result;

    float max_curvature = yarn.max_curvature();
    float min_clearance = yarn.min_clearance();

    // Check curvature constraints
    for (const auto& seg : segments_) {
        if (seg.max_curvature > max_curvature) {
            result.add_warning(
                "Segment " + std::to_string(seg.segment_id) +
                " exceeds max curvature: " + std::to_string(seg.max_curvature) +
                " > " + std::to_string(max_curvature));
        }
    }

    // Check clearance between non-adjacent anchors
    // This is a simplified O(n^2) check - could be optimized with spatial indexing
    for (size_t i = 0; i < anchors_.size(); ++i) {
        for (size_t j = i + 2; j < anchors_.size(); ++j) {
            float dist = anchors_[i].position.distance_to(anchors_[j].position);
            if (dist < min_clearance) {
                result.add_warning(
                    "Anchors " + std::to_string(anchors_[i].anchor_id) +
                    " and " + std::to_string(anchors_[j].anchor_id) +
                    " are closer than min clearance: " +
                    std::to_string(dist) + " < " + std::to_string(min_clearance));
            }
        }
    }

    return result;
}

float GeometryPath::total_arc_length() const {
    float total = 0.0f;
    for (const auto& seg : segments_) {
        total += seg.arc_length;
    }
    return total;
}

std::pair<Vec3, Vec3> GeometryPath::bounding_box() const {
    if (anchors_.empty()) {
        return {vec3::zero(), vec3::zero()};
    }

    Vec3 min_pt = anchors_[0].position;
    Vec3 max_pt = anchors_[0].position;

    for (const auto& anchor : anchors_) {
        min_pt.x = std::min(min_pt.x, anchor.position.x);
        min_pt.y = std::min(min_pt.y, anchor.position.y);
        min_pt.z = std::min(min_pt.z, anchor.position.z);
        max_pt.x = std::max(max_pt.x, anchor.position.x);
        max_pt.y = std::max(max_pt.y, anchor.position.y);
        max_pt.z = std::max(max_pt.z, anchor.position.z);
    }

    // Also check segment control points
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

    return {min_pt, max_pt};
}

std::string GeometryPath::to_obj(int samples_per_segment) const {
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(6);

    ss << "# YarnPath OBJ Export\n";
    ss << "# Anchors: " << anchors_.size() << "\n";
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
