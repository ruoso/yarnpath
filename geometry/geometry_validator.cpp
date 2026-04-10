#include "geometry_validator.hpp"
#include "logging.hpp"
#include <cmath>
#include <algorithm>

namespace yarnpath {

float center_geometry_x(std::vector<SegmentGeometry>& segments) {
    // Compute bounding box in X from waypoints
    float x_min = std::numeric_limits<float>::max();
    float x_max = std::numeric_limits<float>::lowest();
    for (const auto& seg_geom : segments) {
        for (const auto& wp : seg_geom.curve.waypoints()) {
            x_min = std::min(x_min, wp.x);
            x_max = std::max(x_max, wp.x);
        }
    }
    float x_center = (x_min + x_max) * 0.5f;
    if (std::isfinite(x_center) && std::abs(x_center) > 1e-6f) {
        for (auto& seg_geom : segments) {
            for (auto& wp : seg_geom.curve.waypoints()) {
                wp.x -= x_center;
            }
        }
        return x_center;
    }
    return 0.0f;
}

void validate_geometry(const std::vector<SegmentGeometry>& segments) {
    auto log = yarnpath::logging::get_logger();

    int c0_violations = 0;
    int zero_length_violations = 0;

    for (size_t i = 0; i < segments.size(); ++i) {
        const auto& seg_geom = segments[i];
        if (seg_geom.curve.empty()) continue;

        // Check for near-zero arc length
        if (seg_geom.arc_length < 1e-6f) {
            log->warn("build_geometry: segment {} has near-zero arc length ({:.9f})",
                       seg_geom.segment_id, seg_geom.arc_length);
            zero_length_violations++;
        }

        // Check C0 continuity with next segment
        if (i + 1 < segments.size()) {
            const auto& next_geom = segments[i + 1];
            if (!next_geom.curve.empty()) {
                Vec3 end_pt = seg_geom.curve.end();
                Vec3 start_pt = next_geom.curve.start();
                float gap = (end_pt - start_pt).length();
                if (gap > 1e-5f) {
                    log->warn("build_geometry: C0 gap between segments {} and {}: {:.6f}",
                               seg_geom.segment_id, next_geom.segment_id, gap);
                    c0_violations++;
                }
            }
        }
    }

    if (c0_violations > 0 || zero_length_violations > 0) {
        log->warn("build_geometry: validation found {} C0, {} zero-length violations",
                   c0_violations, zero_length_violations);
    }
}

}  // namespace yarnpath
