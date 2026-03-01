#include "geometry_validator.hpp"
#include "logging.hpp"
#include <cmath>
#include <algorithm>

namespace yarnpath {

void center_geometry_x(std::vector<SegmentGeometry>& segments) {
    // Compute bounding box in X
    float x_min = std::numeric_limits<float>::max();
    float x_max = std::numeric_limits<float>::lowest();
    for (const auto& seg_geom : segments) {
        for (const auto& bez : seg_geom.curve.segments()) {
            for (const auto& cp : bez.control_points) {
                x_min = std::min(x_min, cp.x);
                x_max = std::max(x_max, cp.x);
            }
        }
    }
    float x_center = (x_min + x_max) * 0.5f;
    if (std::isfinite(x_center) && std::abs(x_center) > 1e-6f) {
        for (auto& seg_geom : segments) {
            for (auto& bez : seg_geom.curve.segments()) {
                for (auto& cp : bez.control_points) {
                    cp.x -= x_center;
                }
            }
        }
    }
}

void validate_geometry(const std::vector<SegmentGeometry>& segments,
                       float max_curvature) {
    auto log = yarnpath::logging::get_logger();

    int curvature_violations = 0;
    int c0_violations = 0;
    int c1_violations = 0;
    int zero_length_violations = 0;

    for (size_t i = 0; i < segments.size(); ++i) {
        const auto& seg_geom = segments[i];
        if (seg_geom.curve.empty()) continue;

        // Check curvature
        if (seg_geom.max_curvature > max_curvature) {
            log->warn("build_geometry: segment {} curvature violation: {:.3f} > {:.3f}",
                       seg_geom.segment_id, seg_geom.max_curvature, max_curvature);
            curvature_violations++;
        }

        // Check for zero-length beziers
        for (const auto& bez : seg_geom.curve.segments()) {
            float chord = (bez.end() - bez.start()).length();
            if (chord < 1e-6f) {
                log->warn("build_geometry: segment {} has zero-length bezier (chord={:.9f})",
                           seg_geom.segment_id, chord);
                zero_length_violations++;
            }
        }

        // Check C0/C1 continuity with next segment
        if (i + 1 < segments.size()) {
            const auto& next_geom = segments[i + 1];
            if (!next_geom.curve.empty()) {
                Vec3 end_pt = seg_geom.curve.segments().back().end();
                Vec3 start_pt = next_geom.curve.segments().front().start();
                float gap = (end_pt - start_pt).length();
                if (gap > 1e-5f) {
                    log->warn("build_geometry: C0 gap between segments {} and {}: {:.6f}",
                               seg_geom.segment_id, next_geom.segment_id, gap);
                    c0_violations++;
                }

                Vec3 tan_a = seg_geom.curve.segments().back().tangent(1.0f);
                Vec3 tan_b = next_geom.curve.segments().front().tangent(0.0f);
                float dot = tan_a.dot(tan_b);
                if (dot < 0.95f) { // ~18 degrees
                    float angle_deg = std::acos(std::clamp(dot, -1.0f, 1.0f)) * 180.0f / 3.14159f;
                    log->warn("build_geometry: C1 tangent discontinuity between segments {} and {}: {:.1f} degrees",
                               seg_geom.segment_id, next_geom.segment_id, angle_deg);
                    c1_violations++;
                }
            }
        }
    }

    if (curvature_violations > 0 || c0_violations > 0 || c1_violations > 0 || zero_length_violations > 0) {
        log->warn("build_geometry: validation found {} curvature, {} C0, {} C1, {} zero-length violations",
                   curvature_violations, c0_violations, c1_violations, zero_length_violations);
    }
}

}  // namespace yarnpath
