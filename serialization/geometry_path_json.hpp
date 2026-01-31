#ifndef YARNPATH_SERIALIZATION_GEOMETRY_PATH_JSON_HPP
#define YARNPATH_SERIALIZATION_GEOMETRY_PATH_JSON_HPP

#include <nlohmann/json.hpp>
#include <geometry/geometry_path.hpp>
#include <geometry/cubic_bezier.hpp>
#include "config_json.hpp"

namespace yarnpath {

// CubicBezier serialization
inline void to_json(nlohmann::json& j, const CubicBezier& bezier) {
    j = nlohmann::json::array();
    for (const auto& cp : bezier.control_points) {
        j.push_back(cp);
    }
}

inline void from_json(const nlohmann::json& j, CubicBezier& bezier) {
    if (j.size() != 4) {
        throw std::runtime_error("CubicBezier must have exactly 4 control points");
    }
    for (size_t i = 0; i < 4; ++i) {
        bezier.control_points[i] = j[i].get<Vec3>();
    }
}

// BezierSpline serialization
inline void to_json(nlohmann::json& j, const BezierSpline& spline) {
    j["segments"] = spline.segments();
}

inline void from_json(const nlohmann::json& j, BezierSpline& spline) {
    spline = BezierSpline(j["segments"].get<std::vector<CubicBezier>>());
}

// SegmentGeometry serialization
inline void to_json(nlohmann::json& j, const SegmentGeometry& geom) {
    j["segment_id"] = geom.segment_id;
    j["curve"] = geom.curve;
    j["arc_length"] = geom.arc_length;
    j["max_curvature"] = geom.max_curvature;
}

inline void from_json(const nlohmann::json& j, SegmentGeometry& geom) {
    geom.segment_id = j["segment_id"].get<SegmentId>();
    geom.curve = j["curve"].get<BezierSpline>();
    geom.arc_length = j["arc_length"].get<float>();
    geom.max_curvature = j["max_curvature"].get<float>();
}

// GeometryPath serialization
inline nlohmann::json geometry_path_to_json(const GeometryPath& geometry) {
    nlohmann::json j;
    j["segments"] = geometry.segments();
    return j;
}

// GeometryPath deserialization
inline GeometryPath geometry_path_from_json(const nlohmann::json& j) {
    auto segments = j["segments"].get<std::vector<SegmentGeometry>>();
    return GeometryPath(std::move(segments));
}

}  // namespace yarnpath

#endif // YARNPATH_SERIALIZATION_GEOMETRY_PATH_JSON_HPP
