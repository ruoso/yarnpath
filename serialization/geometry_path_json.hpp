#ifndef YARNPATH_SERIALIZATION_GEOMETRY_PATH_JSON_HPP
#define YARNPATH_SERIALIZATION_GEOMETRY_PATH_JSON_HPP

#include <nlohmann/json.hpp>
#include <geometry/geometry_path.hpp>
#include <math/catmull_rom_spline.hpp>
#include "config_json.hpp"

namespace yarnpath {

// CatmullRomSpline serialization
inline void to_json(nlohmann::json& j, const CatmullRomSpline& spline) {
    j["waypoints"] = nlohmann::json::array();
    for (const auto& wp : spline.waypoints()) {
        j["waypoints"].push_back(wp);
    }
}

inline void from_json(const nlohmann::json& j, CatmullRomSpline& spline) {
    spline = CatmullRomSpline();
    if (j.contains("waypoints")) {
        for (const auto& wp_json : j["waypoints"]) {
            spline.add_waypoint(wp_json.get<Vec3>());
        }
    }
}

// SegmentGeometry serialization
inline void to_json(nlohmann::json& j, const SegmentGeometry& geom) {
    j["segment_id"] = geom.segment_id;
    j["curve"] = geom.curve;
    j["arc_length"] = geom.arc_length;
}

inline void from_json(const nlohmann::json& j, SegmentGeometry& geom) {
    geom.segment_id = j["segment_id"].get<SegmentId>();
    geom.curve = j["curve"].get<CatmullRomSpline>();
    geom.arc_length = j["arc_length"].get<float>();
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
