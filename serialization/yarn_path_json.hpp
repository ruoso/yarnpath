#ifndef YARNPATH_SERIALIZATION_YARN_PATH_JSON_HPP
#define YARNPATH_SERIALIZATION_YARN_PATH_JSON_HPP

#include <nlohmann/json.hpp>
#include <yarn_path.hpp>

namespace yarnpath {

// YarnSegment serialization
inline void to_json(nlohmann::json& j, const YarnSegment& segment) {
    j["through"] = segment.through;
    j["forms_loop"] = segment.forms_loop;
}

inline void from_json(const nlohmann::json& j, YarnSegment& segment) {
    segment.through = j["through"].get<std::vector<SegmentId>>();
    segment.forms_loop = j["forms_loop"].get<bool>();
}

// YarnPath serialization
inline nlohmann::json yarn_path_to_json(const YarnPath& path) {
    nlohmann::json j;
    j["segments"] = path.segments();
    return j;
}

// YarnPath deserialization
inline YarnPath yarn_path_from_json(const nlohmann::json& j) {
    auto segments = j["segments"].get<std::vector<YarnSegment>>();
    return YarnPath(std::move(segments));
}

}  // namespace yarnpath

#endif // YARNPATH_SERIALIZATION_YARN_PATH_JSON_HPP
