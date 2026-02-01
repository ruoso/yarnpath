#ifndef YARNPATH_SERIALIZATION_YARN_PATH_JSON_HPP
#define YARNPATH_SERIALIZATION_YARN_PATH_JSON_HPP

#include <nlohmann/json.hpp>
#include <yarn_path.hpp>

namespace yarnpath {

// YarnSegment serialization
inline void to_json(nlohmann::json& j, const YarnSegment& segment) {
    j["through"] = segment.through;
    j["forms_loop"] = segment.forms_loop;
    j["target_yarn_length"] = segment.target_yarn_length;

    // Serialize orientation enum as string
    switch (segment.orientation) {
        case YarnSegment::LoopOrientation::Front:
            j["orientation"] = "front";
            break;
        case YarnSegment::LoopOrientation::Back:
            j["orientation"] = "back";
            break;
        case YarnSegment::LoopOrientation::Neutral:
            j["orientation"] = "neutral";
            break;
    }

    // Serialize wrap_direction enum as string
    switch (segment.wrap_direction) {
        case YarnSegment::WrapDirection::Clockwise:
            j["wrap_direction"] = "clockwise";
            break;
        case YarnSegment::WrapDirection::CounterClockwise:
            j["wrap_direction"] = "counterclockwise";
            break;
        case YarnSegment::WrapDirection::None:
            j["wrap_direction"] = "none";
            break;
    }

    // Serialize work_type enum as string
    switch (segment.work_type) {
        case YarnSegment::WorkType::Worked:
            j["work_type"] = "worked";
            break;
        case YarnSegment::WorkType::Transferred:
            j["work_type"] = "transferred";
            break;
        case YarnSegment::WorkType::Created:
            j["work_type"] = "created";
            break;
    }
}

inline void from_json(const nlohmann::json& j, YarnSegment& segment) {
    segment.through = j["through"].get<std::vector<SegmentId>>();
    segment.forms_loop = j["forms_loop"].get<bool>();
    segment.target_yarn_length = j.value("target_yarn_length", 0.0f);

    // Deserialize orientation enum
    std::string orientation_str = j.value("orientation", "neutral");
    if (orientation_str == "front") {
        segment.orientation = YarnSegment::LoopOrientation::Front;
    } else if (orientation_str == "back") {
        segment.orientation = YarnSegment::LoopOrientation::Back;
    } else {
        segment.orientation = YarnSegment::LoopOrientation::Neutral;
    }

    // Deserialize wrap_direction enum
    std::string wrap_str = j.value("wrap_direction", "none");
    if (wrap_str == "clockwise") {
        segment.wrap_direction = YarnSegment::WrapDirection::Clockwise;
    } else if (wrap_str == "counterclockwise") {
        segment.wrap_direction = YarnSegment::WrapDirection::CounterClockwise;
    } else {
        segment.wrap_direction = YarnSegment::WrapDirection::None;
    }

    // Deserialize work_type enum
    std::string work_str = j.value("work_type", "worked");
    if (work_str == "worked") {
        segment.work_type = YarnSegment::WorkType::Worked;
    } else if (work_str == "transferred") {
        segment.work_type = YarnSegment::WorkType::Transferred;
    } else if (work_str == "created") {
        segment.work_type = YarnSegment::WorkType::Created;
    }
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
