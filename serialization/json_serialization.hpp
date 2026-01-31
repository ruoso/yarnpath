#ifndef YARNPATH_SERIALIZATION_JSON_SERIALIZATION_HPP
#define YARNPATH_SERIALIZATION_JSON_SERIALIZATION_HPP

#include <nlohmann/json.hpp>
#include <fstream>
#include <sstream>
#include <string>
#include <chrono>
#include <iomanip>

namespace yarnpath::json {

// Version of the serialization format
constexpr const char* SERIALIZATION_VERSION = "0.1.0";

// Metadata wrapper for serialized data
struct SerializedData {
    std::string version = SERIALIZATION_VERSION;
    std::string step;
    std::string timestamp;
    std::string source_file;
    nlohmann::json config;
    nlohmann::json stats;
    nlohmann::json data;

    // Convert to JSON
    nlohmann::json to_json() const {
        nlohmann::json j;
        j["version"] = version;
        j["step"] = step;
        if (!timestamp.empty()) j["timestamp"] = timestamp;
        if (!source_file.empty()) j["source_file"] = source_file;
        if (!config.is_null()) j["config"] = config;
        if (!stats.is_null()) j["stats"] = stats;
        j["data"] = data;
        return j;
    }

    // Load from JSON
    static SerializedData from_json(const nlohmann::json& j) {
        SerializedData result;
        result.version = j.value("version", "unknown");
        result.step = j.value("step", "unknown");
        result.timestamp = j.value("timestamp", "");
        result.source_file = j.value("source_file", "");
        if (j.contains("config")) result.config = j["config"];
        if (j.contains("stats")) result.stats = j["stats"];
        result.data = j["data"];
        return result;
    }
};

// Get current timestamp in ISO 8601 format
inline std::string get_timestamp() {
    auto now = std::chrono::system_clock::now();
    auto time = std::chrono::system_clock::to_time_t(now);
    std::ostringstream oss;
    oss << std::put_time(std::gmtime(&time), "%Y-%m-%dT%H:%M:%SZ");
    return oss.str();
}

// Write JSON to file
inline void write_json_file(const std::string& path, const nlohmann::json& j) {
    std::ofstream file(path);
    if (!file) {
        throw std::runtime_error("Cannot write to file: " + path);
    }
    file << j.dump(2);  // Pretty print with 2-space indent
}

// Read JSON from file
inline nlohmann::json read_json_file(const std::string& path) {
    std::ifstream file(path);
    if (!file) {
        throw std::runtime_error("Cannot open file: " + path);
    }
    nlohmann::json j;
    file >> j;
    return j;
}

// Write SerializedData to file
inline void write_serialized(const std::string& path, const SerializedData& data) {
    write_json_file(path, data.to_json());
}

// Read SerializedData from file
inline SerializedData read_serialized(const std::string& path) {
    return SerializedData::from_json(read_json_file(path));
}

}  // namespace yarnpath::json

#endif // YARNPATH_SERIALIZATION_JSON_SERIALIZATION_HPP
