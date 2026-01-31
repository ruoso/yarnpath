#ifndef YARNPATH_SERIALIZATION_STITCH_GRAPH_JSON_HPP
#define YARNPATH_SERIALIZATION_STITCH_GRAPH_JSON_HPP

#include <nlohmann/json.hpp>
#include <stitch_graph/stitch_node.hpp>
#include <stitch_graph/stitch_operation.hpp>
#include "instruction_json.hpp"  // For RowSide enum serialization

namespace yarnpath {

// StitchOperation serialization
inline nlohmann::json stitch_operation_to_json(const StitchOperation& op) {
    return std::visit([](const auto& operation) -> nlohmann::json {
        using T = std::decay_t<decltype(operation)>;

        nlohmann::json j;

        if constexpr (std::is_same_v<T, stitch::Knit>) {
            j["type"] = "Knit";
        } else if constexpr (std::is_same_v<T, stitch::Purl>) {
            j["type"] = "Purl";
        } else if constexpr (std::is_same_v<T, stitch::Slip>) {
            j["type"] = "Slip";
        } else if constexpr (std::is_same_v<T, stitch::CastOn>) {
            j["type"] = "CastOn";
        } else if constexpr (std::is_same_v<T, stitch::BindOff>) {
            j["type"] = "BindOff";
        } else if constexpr (std::is_same_v<T, stitch::YarnOver>) {
            j["type"] = "YarnOver";
        } else if constexpr (std::is_same_v<T, stitch::KFB>) {
            j["type"] = "KFB";
        } else if constexpr (std::is_same_v<T, stitch::M1L>) {
            j["type"] = "M1L";
        } else if constexpr (std::is_same_v<T, stitch::M1R>) {
            j["type"] = "M1R";
        } else if constexpr (std::is_same_v<T, stitch::K2tog>) {
            j["type"] = "K2tog";
        } else if constexpr (std::is_same_v<T, stitch::SSK>) {
            j["type"] = "SSK";
        } else if constexpr (std::is_same_v<T, stitch::S2KP>) {
            j["type"] = "S2KP";
        } else if constexpr (std::is_same_v<T, stitch::CableLeft>) {
            j["type"] = "CableLeft";
            j["held"] = operation.held;
            j["crossed"] = operation.crossed;
        } else if constexpr (std::is_same_v<T, stitch::CableRight>) {
            j["type"] = "CableRight";
            j["held"] = operation.held;
            j["crossed"] = operation.crossed;
        }

        return j;
    }, op);
}

inline StitchOperation stitch_operation_from_json(const nlohmann::json& j) {
    std::string type = j["type"].get<std::string>();

    if (type == "Knit") {
        return stitch::Knit{};
    } else if (type == "Purl") {
        return stitch::Purl{};
    } else if (type == "Slip") {
        return stitch::Slip{};
    } else if (type == "CastOn") {
        return stitch::CastOn{};
    } else if (type == "BindOff") {
        return stitch::BindOff{};
    } else if (type == "YarnOver") {
        return stitch::YarnOver{};
    } else if (type == "KFB") {
        return stitch::KFB{};
    } else if (type == "M1L") {
        return stitch::M1L{};
    } else if (type == "M1R") {
        return stitch::M1R{};
    } else if (type == "K2tog") {
        return stitch::K2tog{};
    } else if (type == "SSK") {
        return stitch::SSK{};
    } else if (type == "S2KP") {
        return stitch::S2KP{};
    } else if (type == "CableLeft") {
        return stitch::CableLeft{
            j["held"].get<std::vector<StitchId>>(),
            j["crossed"].get<std::vector<StitchId>>()
        };
    } else if (type == "CableRight") {
        return stitch::CableRight{
            j["held"].get<std::vector<StitchId>>(),
            j["crossed"].get<std::vector<StitchId>>()
        };
    } else {
        throw std::runtime_error("Unknown stitch operation type: " + type);
    }
}

// StitchNode serialization
inline void to_json(nlohmann::json& j, const StitchNode& node) {
    j["id"] = node.id;
    j["operation"] = stitch_operation_to_json(node.operation);
    j["row"] = node.row;
    j["column"] = node.column;
    j["worked_through"] = node.worked_through;
    if (node.panel_id.has_value()) {
        j["panel_id"] = node.panel_id.value();
    } else {
        j["panel_id"] = nullptr;
    }
}

inline void from_json(const nlohmann::json& j, StitchNode& node) {
    node.id = j["id"].get<StitchId>();
    node.operation = stitch_operation_from_json(j["operation"]);
    node.row = j["row"].get<uint32_t>();
    node.column = j["column"].get<uint32_t>();
    node.worked_through = j["worked_through"].get<std::vector<StitchId>>();
    if (j["panel_id"].is_null()) {
        node.panel_id = std::nullopt;
    } else {
        node.panel_id = j["panel_id"].get<uint32_t>();
    }
}

// RowInfo serialization
inline void to_json(nlohmann::json& j, const RowInfo& row) {
    j["row_number"] = row.row_number;
    j["side"] = row.side;
    j["first_stitch_id"] = row.first_stitch_id;
    j["stitch_count"] = row.stitch_count;
}

inline void from_json(const nlohmann::json& j, RowInfo& row) {
    row.row_number = j["row_number"].get<uint32_t>();
    row.side = j["side"].get<RowSide>();
    row.first_stitch_id = j["first_stitch_id"].get<uint32_t>();
    row.stitch_count = j["stitch_count"].get<uint32_t>();
}

// StitchGraph serialization
inline nlohmann::json stitch_graph_to_json(const StitchGraph& graph) {
    nlohmann::json j;
    j["nodes"] = graph.nodes();
    j["rows"] = graph.row_infos();
    return j;
}

// StitchGraph deserialization
inline StitchGraph stitch_graph_from_json(const nlohmann::json& j) {
    auto nodes = j["nodes"].get<std::vector<StitchNode>>();
    auto rows = j["rows"].get<std::vector<RowInfo>>();
    return StitchGraph(std::move(nodes), std::move(rows));
}

}  // namespace yarnpath

#endif // YARNPATH_SERIALIZATION_STITCH_GRAPH_JSON_HPP
