#ifndef YARNPATH_SERIALIZATION_INSTRUCTION_JSON_HPP
#define YARNPATH_SERIALIZATION_INSTRUCTION_JSON_HPP

#include <nlohmann/json.hpp>
#include <instructions/row_instruction.hpp>
#include <instructions/stitch_instruction.hpp>

namespace yarnpath {

// RowSide enum serialization
NLOHMANN_JSON_SERIALIZE_ENUM(RowSide, {
    {RowSide::RS, "RS"},
    {RowSide::WS, "WS"},
})

// Forward declarations for mutual recursion
nlohmann::json stitch_instruction_to_json(const StitchInstruction& instr);
StitchInstruction stitch_instruction_from_json(const nlohmann::json& j);

// StitchInstruction serialization
inline nlohmann::json stitch_instruction_to_json(const StitchInstruction& instr) {
    return std::visit([](const auto& op) -> nlohmann::json {
        using T = std::decay_t<decltype(op)>;

        nlohmann::json j;

        if constexpr (std::is_same_v<T, instruction::Knit>) {
            j["type"] = "Knit";
        } else if constexpr (std::is_same_v<T, instruction::Purl>) {
            j["type"] = "Purl";
        } else if constexpr (std::is_same_v<T, instruction::Slip>) {
            j["type"] = "Slip";
        } else if constexpr (std::is_same_v<T, instruction::CastOn>) {
            j["type"] = "CastOn";
            j["count"] = op.count;
        } else if constexpr (std::is_same_v<T, instruction::BindOff>) {
            j["type"] = "BindOff";
            j["count"] = op.count;
        } else if constexpr (std::is_same_v<T, instruction::YarnOver>) {
            j["type"] = "YarnOver";
        } else if constexpr (std::is_same_v<T, instruction::KFB>) {
            j["type"] = "KFB";
        } else if constexpr (std::is_same_v<T, instruction::M1L>) {
            j["type"] = "M1L";
        } else if constexpr (std::is_same_v<T, instruction::M1R>) {
            j["type"] = "M1R";
        } else if constexpr (std::is_same_v<T, instruction::K2tog>) {
            j["type"] = "K2tog";
        } else if constexpr (std::is_same_v<T, instruction::SSK>) {
            j["type"] = "SSK";
        } else if constexpr (std::is_same_v<T, instruction::S2KP>) {
            j["type"] = "S2KP";
        } else if constexpr (std::is_same_v<T, instruction::CableLeft>) {
            j["type"] = "CableLeft";
            j["hold"] = op.hold;
            j["cross"] = op.cross;
        } else if constexpr (std::is_same_v<T, instruction::CableRight>) {
            j["type"] = "CableRight";
            j["hold"] = op.hold;
            j["cross"] = op.cross;
        } else if constexpr (std::is_same_v<T, instruction::Repeat>) {
            j["type"] = "Repeat";
            j["times"] = op.times;
            j["instructions"] = nlohmann::json::array();
            for (const auto& sub_instr : op.instructions) {
                j["instructions"].push_back(stitch_instruction_to_json(sub_instr));
            }
        }

        return j;
    }, instr);
}

inline StitchInstruction stitch_instruction_from_json(const nlohmann::json& j) {
    std::string type = j["type"].get<std::string>();

    if (type == "Knit") {
        return instruction::Knit{};
    } else if (type == "Purl") {
        return instruction::Purl{};
    } else if (type == "Slip") {
        return instruction::Slip{};
    } else if (type == "CastOn") {
        return instruction::CastOn{j["count"].get<uint32_t>()};
    } else if (type == "BindOff") {
        return instruction::BindOff{j["count"].get<uint32_t>()};
    } else if (type == "YarnOver") {
        return instruction::YarnOver{};
    } else if (type == "KFB") {
        return instruction::KFB{};
    } else if (type == "M1L") {
        return instruction::M1L{};
    } else if (type == "M1R") {
        return instruction::M1R{};
    } else if (type == "K2tog") {
        return instruction::K2tog{};
    } else if (type == "SSK") {
        return instruction::SSK{};
    } else if (type == "S2KP") {
        return instruction::S2KP{};
    } else if (type == "CableLeft") {
        return instruction::CableLeft{
            j["hold"].get<uint8_t>(),
            j["cross"].get<uint8_t>()
        };
    } else if (type == "CableRight") {
        return instruction::CableRight{
            j["hold"].get<uint8_t>(),
            j["cross"].get<uint8_t>()
        };
    } else if (type == "Repeat") {
        instruction::Repeat repeat;
        repeat.times = j["times"].get<uint32_t>();
        for (const auto& sub_j : j["instructions"]) {
            repeat.instructions.push_back(stitch_instruction_from_json(sub_j));
        }
        return repeat;
    } else {
        throw std::runtime_error("Unknown stitch instruction type: " + type);
    }
}

// RowInstruction serialization
inline void to_json(nlohmann::json& j, const RowInstruction& row) {
    j["side"] = row.side;
    j["stitches"] = nlohmann::json::array();
    for (const auto& stitch : row.stitches) {
        j["stitches"].push_back(stitch_instruction_to_json(stitch));
    }
}

inline void from_json(const nlohmann::json& j, RowInstruction& row) {
    row.side = j["side"].get<RowSide>();
    row.stitches.clear();
    for (const auto& stitch_j : j["stitches"]) {
        row.stitches.push_back(stitch_instruction_from_json(stitch_j));
    }
}

// PatternInstructions serialization
inline void to_json(nlohmann::json& j, const PatternInstructions& pattern) {
    j["rows"] = pattern.rows;
    if (pattern.panel_id.has_value()) {
        j["panel_id"] = pattern.panel_id.value();
    } else {
        j["panel_id"] = nullptr;
    }
}

inline void from_json(const nlohmann::json& j, PatternInstructions& pattern) {
    pattern.rows = j["rows"].get<std::vector<RowInstruction>>();
    if (j["panel_id"].is_null()) {
        pattern.panel_id = std::nullopt;
    } else {
        pattern.panel_id = j["panel_id"].get<uint32_t>();
    }
}

}  // namespace yarnpath

#endif // YARNPATH_SERIALIZATION_INSTRUCTION_JSON_HPP
