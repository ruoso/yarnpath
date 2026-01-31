#include "cli_common.hpp"
#include <parser/tokenizer.hpp>
#include <parser/parser.hpp>
#include <parser/emitter.hpp>
#include <serialization/json_serialization.hpp>
#include <serialization/instruction_json.hpp>
#include <common/logging.hpp>

namespace yarnpath::cli {

int command_parse(int argc, char** argv) {
    auto log = yarnpath::logging::get_logger();

    try {
        auto [ctx, _] = parse_common_args(argc, argv, 2);

        if (ctx.input_path.empty() || ctx.output_path.empty()) {
            std::cerr << "Usage: yarnpath parse <input.txt> -o <output.instructions.json>\n";
            return 1;
        }

        log->info("Parsing pattern: {}", ctx.input_path);

        // Read input file
        std::string input = read_file(ctx.input_path);

        // Tokenize
        yarnpath::parser::Tokenizer tokenizer(input);

        // Parse to AST
        yarnpath::parser::Parser parser(tokenizer);
        yarnpath::parser::ast::PatternAST ast = parser.parse();

        if (parser.has_errors()) {
            for (const auto& error : parser.errors()) {
                log->error("Parse error: {}", error);
            }
            return 1;
        }

        // Emit instructions
        yarnpath::parser::Emitter emitter;
        yarnpath::PatternInstructions instructions = emitter.emit(ast);

        for (const auto& warning : emitter.warnings()) {
            log->warn("Warning: {}", warning);
        }

        if (emitter.has_errors()) {
            for (const auto& error : emitter.errors()) {
                log->error("Emitter error: {}", error);
            }
            return 1;
        }

        // Serialize to JSON
        json::SerializedData data;
        data.step = "instructions";
        data.timestamp = json::get_timestamp();
        data.source_file = ctx.input_path;
        data.data = nlohmann::json(instructions);
        data.stats = {
            {"row_count", instructions.rows.size()},
            {"total_stitches", std::accumulate(instructions.rows.begin(), instructions.rows.end(), 0,
                [](int sum, const RowInstruction& row) { return sum + row.stitches.size(); })}
        };

        json::write_serialized(ctx.output_path, data);

        log->info("Wrote instructions to {}", ctx.output_path);
        std::cerr << "Wrote " << ctx.output_path << " ("
                  << instructions.rows.size() << " rows)\n";

        return 0;

    } catch (const std::exception& e) {
        log->error("Error: {}", e.what());
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }
}

}  // namespace yarnpath::cli
