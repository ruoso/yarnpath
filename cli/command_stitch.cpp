#include "cli_common.hpp"
#include <stitch_graph/stitch_node.hpp>
#include <serialization/json_serialization.hpp>
#include <serialization/instruction_json.hpp>
#include <serialization/stitch_graph_json.hpp>
#include <common/logging.hpp>

namespace yarnpath::cli {

int command_stitch(int argc, char** argv) {
    auto log = yarnpath::logging::get_logger();

    try {
        auto [ctx, _] = parse_common_args(argc, argv, 2);

        if (ctx.input_path.empty() || ctx.output_path.empty()) {
            std::cerr << "Usage: yarnpath stitch <instructions.json> -o <graph.json>\n";
            return 1;
        }

        log->info("Building stitch graph from: {}", ctx.input_path);

        // Load instructions
        json::SerializedData input_data = json::read_serialized(ctx.input_path);
        PatternInstructions instructions = input_data.data.get<PatternInstructions>();

        // Build stitch graph
        StitchGraph graph = StitchGraph::from_instructions(instructions);

        // Serialize to JSON
        json::SerializedData data;
        data.step = "stitch_graph";
        data.timestamp = json::get_timestamp();
        data.source_file = input_data.source_file;
        data.data = stitch_graph_to_json(graph);
        data.stats = {
            {"node_count", graph.size()},
            {"row_count", graph.row_count()}
        };

        json::write_serialized(ctx.output_path, data);

        log->info("Wrote stitch graph to {}", ctx.output_path);
        std::cerr << "Wrote " << ctx.output_path << " ("
                  << graph.size() << " nodes, "
                  << graph.row_count() << " rows)\n";

        return 0;

    } catch (const std::exception& e) {
        log->error("Error: {}", e.what());
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }
}

}  // namespace yarnpath::cli
