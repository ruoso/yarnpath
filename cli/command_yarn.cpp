#include "cli_common.hpp"
#include <yarn_path/yarn_path.hpp>
#include <serialization/json_serialization.hpp>
#include <serialization/stitch_graph_json.hpp>
#include <serialization/yarn_path_json.hpp>
#include <common/logging.hpp>
#include <yarn/yarn_properties.hpp>
#include <yarn/gauge.hpp>

namespace yarnpath::cli {

int command_yarn(int argc, char** argv) {
    auto log = yarnpath::logging::get_logger();

    try {
        auto [ctx, _] = parse_common_args(argc, argv, 2);

        if (ctx.input_path.empty() || ctx.output_path.empty()) {
            std::cerr << "Usage: yarnpath yarn <graph.json> -o <yarn.json>\n";
            return 1;
        }

        log->info("Building yarn path from: {}", ctx.input_path);

        // Load stitch graph
        json::SerializedData input_data = json::read_serialized(ctx.input_path);
        StitchGraph graph = stitch_graph_from_json(input_data.data);

        // Create default yarn and gauge (TODO: load from config or command line)
        YarnProperties yarn;
        yarn.relaxed_radius = 0.75f;
        yarn.compressed_radius = 0.5f;
        yarn.stiffness = 0.8f;
        yarn.elasticity = 0.3f;
        yarn.tension = 0.5f;
        Gauge gauge{5.0f};  // US 8 needles

        // Build yarn path
        YarnPath yarn_path = YarnPath::from_stitch_graph(graph, yarn, gauge);

        // Serialize to JSON
        json::SerializedData data;
        data.step = "yarn_path";
        data.timestamp = json::get_timestamp();
        data.source_file = input_data.source_file;
        data.data = yarn_path_to_json(yarn_path);
        data.stats = {
            {"segment_count", yarn_path.segment_count()},
            {"loop_count", std::count_if(yarn_path.segments().begin(), yarn_path.segments().end(),
                [](const YarnSegment& seg) { return seg.forms_loop; })}
        };

        json::write_serialized(ctx.output_path, data);

        log->info("Wrote yarn path to {}", ctx.output_path);
        std::cerr << "Wrote " << ctx.output_path << " ("
                  << yarn_path.segment_count() << " segments)\n";

        return 0;

    } catch (const std::exception& e) {
        log->error("Error: {}", e.what());
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }
}

}  // namespace yarnpath::cli
