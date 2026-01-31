#include "cli_common.hpp"
#include <geometry/geometry_path.hpp>
#include <serialization/json_serialization.hpp>
#include <serialization/config_json.hpp>
#include <serialization/yarn_path_json.hpp>
#include <serialization/surface_graph_json.hpp>
#include <serialization/geometry_path_json.hpp>
#include <common/logging.hpp>

namespace yarnpath::cli {

int command_geometry(int argc, char** argv) {
    auto log = yarnpath::logging::get_logger();

    try {
        auto [ctx, _] = parse_common_args(argc, argv, 2);

        if (ctx.input_path.empty() || ctx.output_path.empty()) {
            std::cerr << "Usage: yarnpath geometry <surface.json> -o <geometry.json>\n";
            return 1;
        }

        log->info("Building geometry from: {}", ctx.input_path);

        // Load surface graph
        json::SerializedData input_data = json::read_serialized(ctx.input_path);
        SurfaceGraph surface_graph = surface_graph_from_json(input_data.data);

        // Extract configuration and yarn path
        if (input_data.config.is_null() || !input_data.config.contains("yarn_path")) {
            log->error("Surface file missing yarn_path in config");
            log->error("This surface.json was created with an older version");
            return 1;
        }

        YarnPath yarn_path = yarn_path_from_json(input_data.config["yarn_path"]);
        YarnProperties yarn = input_data.config["yarn"].get<YarnProperties>();
        Gauge gauge = input_data.config["gauge"].get<Gauge>();

        // Build geometry path
        log->debug("Building geometry path");
        GeometryPath geometry = GeometryPath::from_yarn_path(
            yarn_path, surface_graph, yarn, gauge);

        // Serialize to JSON
        json::SerializedData data;
        data.step = "geometry_path";
        data.timestamp = json::get_timestamp();
        data.source_file = input_data.source_file;

        // Carry forward configuration
        data.config = {
            {"yarn", yarn},
            {"gauge", gauge}
        };

        data.data = geometry_path_to_json(geometry);
        data.stats = {
            {"segment_count", geometry.segments().size()},
            {"total_arc_length", geometry.total_arc_length()}
        };

        json::write_serialized(ctx.output_path, data);

        log->info("Wrote geometry to {}", ctx.output_path);
        std::cerr << "Wrote " << ctx.output_path << " ("
                  << geometry.segments().size() << " segments, "
                  << "arc length: " << geometry.total_arc_length() << ")\n";

        return 0;

    } catch (const std::exception& e) {
        log->error("Error: {}", e.what());
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }
}

}  // namespace yarnpath::cli
