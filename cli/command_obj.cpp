#include "cli_common.hpp"
#include <geometry/geometry_path.hpp>
#include <serialization/json_serialization.hpp>
#include <serialization/geometry_path_json.hpp>
#include <common/logging.hpp>

namespace yarnpath::cli {

int command_obj(int argc, char** argv) {
    auto log = yarnpath::logging::get_logger();

    try {
        auto [ctx, _] = parse_common_args(argc, argv, 2);

        if (ctx.input_path.empty() || ctx.output_path.empty()) {
            std::cerr << "Usage: yarnpath obj <geometry.json> -o <output.obj>\n";
            return 1;
        }

        log->info("Exporting OBJ from: {}", ctx.input_path);

        // Load geometry
        json::SerializedData input_data = json::read_serialized(ctx.input_path);
        GeometryPath geometry = geometry_path_from_json(input_data.data);

        // Export to OBJ
        log->debug("Generating OBJ");
        std::string obj = geometry.to_obj();

        // Write to file
        write_file(ctx.output_path, obj);

        log->info("Wrote OBJ to {}", ctx.output_path);
        std::cerr << "Wrote " << ctx.output_path << " ("
                  << geometry.segments().size() << " segments, "
                  << obj.size() << " bytes)\n";

        return 0;

    } catch (const std::exception& e) {
        log->error("Error: {}", e.what());
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }
}

}  // namespace yarnpath::cli
