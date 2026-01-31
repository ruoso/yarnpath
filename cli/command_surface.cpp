#include "cli_common.hpp"
#include <yarn_path/yarn_path.hpp>
#include <surface/surface_builder.hpp>
#include <surface/surface_solver.hpp>
#include <serialization/json_serialization.hpp>
#include <serialization/config_json.hpp>
#include <serialization/yarn_path_json.hpp>
#include <serialization/surface_graph_json.hpp>
#include <common/logging.hpp>

namespace yarnpath::cli {

int command_surface(int argc, char** argv) {
    auto log = yarnpath::logging::get_logger();

    try {
        auto [ctx, _] = parse_common_args(argc, argv, 2);

        if (ctx.input_path.empty() || ctx.output_path.empty()) {
            std::cerr << "Usage: yarnpath surface <yarn.json> -o <surface.json> [-c config.json]\n";
            return 1;
        }

        log->info("Building and solving surface from: {}", ctx.input_path);

        // Load yarn path
        json::SerializedData input_data = json::read_serialized(ctx.input_path);
        YarnPath yarn_path = yarn_path_from_json(input_data.data);

        // Load or use default configuration
        YarnProperties yarn = YarnProperties::worsted();
        Gauge gauge = Gauge::worsted();
        SurfaceBuildConfig build_config;
        SolveConfig solve_config;

        if (ctx.config_path.has_value()) {
            nlohmann::json config_json = json::read_json_file(ctx.config_path.value());
            if (config_json.contains("yarn")) {
                yarn = config_json["yarn"].get<YarnProperties>();
            }
            if (config_json.contains("gauge")) {
                gauge = config_json["gauge"].get<Gauge>();
            }
            if (config_json.contains("build")) {
                build_config = config_json["build"].get<SurfaceBuildConfig>();
            }
            if (config_json.contains("solve")) {
                solve_config = config_json["solve"].get<SolveConfig>();
            }
        }

        // Build surface graph
        log->debug("Building surface graph");
        SurfaceGraph surface_graph = SurfaceBuilder::from_yarn_path(
            yarn_path, yarn, gauge, build_config);

        // Solve surface relaxation
        log->debug("Solving surface relaxation");
        SolveResult result = SurfaceSolver::solve(surface_graph, yarn, solve_config);

        log->info("Surface relaxation: {} after {} iterations",
                  result.converged ? "converged" : "did not converge",
                  result.iterations);

        // Serialize to JSON
        json::SerializedData data;
        data.step = "surface_graph";
        data.timestamp = json::get_timestamp();
        data.source_file = input_data.source_file;

        // Store configuration used and yarn path for downstream use
        data.config = {
            {"yarn", yarn},
            {"gauge", gauge},
            {"build", build_config},
            {"solve", solve_config},
            {"result", result},
            {"yarn_path", yarn_path_to_json(yarn_path)}
        };

        data.data = surface_graph_to_json(surface_graph);
        data.stats = {
            {"node_count", surface_graph.node_count()},
            {"edge_count", surface_graph.edge_count()},
            {"constraint_count", surface_graph.constraint_count()},
            {"converged", result.converged},
            {"iterations", result.iterations},
            {"final_energy", result.final_energy}
        };

        json::write_serialized(ctx.output_path, data);

        log->info("Wrote surface graph to {}", ctx.output_path);
        std::cerr << "Wrote " << ctx.output_path << " ("
                  << surface_graph.node_count() << " nodes, "
                  << surface_graph.edge_count() << " edges, "
                  << (result.converged ? "converged" : "did not converge") << " after "
                  << result.iterations << " iterations)\n";

        return 0;

    } catch (const std::exception& e) {
        log->error("Error: {}", e.what());
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }
}

}  // namespace yarnpath::cli
