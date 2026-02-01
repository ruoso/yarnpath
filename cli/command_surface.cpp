#include "cli_common.hpp"
#include <yarn_path/yarn_path.hpp>
#include <surface/surface_builder.hpp>
#include <surface/surface_solver.hpp>
#include <visualizer/visualizer.hpp>
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

        if (ctx.input_path.empty()) {
            std::cerr << "Usage: yarnpath surface <yarn.json> -o <surface.json> [options]\n";
            std::cerr << "Options:\n";
            std::cerr << "  -c, --config FILE    Override config or set surface-specific options\n";
            std::cerr << "                       (yarn/gauge read from input file by default)\n";
            std::cerr << "  --iterations N       Max solver iterations (default: 100000)\n";
            std::cerr << "  --threshold N        Convergence threshold (default: 1e-6)\n";
            std::cerr << "  --visualize          Open interactive visualization window\n";
            return 1;
        }

        if (!ctx.visualize && ctx.output_path.empty()) {
            std::cerr << "Usage: yarnpath surface <yarn.json> -o <surface.json> [options]\n";
            std::cerr << "Error: -o <output> is required (unless --visualize is used)\n";
            return 1;
        }

        log->info("Building and solving surface from: {}", ctx.input_path);

        // Load yarn path
        json::SerializedData input_data = json::read_serialized(ctx.input_path);
        YarnPath yarn_path = yarn_path_from_json(input_data.data);

        // Load configuration from yarn output first (if available), then apply overrides
        YarnProperties yarn = YarnProperties::thin();  // Default fallback
        Gauge gauge = Gauge::worsted();                 // Default fallback
        SurfaceBuildConfig build_config;
        SolveConfig solve_config;
        solve_config.max_iterations = 100000;
        solve_config.convergence_threshold = 1e-6f;

        // Read yarn/gauge from yarn output config (if present)
        if (input_data.config.contains("yarn")) {
            yarn = input_data.config["yarn"].get<YarnProperties>();
            log->info("Using yarn properties from input file");
        }
        if (input_data.config.contains("gauge")) {
            gauge = input_data.config["gauge"].get<Gauge>();
            log->info("Using gauge from input file");
        }

        // Override with explicit config file (if provided)
        if (ctx.config_path.has_value()) {
            nlohmann::json config_json = json::read_json_file(ctx.config_path.value());
            if (config_json.contains("yarn")) {
                yarn = config_json["yarn"].get<YarnProperties>();
                log->info("Overriding yarn properties from config file");
            }
            if (config_json.contains("gauge")) {
                gauge = config_json["gauge"].get<Gauge>();
                log->info("Overriding gauge from config file");
            }
            if (config_json.contains("build")) {
                build_config = config_json["build"].get<SurfaceBuildConfig>();
            }
            if (config_json.contains("solve")) {
                solve_config = config_json["solve"].get<SolveConfig>();
            }
        }

        // Override with command-line arguments
        if (ctx.max_iterations.has_value()) {
            solve_config.max_iterations = ctx.max_iterations.value();
        }
        if (ctx.threshold.has_value()) {
            solve_config.convergence_threshold = ctx.threshold.value();
        }

        // Build surface graph
        log->debug("Building surface graph");
        SurfaceGraph surface_graph = SurfaceBuilder::from_yarn_path(
            yarn_path, yarn, gauge, build_config);

        SolveResult result;

        // Solve/visualize surface relaxation
        if (ctx.visualize) {
            if (!visualization_available()) {
                log->error("Visualization not available - recompile with GLFW and OpenGL");
                std::cerr << "Error: Visualization not available\n";
                return 1;
            }

            log->info("Starting visualization (surface relaxation)...");
            log->info("Controls: space=pause, r=reset, q=quit");

            VisualizerConfig viz_config;
            viz_config.window_title = "YarnPath - Surface Relaxation";
            viz_config.steps_per_frame = 10;
            viz_config.auto_run = true;
            viz_config.show_geometry = false;

            VisualizerResult viz_result = visualize_relaxation(
                surface_graph, yarn, gauge, solve_config, viz_config);

            log->info("Visualization complete: {} iterations, final energy = {}",
                      viz_result.total_iterations, viz_result.final_energy);

            // Create solve result from visualization
            result.converged = viz_result.completed;
            result.iterations = viz_result.total_iterations;
            result.final_energy = viz_result.final_energy;
        } else {
            // Solve surface relaxation (non-interactive)
            log->debug("Solving surface relaxation");
            result = SurfaceSolver::solve(surface_graph, yarn, gauge, solve_config);

            log->info("Surface relaxation: {} after {} iterations",
                      result.converged ? "converged" : "did not converge",
                      result.iterations);
        }

        // Skip saving if no output path provided (visualization-only mode)
        if (ctx.output_path.empty()) {
            return 0;
        }

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
