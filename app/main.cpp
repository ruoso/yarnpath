#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>

#include "geometry.hpp"
#include "surface/surface.hpp"
#include "visualizer/visualizer.hpp"
#include "tokenizer.hpp"
#include "parser.hpp"
#include "emitter.hpp"
#include "logging.hpp"

void print_usage(const char* program_name) {
    std::cerr << "Usage: " << program_name << " [options] <input.txt> [output]\n";
    std::cerr << "\n";
    std::cerr << "Converts a knitting pattern text file to 3D geometry.\n";
    std::cerr << "\n";
    std::cerr << "Options:\n";
    std::cerr << "  --dot             Output yarn path as DOT/Graphviz (skips geometry)\n";
    std::cerr << "  --surface         Output relaxed surface as OBJ (nodes as spheres, edges as lines)\n";
    std::cerr << "  --iterations N    Max solver iterations (default: 100000)\n";
    std::cerr << "  --presolve N      Pre-solve constraint iterations (default: 1000)\n";
    std::cerr << "  --damping N       Solver damping factor 0-1 (default: 0.9)\n";
    std::cerr << "  --threshold N     Convergence threshold (default: 1e-6)\n";
    std::cerr << "  --visualize       Open OpenGL window to watch relaxation (requires GLFW)\n";
    std::cerr << "  --help            Show this help message\n";
    std::cerr << "\n";
    std::cerr << "Arguments:\n";
    std::cerr << "  input.txt   - Input pattern file (Shirley Paden notation)\n";
    std::cerr << "  output      - Output file (.obj or .dot, defaults to stdout)\n";
    std::cerr << "\n";
    std::cerr << "Environment:\n";
    std::cerr << "  YARNPATH_LOG_LEVEL - Set log level (trace, debug, info, warn, error)\n";
}

std::string read_file(const std::string& path) {
    std::ifstream file(path);
    if (!file) {
        throw std::runtime_error("Cannot open file: " + path);
    }
    std::stringstream buffer;
    buffer << file.rdbuf();
    return buffer.str();
}

void write_file(const std::string& path, const std::string& content) {
    std::ofstream file(path);
    if (!file) {
        throw std::runtime_error("Cannot write to file: " + path);
    }
    file << content;
}

int main(int argc, char* argv[]) {
    auto log = yarnpath::logging::get_logger();

    // 1. Parse command-line arguments
    bool output_dot = false;
    bool output_surface = false;
    bool visualize = false;
    std::string input_path;
    std::string output_path;
    bool write_to_stdout = true;
    int max_iterations = 100000;
    int pre_solve_iterations = 1000;
    float damping = 0.9f;
    float threshold = 1e-6f;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--dot") {
            output_dot = true;
        } else if (arg == "--surface") {
            output_surface = true;
        } else if (arg == "--visualize") {
            visualize = true;
        } else if (arg == "--iterations" && i + 1 < argc) {
            max_iterations = std::stoi(argv[++i]);
        } else if (arg == "--presolve" && i + 1 < argc) {
            pre_solve_iterations = std::stoi(argv[++i]);
        } else if (arg == "--damping" && i + 1 < argc) {
            damping = std::stof(argv[++i]);
        } else if (arg == "--threshold" && i + 1 < argc) {
            threshold = std::stof(argv[++i]);
        } else if (arg == "--help" || arg == "-h") {
            print_usage(argv[0]);
            return 0;
        } else if (input_path.empty()) {
            input_path = arg;
        } else if (output_path.empty()) {
            output_path = arg;
            write_to_stdout = false;
        }
    }

    if (input_path.empty()) {
        print_usage(argv[0]);
        return 1;
    }

    log->info("Starting yarnpath pipeline");
    log->info("Input file: {}", input_path);
    if (output_dot) {
        log->info("Output format: DOT (yarn path visualization)");
    } else if (output_surface) {
        log->info("Output format: OBJ (surface relaxation visualization)");
    }
    if (!write_to_stdout) {
        log->info("Output file: {}", output_path);
    }

    try {
        // 2. Read input file
        log->debug("Stage 1: Reading input file");
        std::string input = read_file(input_path);
        log->debug("Read {} bytes from input file", input.size());

        // 3. Tokenize
        log->debug("Stage 2: Tokenizing input");
        yarnpath::parser::Tokenizer tokenizer(input);

        // 4. Parse → PatternAST
        log->debug("Stage 3: Parsing tokens to AST");
        yarnpath::parser::Parser parser(tokenizer);
        yarnpath::parser::ast::PatternAST ast = parser.parse();

        if (parser.has_errors()) {
            for (const auto& error : parser.errors()) {
                log->error("Parse error: {}", error);
                std::cerr << "Parse error: " << error << "\n";
            }
            return 1;
        }
        log->debug("Parsed {} AST nodes", ast.nodes.size());

        // 5. Emit → PatternInstructions
        log->debug("Stage 4: Emitting instructions from AST");
        yarnpath::parser::Emitter emitter;
        yarnpath::PatternInstructions instructions = emitter.emit(ast);

        // Print emitter warnings to stderr
        for (const auto& warning : emitter.warnings()) {
            log->warn("Emitter warning: {}", warning);
            std::cerr << "Warning: " << warning << "\n";
        }

        if (emitter.has_errors()) {
            for (const auto& error : emitter.errors()) {
                log->error("Emitter error: {}", error);
                std::cerr << "Emitter error: " << error << "\n";
            }
            return 1;
        }
        log->debug("Emitted {} row instructions", instructions.rows.size());

        // 6. Build StitchGraph
        log->debug("Stage 5: Building stitch graph");
        yarnpath::StitchGraph graph = yarnpath::StitchGraph::from_instructions(instructions);
        log->debug("Built stitch graph with {} nodes, {} rows",
                   graph.size(), graph.row_count());

        // 7. Build YarnPath
        log->debug("Stage 6: Building yarn path");
        yarnpath::YarnPath yarn_path = yarnpath::YarnPath::from_stitch_graph(graph);
        log->debug("Built yarn path with {} segments", yarn_path.segments().size());

        // If --dot, output DOT format and exit early
        if (output_dot) {
            log->debug("Exporting DOT format");
            std::string dot = yarn_path.to_dot();

            if (write_to_stdout) {
                std::cout << dot;
            } else {
                write_file(output_path, dot);
                std::cerr << "Wrote " << output_path << " ("
                          << yarn_path.segments().size() << " segments)\n";
            }

            log->info("DOT export complete: {} segments", yarn_path.segments().size());
            return 0;
        }

        // If --surface, output surface relaxation OBJ and exit early
        if (output_surface) {
            log->debug("Building surface graph");
            yarnpath::YarnProperties yarn = yarnpath::YarnProperties::thin();
            yarnpath::Gauge gauge = yarnpath::Gauge::bulky();

            // Build and solve surface
            yarnpath::SurfaceBuildConfig build_config;
            build_config.random_seed = 42;

            yarnpath::SurfaceGraph surface_graph =
                yarnpath::SurfaceBuilder::from_yarn_path(yarn_path, yarn, gauge, build_config);

            log->debug("Solving surface relaxation");
            yarnpath::SolveConfig solve_config;
            solve_config.max_iterations = max_iterations;
            solve_config.pre_solve_iterations = pre_solve_iterations;
            solve_config.convergence_threshold = threshold;
            solve_config.force_config.damping = damping;

            yarnpath::SolveResult result =
                yarnpath::SurfaceSolver::solve(surface_graph, yarn, solve_config);

            log->info("Surface relaxation: {} after {} iterations",
                      result.converged ? "converged" : "did not converge",
                      result.iterations);

            // Export to OBJ
            log->debug("Exporting surface OBJ");
            yarnpath::SurfaceObjConfig obj_config;
            obj_config.node_radius = 0.3f;
            obj_config.sphere_subdivisions = 1;

            std::string obj = yarnpath::surface_to_obj(surface_graph, obj_config);

            if (write_to_stdout) {
                std::cout << obj;
            } else {
                write_file(output_path, obj);
                std::cerr << "Wrote " << output_path << " ("
                          << surface_graph.node_count() << " nodes, "
                          << surface_graph.edge_count() << " edges)\n";
            }

            log->info("Surface OBJ export complete: {} nodes, {} edges",
                      surface_graph.node_count(), surface_graph.edge_count());
            return 0;
        }

        // If --visualize, open OpenGL window and exit early
        if (visualize) {
            if (!yarnpath::visualization_available()) {
                log->error("Visualization not available - recompile with GLFW and OpenGL");
                std::cerr << "Error: Visualization not available\n";
                return 1;
            }

            log->debug("Building surface graph for visualization");
            yarnpath::YarnProperties yarn = yarnpath::YarnProperties::fingering();
            yarnpath::Gauge gauge = yarnpath::Gauge::fingering();

            yarnpath::SurfaceBuildConfig build_config;
            build_config.random_seed = 42;

            yarnpath::SurfaceGraph surface_graph =
                yarnpath::SurfaceBuilder::from_yarn_path(yarn_path, yarn, gauge, build_config);

            yarnpath::SolveConfig solve_config;
            solve_config.max_iterations = max_iterations;
            solve_config.pre_solve_iterations = pre_solve_iterations;
            solve_config.convergence_threshold = threshold;
            solve_config.force_config.damping = damping;
            solve_config.force_config.enable_gravity = false;
            solve_config.force_config.enable_floor = false;

            yarnpath::VisualizerConfig viz_config;
            viz_config.window_title = "YarnPath - Surface & Geometry";
            viz_config.steps_per_frame = 10;
            viz_config.auto_run = true;
            viz_config.show_geometry = true;

            log->info("Starting visualization (surface + geometry)...");
            log->info("Controls: n=toggle nodes, g=toggle geometry, space=pause, q=quit");
            yarnpath::VisualizerResult result =
                yarnpath::visualize_with_geometry(
                    surface_graph, yarn_path, yarn, gauge, solve_config, viz_config);

            log->info("Visualization complete: {} iterations, final energy = {}",
                      result.total_iterations, result.final_energy);
            return 0;
        }

        // 8. Build GeometryPath with defaults
        log->debug("Stage 7: Building geometry path");
        yarnpath::YarnProperties yarn = yarnpath::YarnProperties::worsted();
        yarnpath::Gauge gauge = yarnpath::Gauge::worsted();

        // Build and solve surface for geometry
        log->debug("Building surface graph for geometry");
        yarnpath::SurfaceBuildConfig build_config;
        build_config.random_seed = 42;

        yarnpath::SurfaceGraph surface_graph =
            yarnpath::SurfaceBuilder::from_yarn_path(yarn_path, yarn, gauge, build_config);

        log->debug("Solving surface relaxation for geometry");
        yarnpath::SolveConfig solve_config;
        solve_config.max_iterations = max_iterations;
        solve_config.pre_solve_iterations = pre_solve_iterations;
        solve_config.convergence_threshold = threshold;
        solve_config.force_config.damping = damping;

        yarnpath::SolveResult solve_result =
            yarnpath::SurfaceSolver::solve(surface_graph, yarn, solve_config);

        log->info("Surface relaxation: {} after {} iterations",
                  solve_result.converged ? "converged" : "did not converge",
                  solve_result.iterations);

        yarnpath::GeometryPath geometry = yarnpath::GeometryPath::from_yarn_path(
            yarn_path, surface_graph, yarn, gauge);
        log->debug("Built geometry with {} segment geometries",
                   geometry.segments().size());

        // 9. Export OBJ
        log->debug("Stage 8: Exporting OBJ");
        std::string obj = geometry.to_obj();
        log->debug("Generated OBJ output: {} bytes", obj.size());

        // 10. Write output
        if (write_to_stdout) {
            std::cout << obj;
        } else {
            write_file(output_path, obj);
            std::cerr << "Wrote " << output_path << " ("
                      << geometry.segments().size() << " segments)\n";
        }

        log->info("Pipeline complete: {} segments", geometry.segments().size());

        return 0;

    } catch (const std::exception& e) {
        log->error("Error: {}", e.what());
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }
}
