#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include "geometry.hpp"
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
    std::cerr << "  --dot       Output yarn path as DOT/Graphviz (skips geometry)\n";
    std::cerr << "  --help      Show this help message\n";
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
    std::string input_path;
    std::string output_path;
    bool write_to_stdout = true;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--dot") {
            output_dot = true;
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
        log->debug("Built yarn path with {} loops, {} anchors, {} segments",
                   yarn_path.loops().size(),
                   yarn_path.anchors().size(),
                   yarn_path.segments().size());

        // If --dot, output DOT format and exit early
        if (output_dot) {
            log->debug("Exporting DOT format");
            std::string dot = yarn_path.to_dot();

            if (write_to_stdout) {
                std::cout << dot;
            } else {
                write_file(output_path, dot);
                std::cerr << "Wrote " << output_path << " ("
                          << yarn_path.loops().size() << " loops)\n";
            }

            log->info("DOT export complete: {} loops, {} anchors, {} segments",
                      yarn_path.loops().size(),
                      yarn_path.anchors().size(),
                      yarn_path.segments().size());
            return 0;
        }

        // 8. Build GeometryPath with defaults
        log->debug("Stage 7: Building geometry path");
        yarnpath::YarnProperties yarn = yarnpath::YarnProperties::worsted();
        yarnpath::Gauge gauge = yarnpath::Gauge::worsted();
        yarnpath::PlaneSurface surface;

        yarnpath::GeometryPath geometry = yarnpath::GeometryPath::from_yarn_path(
            yarn_path, yarn, gauge, surface);
        log->debug("Built geometry with {} positioned loops, {} segment geometries",
                   geometry.loop_positions().size(),
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
                      << geometry.segments().size() << " segments, "
                      << geometry.loop_positions().size() << " loops)\n";
        }

        log->info("Pipeline complete: {} segments, {} loops",
                  geometry.segments().size(),
                  geometry.loop_positions().size());

        return 0;

    } catch (const std::exception& e) {
        log->error("Error: {}", e.what());
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }
}
