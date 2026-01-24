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
    std::cerr << "Usage: " << program_name << " <input.txt> [output.obj]\n";
    std::cerr << "\n";
    std::cerr << "Converts a knitting pattern text file to a 3D OBJ file.\n";
    std::cerr << "\n";
    std::cerr << "Arguments:\n";
    std::cerr << "  input.txt   - Input pattern file (Shirley Paden notation)\n";
    std::cerr << "  output.obj  - Output OBJ file (optional, defaults to stdout)\n";
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
    if (argc < 2) {
        print_usage(argv[0]);
        return 1;
    }

    std::string input_path = argv[1];
    std::string output_path;
    bool write_to_stdout = true;

    if (argc >= 3) {
        output_path = argv[2];
        write_to_stdout = false;
    }

    log->info("Starting yarnpath pipeline");
    log->info("Input file: {}", input_path);
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

        // 8. Build GeometryPath with defaults
        log->debug("Stage 7: Building geometry path");
        yarnpath::YarnProperties yarn = yarnpath::YarnProperties::worsted();
        yarnpath::Gauge gauge = yarnpath::Gauge::worsted();
        yarnpath::PlaneSurface surface;

        yarnpath::GeometryPath geometry = yarnpath::GeometryPath::from_yarn_path(
            yarn_path, yarn, gauge, surface);
        log->debug("Built geometry with {} positioned loops, {} anchor geometries, {} segment geometries",
                   geometry.loop_positions().size(),
                   geometry.anchors().size(),
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
                      << geometry.anchors().size() << " anchors)\n";
        }

        log->info("Pipeline complete: {} segments, {} anchors, {} loops",
                  geometry.segments().size(),
                  geometry.anchors().size(),
                  geometry.loop_positions().size());

        return 0;

    } catch (const std::exception& e) {
        log->error("Error: {}", e.what());
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }
}
