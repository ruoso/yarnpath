#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include "geometry.hpp"
#include "tokenizer.hpp"
#include "parser.hpp"
#include "emitter.hpp"

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

    try {
        // 2. Read input file
        std::string input = read_file(input_path);

        // 3. Tokenize
        yarnpath::parser::Tokenizer tokenizer(input);

        // 4. Parse → PatternAST
        yarnpath::parser::Parser parser(tokenizer);
        yarnpath::parser::ast::PatternAST ast = parser.parse();

        if (parser.has_errors()) {
            for (const auto& error : parser.errors()) {
                std::cerr << "Parse error: " << error << "\n";
            }
            return 1;
        }

        // 5. Emit → PatternInstructions
        yarnpath::parser::Emitter emitter;
        yarnpath::PatternInstructions instructions = emitter.emit(ast);

        // Print emitter warnings to stderr
        for (const auto& warning : emitter.warnings()) {
            std::cerr << "Warning: " << warning << "\n";
        }

        if (emitter.has_errors()) {
            for (const auto& error : emitter.errors()) {
                std::cerr << "Emitter error: " << error << "\n";
            }
            return 1;
        }

        // 6. Build StitchGraph
        yarnpath::StitchGraph graph = yarnpath::StitchGraph::from_instructions(instructions);

        // 7. Build YarnPath
        yarnpath::YarnPath yarn_path = yarnpath::YarnPath::from_stitch_graph(graph);

        // 8. Build GeometryPath with defaults
        yarnpath::YarnProperties yarn = yarnpath::YarnProperties::worsted();
        yarnpath::Gauge gauge = yarnpath::Gauge::worsted();
        yarnpath::PlaneSurface surface;

        yarnpath::GeometryPath geometry = yarnpath::GeometryPath::from_yarn_path(
            yarn_path, yarn, gauge, surface);

        // 9. Export OBJ
        std::string obj = geometry.to_obj();

        // 10. Write output
        if (write_to_stdout) {
            std::cout << obj;
        } else {
            write_file(output_path, obj);
            std::cerr << "Wrote " << output_path << " ("
                      << geometry.segments().size() << " segments, "
                      << geometry.anchors().size() << " anchors)\n";
        }

        return 0;

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }
}
