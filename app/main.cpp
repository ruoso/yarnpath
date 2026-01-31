#include <iostream>
#include <string>
#include <cli/cli_common.hpp>
#include <common/logging.hpp>

void print_usage(const char* program_name) {
    std::cerr << "Usage: " << program_name << " <command> <input> -o <output> [options]\n";
    std::cerr << "\n";
    std::cerr << "YarnPath - Convert knitting patterns to 3D geometry\n";
    std::cerr << "\n";
    std::cerr << "Commands:\n";
    std::cerr << "  generate-makefile  Create Makefile for pipeline\n";
    std::cerr << "  parse              Convert pattern text to instructions JSON\n";
    std::cerr << "  stitch             Build stitch graph from instructions JSON\n";
    std::cerr << "  yarn               Build yarn path topology from stitch graph JSON\n";
    std::cerr << "  surface            Build and relax surface from yarn path JSON\n";
    std::cerr << "  geometry           Build 3D geometry from surface JSON\n";
    std::cerr << "  obj                Export geometry JSON to OBJ format\n";
    std::cerr << "\n";
    std::cerr << "Options:\n";
    std::cerr << "  -o, --output FILE   Output file (required)\n";
    std::cerr << "  -c, --config FILE   Load configuration from FILE (surface command only)\n";
    std::cerr << "  -v, --verbose       Enable verbose logging\n";
    std::cerr << "  -h, --help          Show this help message\n";
    std::cerr << "\n";
    std::cerr << "Examples:\n";
    std::cerr << "  # Run full pipeline step by step:\n";
    std::cerr << "  " << program_name << " parse pattern.txt -o pattern.instructions.json\n";
    std::cerr << "  " << program_name << " stitch pattern.instructions.json -o pattern.stitch.json\n";
    std::cerr << "  " << program_name << " yarn pattern.stitch.json -o pattern.yarn.json\n";
    std::cerr << "  " << program_name << " surface pattern.yarn.json -o pattern.surface.json\n";
    std::cerr << "  " << program_name << " geometry pattern.surface.json -o pattern.geometry.json\n";
    std::cerr << "  " << program_name << " obj pattern.geometry.json -o pattern.obj\n";
    std::cerr << "\n";
    std::cerr << "  # With custom configuration:\n";
    std::cerr << "  " << program_name << " surface pattern.yarn.json -o pattern.surface.json -c custom.json\n";
    std::cerr << "\n";
    std::cerr << "Environment:\n";
    std::cerr << "  YARNPATH_LOG_LEVEL - Set log level (trace, debug, info, warn, error)\n";
}

int main(int argc, char* argv[]) {
    auto log = yarnpath::logging::get_logger();

    if (argc < 2) {
        print_usage(argv[0]);
        return 1;
    }

    std::string command = argv[1];

    // Handle help
    if (command == "-h" || command == "--help" || command == "help") {
        print_usage(argv[0]);
        return 0;
    }

    // Dispatch to command
    try {
        if (command == "generate-makefile") {
            return yarnpath::cli::command_makefile(argc, argv);
        } else if (command == "parse") {
            return yarnpath::cli::command_parse(argc, argv);
        } else if (command == "stitch") {
            return yarnpath::cli::command_stitch(argc, argv);
        } else if (command == "yarn") {
            return yarnpath::cli::command_yarn(argc, argv);
        } else if (command == "surface") {
            return yarnpath::cli::command_surface(argc, argv);
        } else if (command == "geometry") {
            return yarnpath::cli::command_geometry(argc, argv);
        } else if (command == "obj") {
            return yarnpath::cli::command_obj(argc, argv);
        } else {
            std::cerr << "Unknown command: " << command << "\n";
            std::cerr << "Run '" << argv[0] << " --help' for usage information\n";
            return 1;
        }
    } catch (const std::exception& e) {
        log->error("Fatal error: {}", e.what());
        std::cerr << "Fatal error: " << e.what() << "\n";
        return 1;
    }
}
