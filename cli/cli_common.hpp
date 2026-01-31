#ifndef YARNPATH_CLI_COMMON_HPP
#define YARNPATH_CLI_COMMON_HPP

#include <string>
#include <optional>
#include <iostream>
#include <fstream>
#include <sstream>

namespace yarnpath::cli {

// Common context for all CLI commands
struct CommandContext {
    std::string input_path;
    std::string output_path;
    std::optional<std::string> config_path;
    bool verbose = false;
};

// Parse common arguments from command line
// Returns the context and the index of the first unprocessed argument
inline std::pair<CommandContext, int> parse_common_args(int argc, char** argv, int start_idx) {
    CommandContext ctx;
    int i = start_idx;

    // Parse flags and positional arguments
    while (i < argc) {
        std::string arg = argv[i];

        if (arg == "-v" || arg == "--verbose") {
            ctx.verbose = true;
            ++i;
        } else if (arg == "-o" || arg == "--output") {
            if (i + 1 < argc) {
                ctx.output_path = argv[++i];
                ++i;
            } else {
                throw std::runtime_error("-o/--output requires an argument");
            }
        } else if (arg == "-c" || arg == "--config") {
            if (i + 1 < argc) {
                ctx.config_path = argv[++i];
                ++i;
            } else {
                throw std::runtime_error("-c/--config requires an argument");
            }
        } else if (arg == "-h" || arg == "--help") {
            // Handled by caller
            ++i;
        } else if (arg[0] != '-') {
            // Positional argument (input file)
            if (ctx.input_path.empty()) {
                ctx.input_path = arg;
                ++i;
            } else {
                throw std::runtime_error("Unexpected positional argument: " + arg);
            }
        } else {
            throw std::runtime_error("Unknown option: " + arg);
        }
    }

    return {ctx, i};
}

// Resolve output path: if empty, generate from input path with given suffix
inline std::string resolve_output_path(const std::string& input,
                                       const std::string& suffix,
                                       const std::string& provided_output) {
    if (!provided_output.empty()) {
        return provided_output;
    }

    // Find last dot in input path
    size_t dot_pos = input.find_last_of('.');
    size_t slash_pos = input.find_last_of('/');

    // Make sure dot comes after last slash (if any)
    if (dot_pos != std::string::npos &&
        (slash_pos == std::string::npos || dot_pos > slash_pos)) {
        return input.substr(0, dot_pos) + suffix;
    } else {
        return input + suffix;
    }
}

// Read entire file to string
inline std::string read_file(const std::string& path) {
    std::ifstream file(path);
    if (!file) {
        throw std::runtime_error("Cannot open file: " + path);
    }
    std::stringstream buffer;
    buffer << file.rdbuf();
    return buffer.str();
}

// Write string to file
inline void write_file(const std::string& path, const std::string& content) {
    std::ofstream file(path);
    if (!file) {
        throw std::runtime_error("Cannot write to file: " + path);
    }
    file << content;
}

// Command function declarations
int command_parse(int argc, char** argv);
int command_stitch(int argc, char** argv);
int command_yarn(int argc, char** argv);
int command_surface(int argc, char** argv);
int command_geometry(int argc, char** argv);
int command_obj(int argc, char** argv);
int command_makefile(int argc, char** argv);

}  // namespace yarnpath::cli

#endif // YARNPATH_CLI_COMMON_HPP
