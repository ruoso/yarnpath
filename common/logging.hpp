#pragma once

#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <cstdlib>
#include <memory>
#include <string>

namespace yarnpath {
namespace logging {

inline std::shared_ptr<spdlog::logger> get_logger() {
    static std::shared_ptr<spdlog::logger> logger = []() {
        auto log = spdlog::stderr_color_mt("yarnpath");
        log->set_pattern("[%H:%M:%S.%e] [%^%l%$] %v");

        // Set log level from environment variable
        const char* level_env = std::getenv("YARNPATH_LOG_LEVEL");
        if (level_env) {
            std::string level(level_env);
            if (level == "trace") {
                log->set_level(spdlog::level::trace);
            } else if (level == "debug") {
                log->set_level(spdlog::level::debug);
            } else if (level == "info") {
                log->set_level(spdlog::level::info);
            } else if (level == "warn") {
                log->set_level(spdlog::level::warn);
            } else if (level == "error") {
                log->set_level(spdlog::level::err);
            } else if (level == "off") {
                log->set_level(spdlog::level::off);
            }
        } else {
            log->set_level(spdlog::level::info);
        }

        return log;
    }();
    return logger;
}

}  // namespace logging
}  // namespace yarnpath
