cmake_minimum_required(VERSION 3.20)
project(yarnpath VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Compiler warnings
add_compile_options(-Wall -Wextra -Wpedantic)

# spdlog for logging
include(FetchContent)
FetchContent_Declare(
    spdlog
    GIT_REPOSITORY https://github.com/gabime/spdlog.git
    GIT_TAG v1.14.1
)
FetchContent_MakeAvailable(spdlog)

# Common utilities (logging)
add_subdirectory(common)

# Libraries
add_subdirectory(instructions)
add_subdirectory(parser)
add_subdirectory(stitch_graph)
add_subdirectory(yarn_path)
add_subdirectory(geometry)

# Application
add_subdirectory(app)

# Testing
enable_testing()
FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.14.0
)
FetchContent_MakeAvailable(googletest)
include(GoogleTest)
add_subdirectory(tests)
