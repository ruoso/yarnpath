cmake_minimum_required(VERSION 3.20)
project(yarnpath VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Compiler warnings
add_compile_options(-Wall -Wextra -Wpedantic)

# Find OpenMP for parallel surface relaxation
find_package(OpenMP)
if(OpenMP_CXX_FOUND)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    message(STATUS "OpenMP found - parallel surface relaxation enabled")
else()
    message(WARNING "OpenMP not found - surface relaxation will run single-threaded")
endif()

# spdlog for logging
include(FetchContent)
FetchContent_Declare(
    spdlog
    GIT_REPOSITORY https://github.com/gabime/spdlog.git
    GIT_TAG v1.14.1
)
FetchContent_MakeAvailable(spdlog)

# nlohmann/json for serialization
FetchContent_Declare(
    nlohmann_json
    GIT_REPOSITORY https://github.com/nlohmann/json.git
    GIT_TAG v3.11.3
)
FetchContent_MakeAvailable(nlohmann_json)

# GLFW for OpenGL windowing (optional visualization)
find_package(glfw3 QUIET)
find_package(OpenGL QUIET)
if(glfw3_FOUND AND OpenGL_FOUND)
    set(YARNPATH_HAS_VISUALIZATION ON)
    message(STATUS "Visualization enabled (GLFW + OpenGL found)")
else()
    set(YARNPATH_HAS_VISUALIZATION OFF)
    message(STATUS "Visualization disabled (GLFW or OpenGL not found)")
endif()

# Common utilities (logging)
add_subdirectory(common)

# Shared type modules
add_subdirectory(math)
add_subdirectory(yarn)

# Serialization
add_subdirectory(serialization)

# Libraries
add_subdirectory(instructions)
add_subdirectory(parser)
add_subdirectory(stitch_graph)
add_subdirectory(yarn_path)
add_subdirectory(geometry)
add_subdirectory(surface)
add_subdirectory(visualizer)

# CLI commands
add_subdirectory(cli)

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
