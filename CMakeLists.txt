cmake_minimum_required(VERSION 3.20)
project(yarnpath VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Compiler warnings
add_compile_options(-Wall -Wextra -Wpedantic)

# Libraries
add_subdirectory(stitch_graph)
add_subdirectory(yarn_path)
add_subdirectory(geometry)

# Application
add_subdirectory(app)

# Testing
enable_testing()
add_subdirectory(tests)
