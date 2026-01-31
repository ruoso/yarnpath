# Serialization library for JSON I/O
add_library(yarnpath_serialization INTERFACE)

target_include_directories(yarnpath_serialization INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/..
)

target_link_libraries(yarnpath_serialization INTERFACE
    nlohmann_json::nlohmann_json
    math
    yarn
    instructions
    stitch_graph
    yarn_path
    surface
    geometry
)
