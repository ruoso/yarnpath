# CLI commands library
add_library(yarnpath_cli
    command_parse.cpp
    command_stitch.cpp
    command_yarn.cpp
    command_surface.cpp
    command_geometry.cpp
    command_obj.cpp
    command_makefile.cpp
)

target_include_directories(yarnpath_cli PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/..
)

target_link_libraries(yarnpath_cli PUBLIC
    common
    parser
    stitch_graph
    yarn_path
    surface
    geometry
    yarnpath_serialization
    nlohmann_json::nlohmann_json
)
