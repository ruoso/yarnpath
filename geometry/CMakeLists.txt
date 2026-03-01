add_library(geometry
    geometry.hpp
    geometry.cpp
    geometry_path.hpp
    geometry_path.cpp
    geometry_builder.hpp
    geometry_builder.cpp
    geometry_build_state.hpp
    geometry_build_state.cpp
    loop_precompute.hpp
    loop_precompute.cpp
    position_resolver.hpp
    position_resolver.cpp
    geometry_validator.hpp
    geometry_validator.cpp
    crossover_geometry.hpp
    crossover_geometry.cpp
    physical_loop.hpp
    physical_loop.cpp
)

target_include_directories(geometry PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(geometry PUBLIC yarn_path common math yarn surface stitch_shape)
