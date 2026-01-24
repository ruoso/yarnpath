add_library(geometry
    geometry.hpp
    geometry.cpp
    vec3.hpp
    yarn_properties.hpp
    gauge.hpp
    cubic_bezier.hpp
    cubic_bezier.cpp
    fabric_surface.hpp
    geometry_path.hpp
    geometry_path.cpp
    geometry_builder.hpp
    geometry_builder.cpp
    physical_loop.hpp
    physical_loop.cpp
)

target_include_directories(geometry PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(geometry PUBLIC yarn_path common)
