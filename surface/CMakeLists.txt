add_library(surface
    surface.hpp
    surface.cpp
    surface_graph.hpp
    surface_graph.cpp
    surface_node.hpp
    surface_edge.hpp
    surface_constraint.hpp
    surface_builder.hpp
    surface_builder.cpp
    surface_forces.hpp
    surface_forces.cpp
    surface_solver.hpp
    surface_solver.cpp
    surface_visualizer.hpp
    surface_visualizer.cpp
)

target_include_directories(surface PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(surface PUBLIC math yarn yarn_path common)

# Optional visualization support
if(YARNPATH_HAS_VISUALIZATION)
    target_compile_definitions(surface PUBLIC YARNPATH_HAS_VISUALIZATION)
    target_link_libraries(surface PUBLIC glfw OpenGL::GL)
endif()
