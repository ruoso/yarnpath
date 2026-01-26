add_library(visualizer
    visualizer.hpp
    visualizer.cpp
)

target_include_directories(visualizer PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(visualizer PUBLIC surface geometry common)

# Visualization requires GLFW and OpenGL
if(YARNPATH_HAS_VISUALIZATION)
    target_compile_definitions(visualizer PUBLIC YARNPATH_HAS_VISUALIZATION)
    target_link_libraries(visualizer PUBLIC glfw OpenGL::GL)
endif()
