add_library(math
    vec3.hpp
    cubic_bezier.hpp
    cubic_bezier.cpp
    curvature_utils.hpp
    curvature_utils.cpp
)
target_include_directories(math PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/..)
target_link_libraries(math PUBLIC common)
