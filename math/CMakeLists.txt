add_library(math
    vec3.hpp
    catmull_rom_spline.hpp
    catmull_rom_spline.cpp
)
target_include_directories(math PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/..)
target_link_libraries(math PUBLIC common)
