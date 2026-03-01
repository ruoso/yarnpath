add_library(stitch_shape
    stitch_enums.hpp
    loop_dimensions.hpp
    loop_dimensions.cpp
    stitch_shape.hpp
    stitch_shape.cpp
)

target_include_directories(stitch_shape PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/..)
target_link_libraries(stitch_shape PUBLIC yarn math)
