add_library(stitch_graph
    stitch_node.hpp
    stitch_node.cpp
)

target_include_directories(stitch_graph PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
