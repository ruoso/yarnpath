add_library(yarn_path
    yarn_path.hpp
    yarn_path.cpp
)

target_include_directories(yarn_path PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(yarn_path PUBLIC stitch_graph common)
