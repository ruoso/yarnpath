add_library(geometry
    geometry.hpp
    geometry.cpp
)

target_include_directories(geometry PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(geometry PUBLIC yarn_path)
