add_library(parser
    token.hpp
    tokenizer.hpp
    tokenizer.cpp
    ast.hpp
    parser.hpp
    parser.cpp
    emitter.hpp
    emitter.cpp
)

target_include_directories(parser PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(parser PUBLIC instructions)
