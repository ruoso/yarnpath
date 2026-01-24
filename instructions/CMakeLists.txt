add_library(instructions
    stitch_instruction.hpp
    stitch_instruction.cpp
    row_instruction.hpp
    row_instruction.cpp
)

target_include_directories(instructions PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
