add_executable(tests
    test_instructions.cpp
    test_stitch_graph.cpp
    test_parser.cpp
)

target_link_libraries(tests PRIVATE
    stitch_graph
    instructions
    parser
    GTest::gtest_main
)

gtest_discover_tests(tests)
