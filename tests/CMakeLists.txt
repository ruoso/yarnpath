add_executable(tests
    test_instructions.cpp
    test_stitch_graph.cpp
    test_parser.cpp
    test_yarn_path.cpp
    test_geometry.cpp
    test_loop_shape.cpp
)

target_link_libraries(tests PRIVATE
    stitch_graph
    instructions
    parser
    yarn_path
    geometry
    GTest::gtest_main
)

gtest_discover_tests(tests)
