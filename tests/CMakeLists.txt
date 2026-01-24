add_executable(tests
    test_instructions.cpp
    test_stitch_graph.cpp
    test_parser.cpp
    test_yarn_path.cpp
    test_loop_shape.cpp
    test_vec3.cpp
    test_bezier.cpp
    test_yarn_properties.cpp
    test_fabric_surface.cpp
    test_geometry_path.cpp
    test_continuity.cpp
    test_yarn_interlocking.cpp
    test_topology_geometry.cpp
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
