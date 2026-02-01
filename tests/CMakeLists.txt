add_executable(tests
    test_instructions.cpp
    test_stitch_graph.cpp
    test_parser.cpp
    test_yarn_path.cpp
    test_yarn_topology_distinctions.cpp
    test_loop_shape.cpp
    test_vec3.cpp
    test_bezier.cpp
    test_yarn_properties.cpp
    test_geometry_path.cpp
    test_continuity.cpp
    test_yarn_interlocking.cpp
    test_topology_geometry.cpp
    test_surface_graph.cpp
    test_surface_forces.cpp
    test_surface_solver.cpp
    test_surface_builder.cpp
)

target_link_libraries(tests PRIVATE
    stitch_graph
    instructions
    parser
    yarn_path
    geometry
    surface
    GTest::gtest_main
)

gtest_discover_tests(tests)
