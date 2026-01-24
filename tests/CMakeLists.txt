add_executable(tests
    main.cpp
    test_instructions.cpp
    test_stitch_graph.cpp
)

target_link_libraries(tests PRIVATE stitch_graph instructions)

add_test(NAME tests COMMAND tests)
