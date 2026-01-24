add_executable(tests
    main.cpp
)

target_link_libraries(tests PRIVATE geometry)

add_test(NAME tests COMMAND tests)
