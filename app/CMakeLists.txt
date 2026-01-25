add_executable(yarnpath
    main.cpp
)

target_link_libraries(yarnpath PRIVATE geometry surface parser common)
