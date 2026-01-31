add_executable(yarnpath
    main.cpp
)

target_link_libraries(yarnpath PRIVATE
    yarnpath_cli
    yarnpath_serialization
    geometry
    surface
    visualizer
    parser
    common
)
