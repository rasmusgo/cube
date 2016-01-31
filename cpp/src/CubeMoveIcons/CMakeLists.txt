add_executable(GenerateMoveIcons
    MoveIcons.cpp
)

target_link_libraries(GenerateMoveIcons
    X11
    pthread
)
