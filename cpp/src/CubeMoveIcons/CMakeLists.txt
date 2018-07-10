add_executable(GenerateMoveIcons
    MoveIcons.cpp
)

target_link_libraries(GenerateMoveIcons
    ${OpenCV_LIBS}
)

add_executable(letters2arrows
    letters2arrows.cpp
)

target_link_libraries(letters2arrows
    SolverLib
)
