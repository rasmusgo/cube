add_executable(GraphicalCube
	GraphicalCube.cpp
	GraphicalCube.h
	Sequence.h
	WinMain.cpp
)

target_link_libraries(GraphicalCube
	SolverLib
)
