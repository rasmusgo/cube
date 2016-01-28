add_executable(GraphicalCube
	GraphicalCube.cpp
	GraphicalCube.h
	Sequence.h
	main.cpp
)

target_link_libraries(GraphicalCube
	SolverLib
	glfw
	${GLFW_LIBRARIES}
)
