add_executable(GraphicalCube
	graphical_cube_main.cpp
	GraphicalCube.cpp
	GraphicalCube.hpp
	Sequence.hpp
)

target_link_libraries(GraphicalCube
	SolverLib
	glfw
	${GLFW_LIBRARIES}
)
