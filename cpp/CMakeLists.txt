cmake_minimum_required(VERSION 2.8)

set(CMAKE_CXX_STANDARD 11)

set(EXECUTABLE_OUTPUT_PATH "${PROJECT_SOURCE_DIR}/bin")
set(LIBRARY_OUTPUT_PATH "${PROJECT_SOURCE_DIR}/lib")

add_subdirectory("thirdparty/glfw-3.1.2")

include_directories(
	"${PROJECT_SOURCE_DIR}/src"
	"${PROJECT_SOURCE_DIR}/thirdparty"
	"${PROJECT_SOURCE_DIR}/thirdparty/glfw-3.1.2/include"
)

add_subdirectory("src/CubeMoveIcons")
add_subdirectory("src/CubeRecognition")
add_subdirectory("src/GraphicalCube")
add_subdirectory("src/SimpleSolver")
add_subdirectory("src/SolverLib")
