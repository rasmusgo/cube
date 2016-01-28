cmake_minimum_required(VERSION 2.8)

set(EXECUTABLE_OUTPUT_PATH "${PROJECT_SOURCE_DIR}/bin")
set(LIBRARY_OUTPUT_PATH "${PROJECT_SOURCE_DIR}/lib")

include_directories("${PROJECT_SOURCE_DIR}/src")

add_subdirectory(src/GraphicalCube)
add_subdirectory(src/SimpleSolver)
add_subdirectory(src/SolverLib)
