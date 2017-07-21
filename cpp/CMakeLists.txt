cmake_minimum_required(VERSION 2.8)

project(Cubing)

set(CMAKE_CXX_STANDARD 11)

set(EXECUTABLE_OUTPUT_PATH "${PROJECT_SOURCE_DIR}/bin")
set(LIBRARY_OUTPUT_PATH "${PROJECT_SOURCE_DIR}/lib")

add_subdirectory("thirdparty/glfw-3.1.2" EXCLUDE_FROM_ALL)

include_directories(
    "${PROJECT_SOURCE_DIR}/src"
    "${PROJECT_SOURCE_DIR}/thirdparty/glfw-3.1.2/include"
)

find_package(OpenCV REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )

add_subdirectory("src/CubeMoveIcons")
add_subdirectory("src/CubeRecognition")
add_subdirectory("src/GraphicalCube")
add_subdirectory("src/SimpleSolver")
add_subdirectory("src/SolverLib")
