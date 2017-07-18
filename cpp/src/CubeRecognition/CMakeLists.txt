add_library(CubeRecognitionLib
    AssignColors.cpp
    AssignColors.hpp
    ConnectLabelContours.cpp
    ConnectLabelContours.hpp
    FindLabelContours.cpp
    FindLabelContours.hpp
    FindLabels.cpp
    FindLabels.hpp
    Image.hpp
    LabelContour.cpp
    LabelContour.hpp
    MatrixTools.cpp
    MatrixTools.hpp
    ObserveLabels.cpp
    ObserveLabels.hpp
    ProbabalisticCube.cpp
    ProbabalisticCube.hpp
    Settings.hpp
    SolveCamera.cpp
    SolveCamera.hpp
    Timer.cpp
    Timer.hpp
    VectorMath.hpp
)

target_link_libraries(CubeRecognitionLib
    SolverLib
    ${OpenCV_LIBS}
)

add_executable(CubeRecognition
    cube_recognition_main.cpp
)

target_link_libraries(CubeRecognition
    CubeRecognitionLib
)

add_executable(CubeRecognitionTest
    cube_recognition_test.cpp
)

target_link_libraries(CubeRecognitionTest
    CubeRecognitionLib
)
