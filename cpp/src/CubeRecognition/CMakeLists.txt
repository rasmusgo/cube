add_executable(CubeRecognition
    ConnectLabels.cpp
    ConnectLabels.hpp
    CreateLabels.cpp
    CreateLabels.hpp
    cube_recognition_main.cpp
    Image.hpp
    Label.cpp
    Label.hpp
    Settings.hpp
    SolveCamera.cpp
    SolveCamera.hpp
    VectorMath.hpp
)

target_link_libraries(CubeRecognition
    ${OpenCV_LIBS}
)
