add_executable(CubeRecognition
    CreateLabels.cpp
    CreateLabels.hpp
    cube_recognition_main.cpp
    Image.hpp
    Label.cpp
    Label.hpp
    Settings.hpp
    VectorMath.hpp
)

target_link_libraries(CubeRecognition
    ${OpenCV_LIBS}
)
