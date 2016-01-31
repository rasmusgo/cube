add_executable(CubeRecognition
	CreateLabels.cpp
	CreateLabels.hpp
	CubeRecognition.cpp
	Image.hpp
	Label.cpp
	Label.hpp
	Settings.hpp
	VectorMath.hpp
)

target_link_libraries(CubeRecognition
	X11
	pthread
)
