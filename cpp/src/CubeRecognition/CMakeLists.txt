add_executable(CubeRecognition
	createlabels.cpp
	createlabels.dep
	createlabels.h
	CubeRecognition.cpp
	image.h
	Label.cpp
	Label.h
	settings.h
	vectormath.h
)

target_link_libraries(CubeRecognition
	X11
	pthread
)
