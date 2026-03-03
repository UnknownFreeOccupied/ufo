add_executable(VisionTests)

target_sources(VisionTests
	PRIVATE
		camera_test.cpp
		color_test.cpp
		image_test.cpp
)

set_target_properties(VisionTests
	PROPERTIES
		VERSION ${PROJECT_VERSION}
		SOVERSION ${PROJECT_VERSION_MAJOR}
		CXX_STANDARD 23
		CXX_EXTENSIONS OFF
		CXX_SCAN_FOR_MODULES OFF
)

target_link_libraries(VisionTests PRIVATE Catch2::Catch2WithMain UFO::Vision UFO::Warnings)

catch_discover_tests(VisionTests)